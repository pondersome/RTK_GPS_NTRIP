from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set up parameters for the ntrip client launch file to be overridden from the command line
    ntrip_client_arguments = [
        # RTCM mavros_msgs
        DeclareLaunchArgument('host', default_value='rtk2go.com'), # use an ntrip server with base stations near you
        DeclareLaunchArgument('port', default_value='2101'),
        DeclareLaunchArgument('mountpoint', default_value='VN1'), # use a mount point near you
        DeclareLaunchArgument('ntrip_version', default_value='None'),
        DeclareLaunchArgument('ntrip_server_hz', default_value='1'), # rtk2go will ban you if hz>1
        DeclareLaunchArgument('authenticate', default_value='True'),
        DeclareLaunchArgument('username', default_value='realemail@provider.com'), # replace this with a real email address
        DeclareLaunchArgument('password', default_value='none'),
        DeclareLaunchArgument('ssl', default_value='False'),
        DeclareLaunchArgument('cert', default_value='None'),
        DeclareLaunchArgument('key', default_value='None'),
        DeclareLaunchArgument('ca_cert', default_value='None'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('rtcm_message_package', default_value='rtcm_msgs')
    ]

    # Launch ublox GPS node
    ublox_launch_path = os.path.join(
        get_package_share_directory('ublox_gps'),
        'launch',
        'ublox_gps_node-launch.py'
    )
    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ublox_launch_path)
    )

    # Launch NTRIP client node
    ntrip_client_node = Node(
        name='ntrip_client_node',
        namespace='ntrip_client',
        package='ntrip_client',
        executable='ntrip_ros.py',
        parameters=[
            {
                # Required parameters used to connect to the NTRIP server
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'mountpoint': LaunchConfiguration('mountpoint'),

                # Optional parameter that will set the NTRIP version in the initial HTTP request to the NTRIP caster.
                'ntrip_version': LaunchConfiguration('ntrip_version'),

                # Frequency to request correction messages. Some servers will sandbox clients that request too often
                'ntrip_server_hz': LaunchConfiguration('ntrip_server_hz'),

                # If this is set to true, we will read the username and password and attempt to authenticate. If not, we will attempt to connect unauthenticated
                'authenticate': LaunchConfiguration('authenticate'),

                # If authenticate is set the true, we will use these to authenticate with the server
                'username': LaunchConfiguration('username'),
                'password': LaunchConfiguration('password'),

                # Whether to connect with SSL. cert, key, and ca_cert options will only take effect if this is true
                'ssl': LaunchConfiguration('ssl'),

                # If the NTRIP caster uses cert based authentication, you can specify the cert and keys to use with these options
                'cert': LaunchConfiguration('cert'),
                'key': LaunchConfiguration('key'),

                # If the NTRIP caster uses self signed certs, or you need to use a different CA chain, specify the path to the file here
                'ca_cert': LaunchConfiguration('ca_cert'),

                # Not sure if this will be looked at by other nodes, but this frame ID will be added to the RTCM messages published by this node
                'rtcm_frame_id': 'odom',

                # Optional parameters that will allow for longer or shorter NMEA messages. Standard max length for NMEA is 82
                'nmea_max_length': 100,
                'nmea_min_length': 3,

                # Use this parameter to change the type of RTCM message published by the node. Defaults to "mavros_msgs", but we also support "rtcm_msgs"
                'rtcm_message_package': LaunchConfiguration('rtcm_message_package'),

                # Will affect how many times the node will attempt to reconnect before exiting, and how long it will wait in between attempts when a reconnect occurs
                'reconnect_attempt_max': 10,
                'reconnect_attempt_wait_seconds': 5,

                # How many seconds is acceptable in between receiving RTCM. If RTCM is not received for this duration, the node will attempt to reconnect
                'rtcm_timeout_seconds': 4
            }
        ],
    )

    # Set the environment variable for debugging if needed
    set_debug_env = SetEnvironmentVariable(
        name='NTRIP_CLIENT_DEBUG', value=LaunchConfiguration('debug')
    )

    # Launch fix2nmea node
    fix2nmea_node = Node(
        package='fix2nmea',
        executable='fix2nmea',
        name='fix2nmea'
    )

    return LaunchDescription(ntrip_client_arguments + [
        ublox_launch,
        set_debug_env,
        ntrip_client_node,
        fix2nmea_node
    ])