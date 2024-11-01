from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments for namespace and other parameters
    namespace_arg = DeclareLaunchArgument('namespace', default_value='/', description='Namespace for all nodes. eg. mybot')
    group_arg = DeclareLaunchArgument('group', default_value='/', description='Group name for all nodes. eg. rtk')

    # Construct the path to the config directory
    config_directory = os.path.join(get_package_share_directory('ublox_gps'), 'config')
    ublox_params_default = os.path.join(config_directory, 'zed_f9p.yaml')

    launch_arguments = [
        namespace_arg,
        group_arg,
        DeclareLaunchArgument('host', default_value='rtk2go.com', description='Use an NTRIP server with base stations near you'),
        DeclareLaunchArgument('port', default_value='2101', description='Port number for the NTRIP server'),
        DeclareLaunchArgument('mountpoint', default_value='VN1', description='Use a mount point near you'),
        DeclareLaunchArgument('ntrip_version', default_value='None', description='NTRIP version to use in the initial HTTP request'),
        DeclareLaunchArgument('ntrip_server_hz', default_value='1', description='RTK2GO will ban you if Hz > 1'),
        DeclareLaunchArgument('authenticate', default_value='True', description='Enable or disable authentication'),
        DeclareLaunchArgument('username', default_value='realemail@provider.com', description='Replace this with a real email address for authentication'),
        DeclareLaunchArgument('password', default_value='none', description='Password for authentication if needed'),
        DeclareLaunchArgument('ssl', default_value='False', description='Whether to connect with SSL'),
        DeclareLaunchArgument('cert', default_value='None', description='Path to SSL certificate'),
        DeclareLaunchArgument('key', default_value='None', description='Path to SSL key'),
        DeclareLaunchArgument('ca_cert', default_value='None', description='Path to CA certificate'),
        DeclareLaunchArgument('debug', default_value='false', description='Enable debugging'),
        DeclareLaunchArgument('rtcm_message_package', default_value='rtcm_msgs', description='RTCM message package type'),
        DeclareLaunchArgument('ublox_params', default_value=ublox_params_default, description='Path to the parameter file for ublox GPS node')
    ]

    # Group nodes under a namespace and group
    rtk_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        PushRosNamespace(LaunchConfiguration('group')),

        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            output='both',
            parameters=[ublox_params_default],
            remappings=[
                ('ntrip_client/nmea', 'nmea'),
                ('ntrip_client/rtcm', 'rtcm'),
                ('ublox_gps_node/fix', 'fix'),
                ('ublox_gps_node/fix_velocity', 'fix_velocity'),
                ('ublox_gps_node/navpvt', 'navpvt')
            ]
        ),
    

        # Launch NTRIP client node
        Node(
            name='ntrip_client_node',
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

                    # If authenticate is set to true, we will use these to authenticate with the server
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
            remappings=[
                ('/ntrip_client/nmea', 'nmea'),
                ('/ntrip_client/rtcm', 'rtcm')
            ]
        ),

        # Launch fix2nmea node
        Node(
            package='fix2nmea',
            executable='fix2nmea',
            name='fix2nmea',
            remappings=[
                ('ntrip_client/nmea', 'nmea'),
                ('ublox_gps_node/fix', 'fix')
            ]
        ),

    ]) #end of rtk_group

    # Set the environment variable for debugging if needed
    set_debug_env = SetEnvironmentVariable(
        name='NTRIP_CLIENT_DEBUG', value=LaunchConfiguration('debug')
    )

    return LaunchDescription(launch_arguments + [
        set_debug_env,
        rtk_group
    ])
