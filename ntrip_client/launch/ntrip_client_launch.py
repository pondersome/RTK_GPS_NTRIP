from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import EnvironmentVariable
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
          # Declare arguments for namespace and other parameters
          namespace_arg = DeclareLaunchArgument('namespace', default_value='ntrip_client', description='Namespace eg. mybot')
          group_arg = DeclareLaunchArgument('group', default_value='', description='Group name. eg. rtk')
          
          # Declare arguments with default values
          # RTCM mavros_msgs
          launch_arguments = [
            namespace_arg,
            group_arg,
            DeclareLaunchArgument('node_name',             default_value='ntrip_client'),
            DeclareLaunchArgument('host',                  default_value=''),
            DeclareLaunchArgument('port',                  default_value='2101'),
            DeclareLaunchArgument('mountpoint',            default_value=''),
            DeclareLaunchArgument('ntrip_version',         default_value='None'),
            DeclareLaunchArgument('user_agent',            default_value='NTRIP ponderbotics_ntrip_client', description='HTTP User-Agent sent to the caster. Must start with "NTRIP ". rtk2go blocks the stock "NTRIP ntrip_client_ros".'),
            DeclareLaunchArgument('ntrip_server_hz',       default_value='10'), # set to 1 for rtk2go
            DeclareLaunchArgument('authenticate',          default_value=''),
            DeclareLaunchArgument('username',              default_value=''),
            DeclareLaunchArgument('password',              default_value='none'),
            DeclareLaunchArgument('ssl',                   default_value='False'),
            DeclareLaunchArgument('cert',                  default_value='None'),
            DeclareLaunchArgument('key',                   default_value='None'),
            DeclareLaunchArgument('ca_cert',               default_value='None'),
            DeclareLaunchArgument('debug',                 default_value='false'),
            DeclareLaunchArgument('rtcm_message_package',  default_value='rtcm_msgs'),
            DeclareLaunchArgument('reconnect_attempt_wait_max_seconds', default_value='120', description='Ceiling for the exponential reconnect backoff. Retries are persistent (never give up); raise this to reduce footprint during long caster outages (e.g. 600 for rtk2go DDoS/maintenance downtime).'),
          ]


          rtk_group = GroupAction([
              PushRosNamespace(LaunchConfiguration('namespace')),
              PushRosNamespace(LaunchConfiguration('group')),

            # ******************************************************************
            # NTRIP Client Node
            # ******************************************************************
            Node(
                  name=LaunchConfiguration('node_name'),
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

                      # User-Agent presented to the caster. Must start with "NTRIP ".
                      # rtk2go blocks the stock "NTRIP ntrip_client_ros" and refuses such clients with a sourcetable response.
                      'user_agent': LaunchConfiguration('user_agent'),
                      
                      # Rate to request correction messages. Some servers will sandbox clients that request too often
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
                      'key':  LaunchConfiguration('key'),

                      # If the NTRIP caster uses self signed certs, or you need to use a different CA chain, specify the path to the file here
                      'ca_cert': LaunchConfiguration('ca_cert'),

                      # Not sure if this will be looked at by other ndoes, but this frame ID will be added to the RTCM messages published by this node
                      'rtcm_frame_id': 'odom',

                      # Optional parameters that will allow for longer or shorter NMEA messages. Standard max length for NMEA is 82
                      'nmea_max_length': 100,
                      'nmea_min_length': 3,

                      # Use this parameter to change the type of RTCM message published by the node. Defaults to "mavros_msgs", but we also support "rtcm_msgs"
                      'rtcm_message_package': LaunchConfiguration('rtcm_message_package'),

                      # Reconnect backoff: wait doubles from reconnect_attempt_wait_seconds up to
                      # reconnect_attempt_wait_max_seconds, then holds at that ceiling. Retries are
                      # persistent (no give-up) so the node recovers from long caster outages on its own.
                      'reconnect_attempt_wait_seconds': 10, #was 5, changed per rtk2go reqs
                      'reconnect_attempt_wait_max_seconds': LaunchConfiguration('reconnect_attempt_wait_max_seconds'),

                      # How many seconds is acceptable in between receiving RTCM. If RTCM is not received for this duration, the node will attempt to reconnect
                      'rtcm_timeout_seconds': 10 #was 4 changed for rtk2go reqs
                    }
                  ],
                  # Uncomment the following section and replace "/gq7/nmea/sentence" with the topic you are sending NMEA on if it is not the one we requested
                  #remappings=[
                  #  ("/ntrip_client/nmea", "/ublox_gps_node/fix")
                  #],
              ) #end Node
          ]) #end Group Action
          
          # Set the environment variable for debugging if needed
          set_debug_env = SetEnvironmentVariable(
              name='NTRIP_CLIENT_DEBUG', value=LaunchConfiguration('debug')
          )       
          
          return LaunchDescription(launch_arguments + [
                  set_debug_env,
                  rtk_group        
          ])
