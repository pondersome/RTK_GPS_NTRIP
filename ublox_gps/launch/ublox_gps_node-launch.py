from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os
import ament_index_python.packages

def generate_launch_description():
    # Declare arguments for namespace and other parameters
    namespace_arg = DeclareLaunchArgument('namespace', default_value='', description='Namespace for the node')
    group_arg = DeclareLaunchArgument('group', default_value='', description='Group name. eg. rtk')

    # Construct the path to the config directory
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ublox_gps'),
        'config'
    )
    params = os.path.join(config_directory, 'zed_f9p.yaml')

    # Group nodes under a namespace
    ublox_gps_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        PushRosNamespace(LaunchConfiguration('group')),
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            output='both',
            parameters=[params]
        ),
    ])

    return LaunchDescription([
        namespace_arg,
        group_arg,
        ublox_gps_group
    ])
