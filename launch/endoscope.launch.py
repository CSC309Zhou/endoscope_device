from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('endoscope_device'), 'config', 'endoscope.yaml'])),
        Node(package='endoscope_device', executable='endoscope_device_node',
             additional_env={
                 'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                 'FASTRTPS_DEFAULT_PROFILES_FILE': PathJoinSubstitution([
                     FindPackageShare('endoscope_device'), 'config', 'fastdds_local.xml']),
             },
             output='screen', parameters=[LaunchConfiguration('config')]),
    ])
