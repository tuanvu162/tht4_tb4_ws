"""Run semantic visualization and voice navigation, optionally with the simulator."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('tht7_tb4')
    nav_share = get_package_share_directory('tht4_tb4_nav')
    common = {
        'map_file': LaunchConfiguration('map'),
        'regions_file': LaunchConfiguration('regions'),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'use_nvidia', default_value='true', choices=['true', 'false'],
            description='Use NVIDIA PRIME render offload for Gazebo and RViz'),
        # Set these before including the simulator so its GUI and GPU sensors
        # inherit the same renderer selection as RViz. Use XWayland on Wayland.
        *[SetEnvironmentVariable(name, value,
                                 condition=IfCondition(LaunchConfiguration('use_nvidia')))
          for name, value in (
              ('__NV_PRIME_RENDER_OFFLOAD', '1'),
              ('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
              ('__VK_LAYER_NV_optimus', 'NVIDIA_only'),
              ('QT_QPA_PLATFORM', 'xcb'),
          )],
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('map', default_value=share + '/config/map.yaml'),
        DeclareLaunchArgument('regions', default_value=share + '/config/regions.yaml'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('world', default_value='maze'),
        DeclareLaunchArgument(
            'nav2_params', default_value=get_package_share_directory('turtlebot4_navigation')
            + '/config/nav2.yaml'),
        DeclareLaunchArgument('goal_timeout', default_value='180.0'),
        # Delayed controller spawners still need the simulator's launch settings.
        # A scoped group would remove namespace before their callbacks execute.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_share + '/launch/bringup.launch.py'),
            launch_arguments={
                'mode': 'navigation', 'map': LaunchConfiguration('map'),
                'world': LaunchConfiguration('world'), 'rviz': LaunchConfiguration('rviz'),
                'rviz_config': share + '/rviz/semantic.rviz',
                'nav2_params': LaunchConfiguration('nav2_params'),
            }.items(), condition=IfCondition(LaunchConfiguration('start_sim'))),
        Node(package='tht7_tb4', executable='region_markers', output='screen',
             parameters=[common]),
        Node(package='tht7_tb4', executable='voice_navigation', output='screen',
             parameters=[common, {'goal_timeout': ParameterValue(
                 LaunchConfiguration('goal_timeout'), value_type=float)}]),
    ])
