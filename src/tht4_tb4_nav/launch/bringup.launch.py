"""Assignment workflow: teleop mapping, then saved-map navigation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include_launch(package, filename, arguments, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(package) + '/launch/' + filename),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='maze'),
        DeclareLaunchArgument('ign_ip', default_value='127.0.0.1',
                              description='Gazebo Transport interface IP (local simulation)'),
        SetEnvironmentVariable('IGN_IP', LaunchConfiguration('ign_ip')),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'rviz_config', default_value=get_package_share_directory('tht4_tb4_nav')
            + '/rviz/maze.rviz'),
        DeclareLaunchArgument('mode', default_value='mapping',
                              choices=['mapping', 'navigation']),
        DeclareLaunchArgument('map', default_value='',
                              description='Saved map YAML required for navigation'),
        DeclareLaunchArgument(
            'slam_params', default_value=get_package_share_directory('tht4_tb4_nav')
            + '/config/slam.yaml'),
        OpaqueFunction(function=launch_workflow),
    ])


def launch_workflow(context):
    mode = LaunchConfiguration('mode').perform(context)
    # Capture outer RViz settings before the simulator sets its own rviz=false.
    # Keep simulator launch configurations alive for its delayed controller nodes.
    start_rviz = LaunchConfiguration('rviz').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    map_path = None
    if mode == 'navigation':
        value = LaunchConfiguration('map').perform(context)
        if not value:
            raise RuntimeError('Navigation requires map:=/absolute/path/to/map.yaml')
        map_path = Path(value).expanduser().resolve()
        if not map_path.is_file():
            raise RuntimeError(f'Saved map does not exist: {map_path}')

    actions = [
        include_launch(
            'turtlebot4_ignition_bringup', 'turtlebot4_ignition.launch.py',
            {'world': LaunchConfiguration('world'),
             'rviz': 'false', 'slam': 'false',
             'localization': 'false', 'nav2': 'false'}),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(start_rviz)),
    ]
    if mode == 'mapping':
        actions.append(GroupAction(actions=[include_launch(
            'turtlebot4_navigation', 'slam.launch.py',
            {'use_sim_time': 'true', 'sync': 'false',
             'params': LaunchConfiguration('slam_params')})], scoped=True))
    else:
        actions.extend([
            GroupAction(actions=[include_launch(
                'turtlebot4_navigation', 'localization.launch.py',
                {'use_sim_time': 'true', 'map': str(map_path)})], scoped=True),
            GroupAction(actions=[include_launch(
                'turtlebot4_navigation', 'nav2.launch.py',
                {'use_sim_time': 'true'})], scoped=True),
        ])
    return actions
