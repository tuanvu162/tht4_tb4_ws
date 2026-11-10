"""Start perception and optionally waypoints against an existing Nav2 session."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def spawn_scene_chair(context):
    """Spawn a named chair without making duplicates when YOLO is restarted."""
    config_path = Path(LaunchConfiguration('scene_file').perform(context)).expanduser()
    with config_path.open(encoding='utf-8') as stream:
        scene = yaml.safe_load(stream)
    chair = scene['chair']
    model_file = Path(chair['model_file']).expanduser().resolve()
    if not model_file.is_file():
        raise RuntimeError(
            f'Chair model missing: {model_file}. Download foldable_chair from Gazebo Fuel '
            'or set spawn_chair:=false to run YOLO without spawning objects.')
    pose = [float(chair.get(key, 0.0)) for key in ('x', 'y', 'z', 'yaw')]
    if not all(math.isfinite(value) for value in pose):
        raise ValueError('Chair pose must contain finite x, y, z and yaw values')
    return [Node(
        package='ros_gz_sim', executable='create', name='spawn_yolo_chair', output='screen',
        arguments=[
            '-world', str(scene.get('world', 'maze')),
            '-file', str(model_file), '-name', str(chair['name']),
            '-allow_renaming', 'false',
            '-x', str(pose[0]), '-y', str(pose[1]), '-z', str(pose[2]), '-Y', str(pose[3]),
        ],
        additional_env={'IGN_IP': str(scene.get('ign_ip', '127.0.0.1'))},
    )]


def generate_launch_description():
    share = get_package_share_directory('tht5_tb4_perception')
    return LaunchDescription([
        DeclareLaunchArgument('yolo_params', default_value=share + '/config/yolo.yaml'),
        DeclareLaunchArgument('goals_file', default_value=share + '/config/goals.yaml'),
        DeclareLaunchArgument('spawn_chair', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('scene_file', default_value=share + '/config/scene_objects.yaml'),
        DeclareLaunchArgument('start_navigation', default_value='false',
                              choices=['true', 'false']),
        OpaqueFunction(function=spawn_scene_chair,
                       condition=IfCondition(LaunchConfiguration('spawn_chair'))),
        Node(package='tht5_tb4_perception', executable='yolo_detector',
             name='yolo_detector', output='screen',
             parameters=[LaunchConfiguration('yolo_params')]),
        Node(package='tht5_tb4_perception', executable='waypoint_navigation',
             name='waypoint_navigation', output='screen',
             parameters=[LaunchConfiguration('goals_file')],
             condition=IfCondition(LaunchConfiguration('start_navigation'))),
    ])
