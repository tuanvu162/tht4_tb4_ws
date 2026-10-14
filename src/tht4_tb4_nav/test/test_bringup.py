"""Regression checks for simulator configuration lifetime and RViz selection."""

import importlib.util
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import PopLaunchConfigurations, PushLaunchConfigurations
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


def load_bringup():
    path = Path(__file__).parents[1] / 'launch' / 'bringup.launch.py'
    spec = importlib.util.spec_from_file_location('bringup', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def visit_configuration(action, context):
    # Expand includes/scopes without starting processes or event handlers.
    if isinstance(action, LaunchDescription):
        for child in action.entities:
            visit_configuration(child, context)
    elif isinstance(action, (
            GroupAction, IncludeLaunchDescription, DeclareLaunchArgument,
            SetLaunchConfiguration, PushLaunchConfigurations, PopLaunchConfigurations)):
        for child in action.execute(context) or []:
            visit_configuration(child, context)


def test_delayed_controller_keeps_namespace_and_rviz_stays_enabled():
    context = LaunchContext()
    context.launch_configurations.update(
        mode='mapping', map='', world='maze', rviz='true',
        rviz_config='/tmp/custom_maze.rviz')
    actions = load_bringup().launch_workflow(context)
    simulator = actions[0]
    assert isinstance(simulator, IncludeLaunchDescription)
    visit_configuration(simulator, context)
    # This lookup happens later, when the first controller spawner exits.
    assert LaunchConfiguration('namespace').perform(context) == ''
    assert LaunchConfiguration('rviz').perform(context) == 'false'
    rviz = next(action for action in actions if isinstance(action, Node))
    assert rviz.condition.evaluate(context)


def test_explicit_rviz_false_is_preserved():
    context = LaunchContext()
    context.launch_configurations.update(
        mode='mapping', map='', world='maze', rviz='false',
        rviz_config='/tmp/custom_maze.rviz')
    actions = load_bringup().launch_workflow(context)
    rviz = next(action for action in actions if isinstance(action, Node))
    assert not rviz.condition.evaluate(context)


def test_mapping_selects_async_slam_and_custom_config():
    context = LaunchContext()
    context.launch_configurations.update(
        mode='mapping', map='', world='maze', rviz='true',
        rviz_config='/tmp/custom_maze.rviz', slam_params='/tmp/custom_slam.yaml')
    actions = load_bringup().launch_workflow(context)
    slam_group = actions[-1]
    include = next(action for action in slam_group.execute(context)
                   if isinstance(action, IncludeLaunchDescription))
    arguments = {
        name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
        for name, value in include.launch_arguments
    }
    assert arguments['sync'] == 'false'
    assert arguments['params'] == '/tmp/custom_slam.yaml'
    assert arguments['use_sim_time'] == 'true'
