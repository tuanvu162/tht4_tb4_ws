"""Keep simulator configuration available to delayed controller spawners."""

import importlib.util
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    PopLaunchConfigurations, PushLaunchConfigurations, SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import pytest


def load_launch():
    path = Path(__file__).parents[1] / 'launch/tht7.launch.py'
    spec = importlib.util.spec_from_file_location('tht7_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def expand_configuration(action, context, nodes):
    """Visit real nested launch files without starting processes or GUI apps."""
    condition = getattr(action, 'condition', None)
    if condition is not None and not condition.evaluate(context):
        return
    if isinstance(action, LaunchDescription):
        children = action.entities
    elif isinstance(action, Node):
        nodes.append(action)
        return
    elif isinstance(action, (
            GroupAction, IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument,
            SetLaunchConfiguration, PushLaunchConfigurations, PopLaunchConfigurations,
            SetEnvironmentVariable)):
        children = action.execute(context) or []
    else:
        return
    for child in children:
        expand_configuration(child, context, nodes)


@pytest.mark.parametrize('rviz', ['true', 'false'])
def test_simulator_namespace_survives_until_controller_callbacks(rviz):
    context = LaunchContext()
    context.launch_configurations.update(start_sim='true', rviz=rviz)
    nodes = []
    expand_configuration(load_launch().generate_launch_description(), context, nodes)
    # controller_manager/spawner evaluates this after the include has finished.
    assert LaunchConfiguration('namespace').perform(context) == ''
    assert LaunchConfiguration('world').perform(context) == 'maze'
    rviz_nodes = [node for node in nodes if node.node_package == 'rviz2']
    assert len(rviz_nodes) == int(rviz == 'true')


def test_existing_simulator_mode_does_not_start_another_simulator():
    context = LaunchContext()
    context.launch_configurations['start_sim'] = 'false'
    nodes = []
    expand_configuration(load_launch().generate_launch_description(), context, nodes)
    assert {node.node_package for node in nodes} == {'tht7_tb4'}
    assert len(nodes) == 2
