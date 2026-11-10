"""Check spawn configuration without changing a running Gazebo world."""

import importlib.util
from pathlib import Path
from unittest.mock import Mock

from launch import LaunchContext
from launch.actions import OpaqueFunction
import pytest
import yaml


def load_launch():
    path = Path(__file__).parents[1] / 'launch' / 'tht5.launch.py'
    spec = importlib.util.spec_from_file_location('tht5_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def scene_context(tmp_path, model_exists=True, x=-3.1):
    model = tmp_path / 'model.sdf'
    if model_exists:
        model.write_text('<sdf version="1.6"/>')
    config = tmp_path / 'scene.yaml'
    config.write_text(yaml.safe_dump({
        'world': 'maze', 'ign_ip': '127.0.0.1',
        'chair': {'name': 'yolo_chair', 'model_file': str(model),
                  'x': x, 'y': 3.15, 'z': 0.02, 'yaw': 0.0},
    }))
    context = LaunchContext()
    context.launch_configurations['scene_file'] = str(config)
    return context


def test_spawn_uses_named_chair_and_prevents_duplicate_renaming(tmp_path, monkeypatch):
    module = load_launch()
    create_node = Mock()
    monkeypatch.setattr(module, 'Node', create_node)
    module.spawn_scene_chair(scene_context(tmp_path))
    kwargs = create_node.call_args.kwargs
    assert kwargs['package'] == 'ros_gz_sim'
    args = kwargs['arguments']
    assert args[args.index('-name') + 1] == 'yolo_chair'
    assert args[args.index('-allow_renaming') + 1] == 'false'
    assert args[args.index('-x') + 1] == '-3.1'
    assert kwargs['additional_env']['IGN_IP'] == '127.0.0.1'


def test_missing_model_gives_actionable_error(tmp_path):
    with pytest.raises(RuntimeError, match='spawn_chair:=false'):
        load_launch().spawn_scene_chair(scene_context(tmp_path, model_exists=False))


def test_invalid_pose_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='finite'):
        load_launch().spawn_scene_chair(scene_context(tmp_path, x=float('nan')))


def test_spawn_can_be_disabled_without_loading_model():
    context = LaunchContext()
    context.launch_configurations['spawn_chair'] = 'false'
    actions = load_launch().generate_launch_description().entities
    spawn = next(action for action in actions if isinstance(action, OpaqueFunction))
    assert not spawn.condition.evaluate(context)
