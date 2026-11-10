"""Verify occupancy, coordinates, physical barriers and supplied map seeds."""

from dataclasses import replace
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from tht7_tb4.regions import load_regions
from tht7_tb4.segmentation import MapImage, save_results, segment


PACKAGE = Path(__file__).resolve().parents[1]
MAP = PACKAGE / 'config/map.yaml'


def synthetic_map(tmp_path, image, yaw=0.0, negate=0):
    cv2.imwrite(str(tmp_path / 'map.pgm'), image)
    path = tmp_path / 'map.yaml'
    path.write_text(yaml.safe_dump({
        'image': 'map.pgm', 'mode': 'trinary', 'resolution': 0.1,
        'origin': [1.2, -2.3, yaw], 'free_thresh': 0.25,
        'occupied_thresh': 0.65, 'negate': negate,
    }))
    return MapImage(path)


def test_real_map_segmentation_and_goal_clearance(tmp_path):
    regions, clearance = load_regions(PACKAGE / 'config/regions.yaml')
    map_image = MapImage(MAP)
    labels = segment(map_image, regions, clearance)
    assert set(np.unique(labels)) == {0, 1, 2, 3}
    assert np.all(labels[~map_image.free] == 0)
    assert not np.any(labels[map_image.image == 205])
    for label, region in enumerate(regions, 1):
        u, v = map_image.world_to_pixel(region.x, region.y)
        assert labels[v, u] == label
        assert map_image.clearance[v, u] >= clearance
    metadata = save_results(map_image, regions, clearance, tmp_path)
    saved = cv2.imread(str(tmp_path / 'region_labels.png'), cv2.IMREAD_UNCHANGED)
    np.testing.assert_array_equal(saved, labels)
    assert len(metadata['regions']) == 3


@pytest.mark.parametrize('yaw', [0, math.pi / 2, -0.7])
def test_pixel_centres_round_trip_with_rotated_origin(tmp_path, yaw):
    image = synthetic_map(tmp_path, np.full((20, 30), 254, np.uint8), yaw)
    for u, v in [(0, 0), (29, 19), (5, 8)]:
        assert image.world_to_pixel(*image.pixel_to_world(u, v)) == (u, v)
    with pytest.raises(ValueError, match='outside'):
        image.world_to_pixel(*image.pixel_to_world(-1, 0))


def test_walls_unknown_cells_and_disconnected_seeds(tmp_path):
    pixels = np.full((20, 30), 254, np.uint8)
    pixels[:, 14] = 0
    pixels[2, 2] = 150  # Occupancy between the configured free/occupied thresholds.
    image = synthetic_map(tmp_path, pixels)
    original = load_regions(PACKAGE / 'config/regions.yaml')[0]
    regions = [replace(region, x=image.pixel_to_world(u, 10)[0],
                       y=image.pixel_to_world(u, 10)[1])
               for region, u in zip(original, [5, 10, 20])]
    assert not image.free[2, 2] and not image.free[10, 14]
    with pytest.raises(ValueError, match='separated'):
        segment(image, regions, 0.1)
    x, y = image.pixel_to_world(14, 10)
    with pytest.raises(ValueError, match='occupied'):
        image.validate_goal(replace(regions[0], x=x, y=y), 0.1)


def test_negated_map(tmp_path):
    image = synthetic_map(tmp_path, np.array([[0, 205, 254]], np.uint8), negate=1)
    assert image.free.tolist() == [[True, False, False]]
