"""Geodesic seeded segmentation of known free occupancy-grid cells."""

import argparse
from collections import deque
import hashlib
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from tht7_tb4.regions import load_regions


class MapImage:
    """Convert pixel cells using map.yaml, including nonzero origin yaw."""

    def __init__(self, path):
        self.path = Path(path).expanduser().resolve()
        with self.path.open(encoding='utf-8') as stream:
            self.info = yaml.safe_load(stream)
        if self.info.get('mode', 'trinary') != 'trinary':
            raise ValueError('Only trinary maps are supported')
        self.image_path = self.path.parent / self.info['image']
        self.image = cv2.imread(str(self.image_path), cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise ValueError(f'Cannot read map image: {self.image_path}')
        self.height, self.width = self.image.shape
        self.resolution = float(self.info['resolution'])
        self.origin = tuple(float(value) for value in self.info['origin'])
        if (not math.isfinite(self.resolution) or self.resolution <= 0
                or len(self.origin) != 3 or not all(map(math.isfinite, self.origin))):
            raise ValueError('Invalid map resolution or origin')
        free_threshold = float(self.info['free_thresh'])
        occupied_threshold = float(self.info['occupied_thresh'])
        if not 0 < free_threshold < occupied_threshold < 1:
            raise ValueError('Map thresholds must satisfy 0 < free < occupied < 1')
        occupancy = self.image.astype(np.float64) / 255.0
        if not self.info.get('negate', 0):
            occupancy = 1.0 - occupancy
        self.free = occupancy < free_threshold
        # Padding makes the edge of an image an obstacle for clearance validation.
        padded = np.pad(self.free.astype(np.uint8), 1)
        self.clearance = (cv2.distanceTransform(padded, cv2.DIST_L2, 5)[1:-1, 1:-1]
                          * self.resolution)

    def pixel_to_world(self, u, v):
        """Return the centre of a pixel, not its corner."""
        local_x = (u + 0.5) * self.resolution
        local_y = (self.height - v - 0.5) * self.resolution
        x, y, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return x + c * local_x - s * local_y, y + s * local_x + c * local_y

    def world_to_pixel(self, x, y):
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = x - ox, y - oy
        u = math.floor((c * dx + s * dy) / self.resolution)
        v = self.height - 1 - math.floor((-s * dx + c * dy) / self.resolution)
        if not (0 <= u < self.width and 0 <= v < self.height):
            raise ValueError(f'Goal ({x:.3f}, {y:.3f}) is outside the map')
        return u, v

    def validate_goal(self, region, minimum_clearance):
        u, v = self.world_to_pixel(region.x, region.y)
        if not self.free[v, u] or self.clearance[v, u] < minimum_clearance:
            raise ValueError(
                f'Region {region.id}: goal is occupied, unknown or too close to a wall '
                f'(clearance={self.clearance[v, u]:.2f} m, '
                f'required={minimum_clearance:.2f} m)')
        return u, v


def segment(map_image, regions, minimum_clearance):
    """Multi-source BFS, constrained by walls; 0 denotes unassigned cells.

    Four-connected geodesic distance produces reproducible functional zones.
    This is seeded zoning, not automatic recognition of architectural rooms.
    """
    labels = np.zeros(map_image.free.shape, dtype=np.uint16)
    queue = deque()
    seeds = []
    for label, region in enumerate(regions, 1):
        u, v = map_image.validate_goal(region, minimum_clearance)
        if labels[v, u]:
            raise ValueError('Two region seeds occupy the same map cell')
        labels[v, u] = label
        queue.append((v, u))
        seeds.append((v, u))
    while queue:
        v, u = queue.popleft()
        for nv, nu in ((v - 1, u), (v + 1, u), (v, u - 1), (v, u + 1)):
            if (0 <= nv < map_image.height and 0 <= nu < map_image.width
                    and map_image.free[nv, nu] and labels[nv, nu] == 0):
                labels[nv, nu] = labels[v, u]
                queue.append((nv, nu))
    # All seeds must be in the same robot-accessible component of the static map.
    component = cv2.connectedComponents(map_image.free.astype(np.uint8), connectivity=4)[1]
    if len({int(component[v, u]) for v, u in seeds}) != 1:
        raise ValueError('Region goals are separated by occupied/unknown cells')
    return labels


def save_results(map_image, regions, clearance, output):
    labels = segment(map_image, regions, clearance)
    output = Path(output).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    preview = cv2.cvtColor(map_image.image, cv2.COLOR_GRAY2BGR)
    metadata = {
        'method': 'four-connected multi-source geodesic BFS',
        'frame_id': 'map',
        'map_yaml_sha256': hashlib.sha256(map_image.path.read_bytes()).hexdigest(),
        'map_image_sha256': hashlib.sha256(map_image.image_path.read_bytes()).hexdigest(),
        'resolution': map_image.resolution,
        'origin': list(map_image.origin),
        'label_image': 'region_labels.png',
        'unassigned_free_cells': int(np.count_nonzero(map_image.free & (labels == 0))),
        'regions': [],
    }
    for label, region in enumerate(regions, 1):
        mask = labels == label
        color = tuple(int(255 * channel) for channel in reversed(region.color))
        preview[mask] = (0.4 * preview[mask] + 0.6 * np.array(color)).astype(np.uint8)
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(preview, contours, -1, color, 1)
        u, v = map_image.world_to_pixel(region.x, region.y)
        cv2.circle(preview, (u, v), 4, (0, 0, 0), -1)
        cv2.putText(preview, region.id, (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 0, 0), 2, cv2.LINE_AA)
        metadata['regions'].append({
            'id': region.id, 'name': region.name, 'label_value': label,
            'goal': {'x': region.x, 'y': region.y, 'yaw': region.yaw},
            'goal_clearance_m': float(map_image.clearance[v, u]),
            'area_m2': float(mask.sum() * map_image.resolution ** 2),
        })
    for filename, array in [('region_labels.png', labels), ('segmented_map.png', preview)]:
        if not cv2.imwrite(str(output / filename), array):
            raise OSError(f'Cannot write {output / filename}')
    (output / 'segmentation.yaml').write_text(
        yaml.safe_dump(metadata, allow_unicode=True, sort_keys=False), encoding='utf-8')
    return metadata


def default_paths():
    from ament_index_python.packages import get_package_share_directory
    return (get_package_share_directory('tht7_tb4') + '/config/map.yaml',
            get_package_share_directory('tht7_tb4') + '/config/regions.yaml')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', dest='map_path')
    parser.add_argument('--regions')
    parser.add_argument('--output', default='tht7_results')
    options = parser.parse_args(args)
    if not options.map_path or not options.regions:
        map_path, regions_path = default_paths()
        options.map_path = options.map_path or map_path
        options.regions = options.regions or regions_path
    regions, clearance = load_regions(options.regions)
    metadata = save_results(MapImage(options.map_path), regions, clearance, options.output)
    for item in metadata['regions']:
        print(f"{item['id']}: {item['name']}; area={item['area_m2']:.2f} m²; "
              f"goal clearance={item['goal_clearance_m']:.2f} m")
    print(f'Results: {Path(options.output).resolve()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
