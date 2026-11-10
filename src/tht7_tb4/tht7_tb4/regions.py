"""Load and validate the semantic region catalogue without importing ROS."""

from dataclasses import dataclass
import math
from pathlib import Path
import re
import unicodedata

import yaml


def normalize(text):
    """Accept Vietnamese with or without accents; preserve token boundaries."""
    text = unicodedata.normalize('NFD', text.lower().replace('đ', 'd'))
    text = ''.join(char for char in text if unicodedata.category(char) != 'Mn')
    return re.sub(r'[^a-z0-9]+', ' ', text).strip()


@dataclass(frozen=True)
class Region:
    id: str
    name: str
    aliases: tuple
    x: float
    y: float
    yaw: float
    color: tuple


def load_regions(path):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if config.get('frame_id') != 'map':
        raise ValueError('regions.yaml requires frame_id: map')
    clearance = float(config.get('minimum_clearance', 0.3))
    if not math.isfinite(clearance) or clearance <= 0:
        raise ValueError('minimum_clearance must be finite and positive')
    regions, owners = [], {}
    for item in config['regions']:
        goal = item['goal']
        values = tuple(float(goal[key]) for key in ('x', 'y', 'yaw'))
        color = tuple(float(value) for value in item['color'])
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Region goal must contain finite x, y, yaw')
        if len(color) != 3 or not all(0 <= value <= 1 for value in color):
            raise ValueError('Region color must be RGB in [0, 1]')
        identifier, name = str(item['id']), str(item['name'])
        if any(region.id == identifier for region in regions):
            raise ValueError(f'Duplicate region id: {identifier}')
        raw_aliases = {normalize(alias) for alias in
                       [identifier, name, *item.get('aliases', [])]}
        # Spoken letter variants (bê/bi/xê/...) also work after 'khu vực'.
        aliases = tuple(sorted(raw_aliases | {
            prefix + alias for alias in raw_aliases if ' ' not in alias
            for prefix in ('khu vuc ', 'khu ', 'phong ')}))
        for alias in aliases:
            if not alias or (alias in owners and owners[alias] != identifier):
                raise ValueError(f'Empty or ambiguous alias: {alias}')
            owners[alias] = identifier
        regions.append(Region(identifier, name, aliases, *values, color))
    if len(regions) < 3:
        raise ValueError('At least three semantic regions are required')
    return regions, clearance
