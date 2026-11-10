"""Detect unusable lidar data without hiding near obstacles from Nav2."""

import math


def scan_problem(scan):
    """Return an explanation for empty/invalid scans or range-min saturation."""
    if not scan.ranges:
        return 'LaserScan is empty'
    if not math.isfinite(scan.range_min) or not math.isfinite(scan.range_max):
        return 'LaserScan has invalid range limits'
    if not 0 <= scan.range_min < scan.range_max:
        return 'LaserScan has invalid range limits'
    usable = [distance for distance in scan.ranges
              if (math.isfinite(distance) and scan.range_min <= distance <= scan.range_max)
              or distance == math.inf]
    if not usable:
        return 'LaserScan has no valid ranges'
    if len(usable) == len(scan.ranges) and all(
            distance <= scan.range_min + 0.0001 for distance in usable):
        return (f'All {len(usable)} lidar rays are at range_min={scan.range_min:.3f} m. '
                'Check Gazebo lidar/rendering before navigating')
    return ''
