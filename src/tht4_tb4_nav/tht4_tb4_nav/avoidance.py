"""Reactive wall avoidance using LaserScan angles in the robot frame."""

import math


class WallAvoidance:
    """Keep a turn direction until the forward corridor is clear."""

    def __init__(self, scan_yaw=0.0):
        self.scan_yaw = scan_yaw
        self.turn_direction = 0
        self.clear_scans = 0
        self.recovering = False

    def sectors(self, scan):
        """Return conservative distances; missing sectors mean unknown space."""
        values = {'front': [], 'left': [], 'right': [], 'rear': [], 'near': []}
        if not scan.ranges or scan.angle_increment == 0:
            return None
        for index, distance in enumerate(scan.ranges):
            angle = math.atan2(
                math.sin(scan.angle_min + index * scan.angle_increment + self.scan_yaw),
                math.cos(scan.angle_min + index * scan.angle_increment + self.scan_yaw))
            if math.isinf(distance) and distance > 0:
                # Positive infinity is a valid no-return (open space).
                distance = scan.range_max
            elif not math.isfinite(distance) or not (
                    scan.range_min <= distance <= scan.range_max):
                continue
            if abs(angle) <= math.radians(40):
                values['front'].append(distance)
            if math.radians(40) < angle <= math.radians(110):
                values['left'].append(distance)
            if -math.radians(110) <= angle < -math.radians(40):
                values['right'].append(distance)
            if abs(angle) >= math.radians(140):
                values['rear'].append(distance)
            values['near'].append(distance)
        if any(not samples for samples in values.values()):
            return None
        return {name: min(samples) for name, samples in values.items()}

    def command(self, scan):
        """Return (linear m/s, angular rad/s); never drive into unknown space."""
        distances = self.sectors(scan)
        if distances is None:
            self.clear_scans = 0
            return 0.0, 0.0
        front = distances['front']
        left, right = distances['left'], distances['right']

        # Back out of a frontal contact only if the rear corridor is observed clear.
        if front < 0.3:
            self.recovering = True
        if self.recovering:
            self.clear_scans = 0
            if front < 0.45:
                if distances['rear'] > 0.6 and min(left, right) > 0.23:
                    return -0.06, 0.0
                return 0.0, 0.0
            self.recovering = False
            self.turn_direction = 1 if left >= right else -1

        # Distances are measured from the lidar, not the robot's bumper.
        # Leave room for the circular base to rotate without hitting a wall.
        if distances['near'] < 0.23:
            self.clear_scans = 0
            return 0.0, 0.0

        if self.turn_direction:
            if front > 0.85 and min(left, right) > 0.32:
                self.clear_scans += 1
            else:
                self.clear_scans = 0
            if self.clear_scans < 3:
                return 0.0, 0.45 * self.turn_direction
            self.turn_direction = 0
            self.clear_scans = 0

        if front < 0.65 or min(left, right) < 0.32:
            self.turn_direction = 1 if left >= right else -1
            return 0.0, 0.45 * self.turn_direction

        # Slow near obstacles and gently steer away from the nearer side wall.
        speed = min(0.16, max(0.04, (front - 0.65) * 0.25))
        steering = max(-0.25, min(0.25, 0.6 * (min(left, 1.0) - min(right, 1.0))))
        return speed, steering
