"""Navigate through configured map goals after manual localization/undocking."""

import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from tht5_tb4_perception.scan_health import scan_problem


def validate_goals(xs, ys, yaws):
    """Require at least three complete, finite waypoints."""
    if len(xs) < 3 or len(xs) != len(ys) or len(xs) != len(yaws):
        raise ValueError('goal_x, goal_y and goal_yaw must have equal lengths >= 3')
    goals = list(zip(xs, ys, yaws))
    if not all(math.isfinite(value) for goal in goals for value in goal):
        raise ValueError('All goal coordinates and yaw values must be finite')
    return goals


def goal_pose(x, y, yaw, stamp, frame_id='map'):
    """Convert a planar waypoint to a stamped Nav2 pose."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def run_route(navigator, goals, timeout, now=time.monotonic, health_check=None):
    """Stop the route on rejection, failure, cancellation or wall-time timeout."""
    logger = navigator.get_logger()
    for index, (x, y, yaw) in enumerate(goals, start=1):
        problem = health_check() if health_check is not None else ''
        if problem:
            logger.error(f'{problem}. Route not started.')
            return False
        logger.info(f'Goal {index}/{len(goals)}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')
        pose = goal_pose(x, y, yaw, navigator.get_clock().now().to_msg())
        if not navigator.goToPose(pose):
            logger.error(f'Goal {index} rejected. Route stopped.')
            return False
        started = now()
        last_feedback = started
        while not navigator.isTaskComplete():
            problem = health_check() if health_check is not None else ''
            if problem:
                navigator.cancelTask()
                logger.error(f'{problem}. Cancel requested; route stopped.')
                return False
            if now() - started >= timeout:
                navigator.cancelTask()
                logger.error(f'Goal {index} timed out. Cancel requested; route stopped.')
                return False
            if now() - last_feedback >= 5.0:
                feedback = navigator.getFeedback()
                if feedback is not None:
                    logger.info(f'Goal {index}: {feedback.distance_remaining:.2f} m remaining')
                last_feedback = now()
        result = navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            logger.error(f'Goal {index}: {result.name}. Route stopped.')
            return False
        logger.info(f'Goal {index}: SUCCEEDED')
    logger.info(f'Completed all {len(goals)} goals successfully.')
    return True


class WaypointNavigation(BasicNavigator):
    """Use the existing AMCL initial pose without publishing a replacement."""

    def __init__(self):
        super().__init__(node_name='waypoint_navigation')
        self.declare_parameter('frame_id', 'map')
        for name in ('goal_x', 'goal_y', 'goal_yaw'):
            self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('goal_timeout', 180.0)
        self.declare_parameter('localization_timeout', 120.0)
        self.declare_parameter('scan_timeout', 3.0)
        self.last_scan_received = None
        self.scan_error = ''
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

    def scan_callback(self, scan):
        self.last_scan_received = time.monotonic()
        self.scan_error = scan_problem(scan)

    def navigation_problem(self):
        if self.last_scan_received is None:
            return 'No /scan data received'
        if time.monotonic() - self.last_scan_received > self.get_parameter('scan_timeout').value:
            return 'Lidar data is stale'
        return self.scan_error

    def run(self):
        goals = validate_goals(
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            self.get_parameter('goal_yaw').value)
        if self.get_parameter('frame_id').value != 'map':
            raise ValueError('These waypoints must use frame_id: map')
        timeout = self.get_parameter('goal_timeout').value
        startup_timeout = self.get_parameter('localization_timeout').value
        scan_timeout = self.get_parameter('scan_timeout').value
        if not all(math.isfinite(v) and v > 0 for v in (
                timeout, startup_timeout, scan_timeout)):
            raise ValueError('Timeouts must be finite positive numbers')

        self.get_logger().info(
            'Waiting for /amcl_pose, valid /scan and simulation clock. '
            'Set 2D Pose Estimate in RViz. '
            'Robot must already be undocked; stop teleop before running this node.')
        deadline = time.monotonic() + startup_timeout
        # Humble BasicNavigator otherwise publishes its default initial pose while waiting.
        # Receive the existing AMCL pose first, preserving the user's localization.
        while (not self.initial_pose_received or self.get_clock().now().nanoseconds == 0
               or self.navigation_problem()):
            if self.last_scan_received is not None and self.scan_error:
                self.get_logger().error(self.scan_error + '. Route not started.')
                return False
            if time.monotonic() >= deadline:
                self.get_logger().error('Missing AMCL pose, clock or valid scan. Route not started.')
                return False
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info('AMCL pose received. Waiting for active AMCL and Nav2.')
        self.waitUntilNav2Active()
        return run_route(self, goals, timeout, health_check=self.navigation_problem)


def main(args=None):
    # Keep ROS alive on Ctrl+C long enough to request cancellation of our action.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    navigator = None
    succeeded = False
    try:
        navigator = WaypointNavigation()
        succeeded = navigator.run()
    except KeyboardInterrupt:
        if navigator is not None:
            navigator.get_logger().warning('Interrupted; canceling current navigation task.')
            navigator.cancelTask()
    except Exception as error:
        if navigator is not None:
            navigator.get_logger().error(f'{error}. Check --params-file and Nav2 logs.')
            navigator.cancelTask()
        else:
            print(f'Cannot initialize waypoint_navigation: {error}')
    finally:
        if navigator is not None:
            navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if succeeded else 1


if __name__ == '__main__':
    raise SystemExit(main())
