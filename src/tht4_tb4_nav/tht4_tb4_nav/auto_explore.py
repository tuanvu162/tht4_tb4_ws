import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from sensor_msgs.msg import LaserScan
from tht4_tb4_nav.avoidance import WallAvoidance

class AutoExplore(Node):
    def __init__(self):
        super().__init__('auto_explore')
        # Tao publisher cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Tao subscriber scan
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.ready = False
        self.is_docked = None
        self.undock_started = False
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.dock_sub = self.create_subscription(
            DockStatus, '/dock_status', self.dock_cb, qos_profile_sensor_data)
        self.undock_timer = self.create_timer(1.0, self.check_undock)
        self.get_logger().info('Waiting for dock status before moving.')
        # TurtleBot4's rplidar_link is rotated +90 degrees from the base.
        self.declare_parameter('scan_yaw', math.pi / 2)
        self.avoidance = WallAvoidance(self.get_parameter('scan_yaw').value)
        self.last_scan_time = None
        self.watchdog = self.create_timer(0.1, self.check_scan_timeout)

    def dock_cb(self, msg):
        self.is_docked = msg.is_docked
        if msg.is_docked:
            self.ready = False
        elif not self.undock_started:
            self.ready = True

    def check_undock(self):
        if self.is_docked is not True or self.undock_started:
            return
        if not self.undock_client.server_is_ready():
            return
        self.undock_started = True
        self.get_logger().info('Undocking; exploration waits for completion.')
        future = self.undock_client.send_goal_async(Undock.Goal())
        future.add_done_callback(self.undock_goal_cb)

    def undock_goal_cb(self, future):
        goal = future.result()
        if not goal.accepted:
            self.get_logger().error('Undock rejected; exploration remains stopped.')
            return
        goal.get_result_async().add_done_callback(self.undock_result_cb)

    def undock_result_cb(self, future):
        result = future.result()
        self.ready = (
            result.status == GoalStatus.STATUS_SUCCEEDED
            and not result.result.is_docked)
        if self.ready:
            self.get_logger().info('Undock succeeded; starting exploration.')
        else:
            self.get_logger().error(
                f'Undock failed (status={result.status}); exploration remains stopped.')

    def scan_cb(self, msg):
        # Do not compete with the docking controller, even with zero velocity.
        if not self.ready:
            return
        self.last_scan_time = time.monotonic()
        linear, angular = self.avoidance.command(msg)
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def check_scan_timeout(self):
        if self.ready and (self.last_scan_time is None or
                           time.monotonic() - self.last_scan_time > 0.5):
            self.cmd_pub.publish(Twist())

def main(args=None):
    # Khoi tao rclpy
    rclpy.init(args=args)
    node = AutoExplore()
    # Cho node chay
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            if node.ready:
                node.cmd_pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
