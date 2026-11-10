"""Nonblocking voice command → sequential Nav2 NavigateToPose actions."""

from collections import deque
import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from tht7_tb4.commands import CommandError, parse_command
from tht7_tb4.regions import load_regions
from tht7_tb4.segmentation import default_paths, MapImage


class VoiceNavigation(Node):
    def __init__(self):
        super().__init__('voice_navigation')
        map_path, regions_path = default_paths()
        defaults = {
            'map_file': map_path, 'regions_file': regions_path,
            'goal_timeout': 180.0, 'scan_timeout': 3.0,
            'response_timeout': 10.0, 'base_frame': 'base_link',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        for name in ('goal_timeout', 'scan_timeout', 'response_timeout'):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.regions, clearance = load_regions(self.get_parameter('regions_file').value)
        self.by_id = {region.id: region for region in self.regions}
        map_image = MapImage(self.get_parameter('map_file').value)
        for region in self.regions:
            map_image.validate_goal(region, clearance)
        self.route = deque()
        self.state = 'idle'
        self.goal_handle = None
        self.send_future = None
        self.cancel_requested = False
        self.started = 0.0
        self.last_scan = None
        self.scan_error = 'Chưa có /scan.'
        self.localized = False
        self.last_feedback = 0.0
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/voice_navigation/status', latched)
        self.command_sub = self.create_subscription(
            String, '/voice_command', self.on_command, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        # A wall timer still cancels navigation when the simulator clock stops.
        self.timer = self.create_timer(
            0.2, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.status('READY', 'Đang chờ câu lệnh. Đặt 2D Pose Estimate và undock trước khi đi.')

    def status(self, code, message):
        self.status_pub.publish(String(data=f'{code}: {message}'))
        self.get_logger().info(f'{code}: {message}')

    def on_amcl(self, message):
        pose = message.pose.pose
        values = (pose.position.x, pose.position.y, pose.orientation.z, pose.orientation.w)
        self.localized = all(math.isfinite(value) for value in values)

    def on_scan(self, message):
        self.last_scan = time.monotonic()
        valid = (math.isfinite(message.range_min) and math.isfinite(message.range_max)
                 and 0 <= message.range_min < message.range_max and bool(message.ranges))
        if valid:
            valid = any((math.isfinite(value)
                         and message.range_min <= value <= message.range_max)
                        or value == math.inf for value in message.ranges)
        self.scan_error = '' if valid else '/scan không có phép đo hợp lệ.'

    def navigation_problem(self):
        if not self.localized:
            return 'Chưa nhận /amcl_pose. Đặt 2D Pose Estimate trong RViz.'
        if self.get_clock().now().nanoseconds == 0:
            return 'Chưa có đồng hồ ROS. Kiểm tra Gazebo đang Play.'
        if self.last_scan is None:
            return 'Chưa nhận dữ liệu LiDAR /scan.'
        if time.monotonic() - self.last_scan > self.get_parameter('scan_timeout').value:
            return 'Dữ liệu LiDAR đã cũ.'
        if self.scan_error:
            return self.scan_error
        base = self.get_parameter('base_frame').value
        if not self.tf_buffer.can_transform('map', base, Time()):
            return f'Chưa có TF map → {base}.'
        if not self.client.server_is_ready():
            return 'Nav2 NavigateToPose chưa sẵn sàng.'
        return ''

    def on_command(self, message):
        try:
            command = parse_command(message.data, self.regions)
        except CommandError as error:
            self.status('INVALID', str(error))
            return
        if command.stop:
            if self.state == 'idle':
                self.status('IDLE', 'Robot không có tuyến điều hướng của bài 7.')
            else:
                self.cancel('Người dùng yêu cầu dừng.')
            return
        if self.state != 'idle':
            self.status('BUSY', 'Đang chạy hoặc chờ hủy tuyến cũ. Hãy đợi hoặc nói Dừng lại.')
            return
        problem = self.navigation_problem()
        if problem:
            self.status('NOT_READY', problem)
            return
        self.route = deque(command.route)
        self.cancel_requested = False
        self.status('ROUTE', ' → '.join(self.route))
        self.send_next()

    def send_next(self):
        if not self.route:
            self.state = 'idle'
            self.status('SUCCEEDED', 'Đã đến tất cả khu vực được yêu cầu.')
            return
        problem = self.navigation_problem()
        if problem:
            self.route.clear()
            self.state = 'idle'
            self.status('FAILED', problem)
            return
        region = self.by_id[self.route[0]]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = region.x
        goal.pose.pose.position.y = region.y
        goal.pose.pose.orientation.z = math.sin(region.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(region.yaw / 2)
        self.state, self.started = 'sending', time.monotonic()
        self.status('NAVIGATING', f'{region.id}: {region.name}')
        try:
            self.send_future = self.client.send_goal_async(
                goal, feedback_callback=self.on_feedback)
            self.send_future.add_done_callback(self.on_goal_response)
        except Exception as error:
            self.route.clear()
            self.state = 'fault'
            self.status('ERROR', f'Lỗi gửi goal; kiểm tra Nav2 trước khi khởi động lại: {error}')

    def on_goal_response(self, future):
        self.send_future = None
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.goal_handle = None
                self.route.clear()
                self.state = 'idle'
                self.status('REJECTED', 'Nav2 từ chối đích; đã dừng toàn bộ tuyến.')
                return
            self.goal_handle.get_result_async().add_done_callback(self.on_result)
            if self.cancel_requested:
                self.cancel_goal()
            else:
                self.state = 'active'
        except Exception as error:
            self.route.clear()
            self.state = 'fault'
            self.status('ERROR', f'Không xác nhận được trạng thái goal: {error}')

    def on_feedback(self, message):
        if self.state == 'active' and time.monotonic() - self.last_feedback >= 5:
            self.last_feedback = time.monotonic()
            self.status('FEEDBACK', f'Còn {message.feedback.distance_remaining:.2f} m.')

    def on_result(self, future):
        self.goal_handle = None
        try:
            status = future.result().status
        except Exception as error:
            self.route.clear()
            self.state = 'fault'
            self.status('ERROR', f'Mất kết quả Nav2; cần kiểm tra robot: {error}')
            return
        if self.cancel_requested:
            self.route.clear()
            self.state = 'idle'
            self.cancel_requested = False
            self.status('STOPPED', 'Goal đã kết thúc; tuyến cũ đã được xóa.')
        elif status == GoalStatus.STATUS_SUCCEEDED:
            identifier = self.route.popleft()
            self.status('ARRIVED', f'Đã đến khu vực {identifier}.')
            self.send_next()
        else:
            self.route.clear()
            self.state = 'idle'
            self.status('FAILED', f'Nav2 trả trạng thái {status}; đã dừng toàn bộ tuyến.')

    def cancel(self, reason):
        if self.cancel_requested:
            return
        self.cancel_requested = True
        self.route.clear()
        self.state = 'canceling'
        self.status('CANCELING', reason)
        if self.goal_handle is not None:
            self.cancel_goal()
        # A pending send must resolve before it can be cancelled. Keep BUSY meanwhile.

    def cancel_goal(self):
        try:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.on_cancel_response)
        except Exception as error:
            self.status('ERROR', f'Không gửi được yêu cầu hủy: {error}. Kiểm tra robot.')

    def on_cancel_response(self, future):
        try:
            if not future.result().goals_canceling and self.state == 'canceling':
                self.status('ERROR', 'Nav2 chưa chấp nhận hủy; đang chờ kết quả cuối.')
        except Exception as error:
            self.status('ERROR', f'Lỗi xác nhận hủy: {error}. Đang chờ kết quả cuối.')

    def watchdog(self):
        if self.state not in ('sending', 'active'):
            return
        problem = self.navigation_problem()
        if problem:
            self.cancel(problem)
        elif time.monotonic() - self.started > self.get_parameter('goal_timeout').value:
            self.cancel('Quá thời gian đến đích.')
        elif (self.state == 'sending' and time.monotonic() - self.started
              > self.get_parameter('response_timeout').value):
            self.cancel('Nav2 chưa phản hồi yêu cầu; sẽ hủy khi nhận được goal handle.')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = VoiceNavigation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None and node.state != 'idle':
            node.cancel('Đóng chương trình.')
            deadline = time.monotonic() + 3.0
            while node.state != 'idle' and time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
            if node.state != 'idle':
                node.get_logger().error(
                    'Chưa xác nhận robot dừng. Kiểm tra Nav2 trước khi chạy lại.')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
