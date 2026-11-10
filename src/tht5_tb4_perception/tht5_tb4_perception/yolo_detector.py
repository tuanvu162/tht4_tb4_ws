"""Run YOLO on the newest RGB frame and publish an annotated ROS image."""

import json
import math
from pathlib import Path
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def detection_records(result):
    """Convert YOLO boxes into JSON-compatible records, including confidence."""
    if result.boxes is None:
        return []
    return [
        {'class': result.names[int(box.cls.item())],
         'confidence': float(box.conf.item()),
         'bbox_xyxy': [float(value) for value in box.xyxy[0].tolist()]}
        for box in result.boxes
    ]


def annotated_message(bridge, result, header):
    """Keep the original camera timestamp/frame on the annotated BGR image."""
    msg = bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
    msg.header = header
    return msg


class YoloDetector(Node):
    """Bound camera buffering to one image and limit CPU inference frequency."""

    def __init__(self):
        super().__init__('yolo_detector')
        defaults = {
            'camera_topic': '/oakd/rgb/preview/image_raw',
            'image_topic': '/yolo/image',
            'detections_topic': '/yolo/detections',
            'model_path': '',
            'confidence': 0.4,
            'image_size': 320,
            'inference_hz': 2.0,
            'cpu_threads': 2,
            'max_detections': 20,
            'log_interval': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        if not 0 < self.settings['confidence'] <= 1:
            raise ValueError('confidence must be in (0, 1]')
        for name in ('inference_hz', 'log_interval'):
            if not math.isfinite(self.settings[name]) or self.settings[name] <= 0:
                raise ValueError(f'{name} must be a finite positive number')
        for name in ('image_size', 'cpu_threads', 'max_detections'):
            if self.settings[name] <= 0:
                raise ValueError(f'{name} must be positive')
        model_path = Path(self.settings['model_path']).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(
                f'Model not found: {model_path}. Set model_path to your local yolov8n.pt')

        import torch
        from ultralytics import YOLO

        torch.set_num_threads(self.settings['cpu_threads'])
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(str(model_path))
        self.bridge = CvBridge()
        self.latest_image = None
        self.last_received = None
        self.last_log = 0.0
        self.last_warning = 0.0
        self.image_pub = self.create_publisher(Image, self.settings['image_topic'], 1)
        self.detections_pub = self.create_publisher(
            String, self.settings['detections_topic'], 1)
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Image, self.settings['camera_topic'], self.receive_image, qos)
        # Wall-time scheduling keeps inference bounded even when RTF fluctuates.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / self.settings['inference_hz'], self.detect, clock=self.steady_clock)
        self.get_logger().info(
            f"RGB: {self.settings['camera_topic']} -> {self.settings['image_topic']}; "
            f"CPU, imgsz={self.settings['image_size']}, "
            f"limit={self.settings['inference_hz']} FPS. Waiting for camera.")

    def receive_image(self, msg):
        self.latest_image = msg
        self.last_received = time.monotonic()

    def detect(self):
        now = time.monotonic()
        if self.latest_image is None:
            if (self.last_received is None or now - self.last_received > 5.0) \
                    and now - self.last_warning > 5.0:
                self.get_logger().warning('No recent RGB image. Check camera topic and Gazebo Play.')
                self.last_warning = now
            return
        msg = self.latest_image
        self.latest_image = None
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            result = self.model.predict(
                source=image, conf=self.settings['confidence'],
                imgsz=self.settings['image_size'], device='cpu',
                max_det=self.settings['max_detections'], verbose=False)[0]
            records = detection_records(result)
            self.image_pub.publish(annotated_message(self.bridge, result, msg.header))
            report = String()
            report.data = json.dumps({
                'frame_id': msg.header.frame_id,
                'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'objects': records,
            }, ensure_ascii=False)
            self.detections_pub.publish(report)
            if now - self.last_log >= self.settings['log_interval']:
                summary = ', '.join(
                    f"{record['class']} {record['confidence']:.0%}" for record in records)
                self.get_logger().info(
                    f"YOLO: {summary or 'no objects detected'}; "
                    f'inference+publish={1000 * (time.monotonic() - now):.0f} ms')
                self.last_log = now
        except Exception as error:
            if now - self.last_warning > 5.0:
                self.get_logger().error(f'Image/inference failed: {error}')
                self.last_warning = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f'Cannot start yolo_detector: {error}')
        code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
