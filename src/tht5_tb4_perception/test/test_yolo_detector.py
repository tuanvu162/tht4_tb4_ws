"""Check detection output and image metadata without loading YOLO weights."""

from types import SimpleNamespace
from unittest.mock import Mock

from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Header

from tht5_tb4_perception.yolo_detector import (
    YoloDetector, annotated_message, detection_records,
)


def test_class_confidence_and_boxes_are_exported():
    box = SimpleNamespace(cls=Mock(), conf=Mock(), xyxy=[Mock()])
    box.cls.item.return_value = 0
    box.conf.item.return_value = 0.91
    box.xyxy[0].tolist.return_value = [10, 20, 30, 40]
    result = SimpleNamespace(boxes=[box], names={0: 'person'})
    assert detection_records(result) == [
        {'class': 'person', 'confidence': 0.91, 'bbox_xyxy': [10, 20, 30, 40]}]


def test_empty_result_has_no_invented_detections():
    assert detection_records(SimpleNamespace(boxes=[], names={})) == []
    assert detection_records(SimpleNamespace(boxes=None)) == []


def test_annotated_image_preserves_camera_header_and_bgr():
    bridge = CvBridge()
    header = Header(frame_id='camera_frame')
    header.stamp.sec = 42
    result = Mock()
    result.plot.return_value = np.zeros((16, 24, 3), dtype=np.uint8)
    msg = annotated_message(bridge, result, header)
    assert msg.header == header
    assert msg.encoding == 'bgr8'
    assert bridge.imgmsg_to_cv2(msg, 'bgr8').shape == (16, 24, 3)


def test_new_camera_frame_replaces_old_frame():
    state = SimpleNamespace(latest_image=None, last_received=None)
    first, latest = object(), object()
    YoloDetector.receive_image(state, first)
    YoloDetector.receive_image(state, latest)
    assert state.latest_image is latest
    assert state.last_received is not None
