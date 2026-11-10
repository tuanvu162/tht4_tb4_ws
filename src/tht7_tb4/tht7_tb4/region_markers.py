"""Publish the actual segmented free cells and semantic goals in RViz."""

import math
import unicodedata

from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from tht7_tb4.regions import load_regions
from tht7_tb4.segmentation import default_paths, MapImage, segment


class RegionMarkers(Node):
    def __init__(self):
        super().__init__('region_markers')
        map_path, regions_path = default_paths()
        self.declare_parameter('map_file', map_path)
        self.declare_parameter('regions_file', regions_path)
        regions, clearance = load_regions(self.get_parameter('regions_file').value)
        map_image = MapImage(self.get_parameter('map_file').value)
        labels = segment(map_image, regions, clearance)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, '/semantic_regions', qos)
        self.markers = MarkerArray()
        for index, region in enumerate(regions, 1):
            cells = self.marker(index, Marker.CUBE_LIST, 'cells', region.color, 0.28)
            cells.scale.x = cells.scale.y = map_image.resolution
            cells.scale.z = 0.015
            # Full-resolution cells preserve narrow walls and exact segmentation boundaries.
            vs, us = np.where(labels == index)
            for v, u in zip(vs, us):
                x, y = map_image.pixel_to_world(int(u), int(v))
                cells.points.append(Point(x=x, y=y, z=0.025))
            text = self.marker(index, Marker.TEXT_VIEW_FACING, 'labels', region.color)
            text.pose.position = Point(x=region.x, y=region.y + 0.5, z=0.65)
            text.scale.z = 0.26
            # RViz's marker font does not reliably render Vietnamese glyphs.
            # Keep Vietnamese semantic names/voice aliases, transliterate only the display.
            display_name = unicodedata.normalize(
                'NFKD', region.name.replace('đ', 'd').replace('Đ', 'D')
            ).encode('ascii', 'ignore').decode('ascii')
            text.text = f'{region.id}\n{display_name}'
            goal = self.marker(index, Marker.ARROW, 'goals', region.color)
            goal.pose.position = Point(x=region.x, y=region.y, z=0.08)
            goal.pose.orientation.z = math.sin(region.yaw / 2)
            goal.pose.orientation.w = math.cos(region.yaw / 2)
            goal.scale.x, goal.scale.y, goal.scale.z = 0.5, 0.12, 0.12
            self.markers.markers.extend([cells, text, goal])
        self.publisher.publish(self.markers)
        self.get_logger().info('Published 3 semantic zones on /semantic_regions (MarkerArray).')

    @staticmethod
    def marker(identifier, kind, namespace, color, alpha=1.0):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns, marker.id, marker.type = namespace, identifier, kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = alpha
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RegionMarkers()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
