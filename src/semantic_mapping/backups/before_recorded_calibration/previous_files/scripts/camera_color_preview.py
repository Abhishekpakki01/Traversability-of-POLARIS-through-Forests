#!/usr/bin/env python3
"""Publish a color camera reference for RViz, preserving acquisition headers."""

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraColorPreview(Node):
    def __init__(self):
        super().__init__('camera_color_preview')
        input_topic = self.declare_parameter(
            'input_topic', '/pylon_camera_node/image_raw').value
        output_topic = self.declare_parameter(
            'output_topic', '/pylon_camera_node/image_color').value
        self.bridge = CvBridge()
        # Reliable output works with RViz's default Image subscription.
        self.publisher = self.create_publisher(Image, output_topic, 1)
        self.subscription = self.create_subscription(
            Image, input_topic, self.on_image, qos_profile_sensor_data)
        self.reported_first_frame = False
        self.get_logger().info(f'Camera reference: {input_topic} -> {output_topic}')

    def on_image(self, message):
        try:
            color = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            output = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
        except (CvBridgeError, ValueError, TypeError) as error:
            self.get_logger().error(
                f'Cannot convert camera encoding {message.encoding}: {error}',
                throttle_duration_sec=2.0)
            return
        output.header = message.header
        self.publisher.publish(output)
        if not self.reported_first_frame:
            self.get_logger().info(
                f'Publishing {output.width}x{output.height} bgr8 images; '
                'original timestamps and camera frame are preserved')
            self.reported_first_frame = True


def main(args=None):
    rclpy.init(args=args)
    node = CameraColorPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
