#!/usr/bin/env python3
"""Separate adapter using recorded mounting transforms and stated intrinsics."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
import yaml

from semantic_mapping.recorded_calibration import recorded_geometry
from semantic_mapping.tartan_camera import quaternion_from_matrix


class RecordedCameraAdapter(Node):
    def __init__(self):
        super().__init__('tartan_recorded_camera_adapter')
        default = Path(get_package_share_directory('semantic_mapping'))/'config/tartan_recorded_calibration.yaml'
        path = self.declare_parameter('calibration_file', str(default)).value
        self.config = yaml.safe_load(Path(path).read_text())
        self.intrinsic, camera, lidar = recorded_geometry(self.config)
        self.frame = 'tartan_recorded_rectified_optical'
        self.transforms = [self.make_transform(self.frame, camera),
                           self.make_transform('velodyne_1', lidar)]
        self.broadcaster = StaticTransformBroadcaster(self)
        self.image_pub = self.create_publisher(Image, '/tartan_recorded/camera/image', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/tartan_recorded/camera/camera_info', 1)
        self.sub = self.create_subscription(Image, self.config['image_topic'], self.on_image,
                                             qos_profile_sensor_data)
        self.get_logger().info('Camera mounting transforms match the original recording; '
                               'image pixels and timestamps are preserved.')
        if not self.config['intrinsics_verified']:
            self.get_logger().warning('Camera intrinsics/rectification remain provisional: '
                                      + self.config['intrinsics_status'])

    @staticmethod
    def make_transform(child, matrix):
        message = TransformStamped()
        message.header.frame_id, message.child_frame_id = 'vehicle', child
        t, q = matrix[:3, 3], quaternion_from_matrix(matrix[:3, :3])
        (message.transform.translation.x, message.transform.translation.y,
         message.transform.translation.z) = map(float, t)
        (message.transform.rotation.x, message.transform.rotation.y,
         message.transform.rotation.z, message.transform.rotation.w) = map(float, q)
        return message

    def on_image(self, image):
        if (image.width != self.config['width'] or image.height != self.config['height']
                or image.header.frame_id != self.config['image_frame']):
            self.get_logger().error('Image does not match this recording calibration',
                                    throttle_duration_sec=2.)
            return
        self.broadcaster.sendTransform(self.transforms)
        header = Header(stamp=image.header.stamp, frame_id=self.frame)
        info = CameraInfo(header=header, height=image.height, width=image.width,
            distortion_model='plumb_bob', d=[0.]*5, k=self.intrinsic.reshape(-1).tolist(),
            r=np.eye(3).reshape(-1).tolist(),
            p=np.column_stack((self.intrinsic, np.zeros(3))).reshape(-1).tolist())
        output = Image(header=header, height=image.height, width=image.width,
            encoding=image.encoding, is_bigendian=image.is_bigendian,
            step=image.step, data=image.data)
        self.info_pub.publish(info)
        self.image_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = RecordedCameraAdapter()
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

