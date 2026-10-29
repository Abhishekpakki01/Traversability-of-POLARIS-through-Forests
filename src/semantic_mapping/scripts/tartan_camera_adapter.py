#!/usr/bin/env python3
"""Publish documented virtual pinhole calibration for Tartan rectified images."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
import yaml

from semantic_mapping.tartan_camera import rectified_geometry, quaternion_from_matrix


class TartanCameraAdapter(Node):
    def __init__(self):
        super().__init__('tartan_camera_adapter')
        default = Path(get_package_share_directory('semantic_mapping'))/'config/tartan_meadows_camera.yaml'
        path = self.declare_parameter('calibration_file', str(default)).value
        self.config = yaml.safe_load(Path(path).read_text())
        self.intrinsic, camera, lidar = rectified_geometry(self.config)
        self.rectified_frame = 'tartan_left_rectified_optical'
        self.transforms = [self.make_transform('vehicle', self.rectified_frame, camera),
                           self.make_transform('vehicle', 'velodyne_1', lidar)]
        self.broadcaster = StaticTransformBroadcaster(self)
        self.image_pub = self.create_publisher(Image, '/tartan/camera/image', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/tartan/camera/camera_info', 1)
        self.sub = self.create_subscription(Image, '/multisense/left/image_rect_color',
                                             self.on_image, qos_profile_sensor_data)
        self.get_logger().info('Using publisher S21 calibration and a rectified optical frame; '
                               'camera timestamps/pixels preserved. Recorded default camera URDF is not used.')

    @staticmethod
    def make_transform(parent, child, matrix):
        message = TransformStamped()
        message.header.frame_id, message.child_frame_id = parent, child
        t, q = matrix[:3, 3], quaternion_from_matrix(matrix[:3, :3])
        message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = map(float, t)
        (message.transform.rotation.x, message.transform.rotation.y,
         message.transform.rotation.z, message.transform.rotation.w) = map(float, q)
        return message

    def on_image(self, image):
        if (image.width != self.config['width'] or image.height != self.config['height']
                or image.header.frame_id != 'multisense/left_camera_optical_frame'):
            self.get_logger().error('Image dimensions/frame do not match the selected dataset calibration',
                                    throttle_duration_sec=2.)
            return
        # Reannounce static transforms after the initial bag clock transition.
        self.broadcaster.sendTransform(self.transforms)
        header = Header(stamp=image.header.stamp, frame_id=self.rectified_frame)
        output = Image(header=header, height=image.height, width=image.width,
                       encoding=image.encoding, is_bigendian=image.is_bigendian,
                       step=image.step, data=image.data)
        info = CameraInfo(header=header, height=image.height, width=image.width,
            distortion_model='plumb_bob', d=[0.]*5, k=self.intrinsic.reshape(-1).tolist(),
            r=np.eye(3).reshape(-1).tolist(),
            p=np.column_stack((self.intrinsic, np.zeros(3))).reshape(-1).tolist())
        self.info_pub.publish(info)
        self.image_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = TartanCameraAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
