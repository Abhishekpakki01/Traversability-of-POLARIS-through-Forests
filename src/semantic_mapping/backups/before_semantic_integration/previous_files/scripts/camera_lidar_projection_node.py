#!/usr/bin/env python3
"""Timestamp-paired camera/LiDAR projection reference, separate from OHM."""

from collections import deque
from dataclasses import dataclass
import json
import time

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from semantic_mapping.camera_projection import (
    draw_projection, project_visible_points, transform_matrix, transform_points, xyz_from_cloud)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


@dataclass
class CameraFrame:
    image: Image
    info: CameraInfo

    @property
    def header(self):
        return self.image.header


def rgb_cloud(header, frame, points, rgb):
    cloud = PointCloud2()
    cloud.header = header
    cloud.header.frame_id = frame
    cloud.height, cloud.width = 1, len(points)
    cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                    for name, offset in [('x', 0), ('y', 4), ('z', 8), ('rgb', 12)]]
    cloud.point_step, cloud.row_step = 16, 16 * len(points)
    cloud.is_bigendian, cloud.is_dense = False, True
    packed = np.empty(len(points), dtype=[('xyz', '<f4', (3,)), ('rgb', '<u4')])
    packed['xyz'] = points
    rgb = rgb.astype(np.uint32)
    # PCL FLOAT32 rgb stores these integer bits, not the integer's float value.
    packed['rgb'] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    cloud.data = packed.tobytes()
    return cloud


class CameraLidarProjection(Node):
    def __init__(self):
        super().__init__('camera_lidar_projection')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.fixed_frame = param('fixed_frame', 'odom')
        self.max_range = param('max_range', 10.0)
        self.rectified = param('rectified_image', False)
        self.visibility_cell = param('visibility_cell', 2)
        self.depth_tolerance = param('depth_tolerance', 0.10)
        self.wait_seconds = param('transform_wait_seconds', 0.5)
        queue_size = param('queue_size', 10)
        slop = param('sync_slop_seconds', 0.05)
        if queue_size < 1 or slop <= 0 or self.wait_seconds <= 0:
            raise ValueError('Queue size and synchronization timeouts must be positive')
        self.bridge = CvBridge()
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.image_sub = message_filters.Subscriber(
            self, Image, param('image_topic', '/pylon_camera_node/image_raw'),
            qos_profile=qos_profile_sensor_data)
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, param('camera_info_topic', '/pylon_camera_node/camera_info'),
            qos_profile=qos_profile_sensor_data)
        self.cloud_sub = message_filters.Subscriber(
            self, PointCloud2, param('cloud_topic', '/os1_cloud_node/points'),
            qos_profile=qos_profile_sensor_data)
        self.camera_filter = message_filters.SimpleFilter()
        self.camera_sync = message_filters.TimeSynchronizer(
            [self.image_sub, self.info_sub], queue_size)
        self.camera_sync.registerCallback(self.on_camera)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.camera_filter, self.cloud_sub], queue_size, slop)
        self.sync.registerCallback(self.on_pair)
        self.image_pub = self.create_publisher(Image, '~/projection_image', 1)
        self.cloud_pub = self.create_publisher(PointCloud2, '~/points_rgb', 1)
        self.status_pub = self.create_publisher(String, '~/status', 1)
        self.pending = deque()
        self.queue_size = queue_size
        self.stats = dict(images=0, lidar_scans=0, calibrated_images=0,
                          paired=0, projected=0, dropped=0, last_status='Waiting for camera and LiDAR')
        self.image_sub.registerCallback(lambda _: self.count('images'))
        self.cloud_sub.registerCallback(lambda _: self.count('lidar_scans'))
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.02, self.process_pending, clock=self.wall_clock)
        self.status_timer = self.create_timer(1.0, self.publish_status, clock=self.wall_clock)
        self.get_logger().info(
            f'Camera projection reference | frame={self.fixed_frame} | slop={slop}s | '
            'RGB appearance only; no semantic predictions or OHM map updates')

    def count(self, name):
        self.stats[name] += 1

    def on_camera(self, image, info):
        if (image.header.frame_id != info.header.frame_id
                or image.width != info.width or image.height != info.height
                or info.k[0] <= 0 or info.k[4] <= 0):
            self.stats['last_status'] = 'Camera calibration/frame/dimensions do not match image'
            return
        self.count('calibrated_images')
        self.camera_filter.signalMessage(CameraFrame(image, info))

    def on_pair(self, camera, cloud):
        self.count('paired')
        if (not stamp_ns(camera.header.stamp) or not stamp_ns(cloud.header.stamp)
                or not camera.header.frame_id or not cloud.header.frame_id):
            self.count('dropped')
            self.stats['last_status'] = 'Rejected empty frame or zero acquisition timestamp'
            return
        if len(self.pending) >= self.queue_size:
            self.pending.popleft()
            self.count('dropped')
        self.pending.append((time.monotonic(), camera, cloud))

    def process_pending(self):
        for _ in range(len(self.pending)):
            queued, camera, cloud = self.pending.popleft()
            try:
                world_camera = transform_matrix(self.buffer.lookup_transform(
                    self.fixed_frame, camera.header.frame_id,
                    Time.from_msg(camera.header.stamp)).transform)
                world_lidar = transform_matrix(self.buffer.lookup_transform(
                    self.fixed_frame, cloud.header.frame_id,
                    Time.from_msg(cloud.header.stamp)).transform)
            except TransformException as error:
                self.stats['last_status'] = f'Waiting for acquisition-time TF: {error}'
                if time.monotonic() - queued < self.wait_seconds:
                    self.pending.append((queued, camera, cloud))
                else:
                    self.count('dropped')
                continue
            try:
                self.project_pair(camera, cloud, world_camera, world_lidar)
            except (CvBridgeError, ValueError, TypeError) as error:
                self.count('dropped')
                self.stats['last_status'] = f'Projection rejected: {error}'
                self.get_logger().warning(str(error), throttle_duration_sec=2.0)

    def project_pair(self, camera, cloud, world_camera, world_lidar):
        if not self.rectified and camera.info.distortion_model not in ('', 'plumb_bob'):
            raise ValueError('Raw projection currently supports plumb_bob calibration only')
        bgr = self.bridge.imgmsg_to_cv2(camera.image, desired_encoding='bgr8')
        points = xyz_from_cloud(cloud)
        intrinsic = (np.asarray(camera.info.p).reshape(3, 4)[:, :3] if self.rectified
                     else np.asarray(camera.info.k).reshape(3, 3))
        distortion = [] if self.rectified else camera.info.d
        projection = project_visible_points(
            points, bgr, intrinsic, distortion, np.linalg.solve(world_camera, world_lidar),
            self.max_range, self.visibility_cell, self.depth_tolerance)
        overlay = self.bridge.cv2_to_imgmsg(draw_projection(bgr, projection, self.max_range), 'bgr8')
        overlay.header = camera.header
        self.image_pub.publish(overlay)
        # Construct a separate header; never mutate the input cloud's frame ID.
        header = Header(stamp=cloud.header.stamp, frame_id=self.fixed_frame)
        world_points = transform_points(points[projection.source_indices], world_lidar)
        self.cloud_pub.publish(rgb_cloud(header, self.fixed_frame, world_points, projection.rgb))
        self.count('projected')
        self.stats.update(
            last_status='Projection published', visible_points=len(projection.pixels),
            image_stamp_ns=stamp_ns(camera.header.stamp),
            lidar_stamp_ns=stamp_ns(cloud.header.stamp),
            image_minus_lidar_ms=(stamp_ns(camera.header.stamp)-stamp_ns(cloud.header.stamp))/1e6)
        self.publish_status()

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(dict(self.stats, pending=len(self.pending)))))


def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarProjection()
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
