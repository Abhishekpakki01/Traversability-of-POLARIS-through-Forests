#!/usr/bin/env python3
"""Fuse camera predictions into the existing occupied OHM grid, on a new topic."""

from collections import OrderedDict, deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from semantic_mapping.camera_projection import (
    draw_projection, project_visible_points, transform_matrix, transform_points, xyz_from_cloud)
from semantic_mapping.goose_model import GoosePredictor
from semantic_mapping.semantic_fusion import (
    CLASS_NAMES, PALETTE, SemanticEvidence, surface_observations, voxel_keys)


def ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


@dataclass
class CameraFrame:
    image: Image
    info: CameraInfo

    @property
    def header(self):
        return self.image.header


@dataclass
class MapFrame:
    marker: Marker
    centres: np.ndarray
    keys: np.ndarray
    occupied: frozenset


def semantic_cloud(map_frame, labels, confidence, observations):
    """Keep geometry, class, posterior maximum and evidence count inspectable."""
    dtype = np.dtype({'names': ['x', 'y', 'z', 'rgb', 'label', 'confidence', 'observations'],
                      'formats': ['<f4', '<f4', '<f4', '<u4', 'u1', '<f4', '<u4'],
                      'offsets': [0, 4, 8, 12, 16, 20, 24], 'itemsize': 28})
    packed = np.zeros(len(labels), dtype=dtype)
    for axis, field in enumerate(('x', 'y', 'z')):
        packed[field] = map_frame.centres[:, axis]
    rgb = PALETTE[labels].astype(np.uint32)
    packed['rgb'] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    packed['label'], packed['confidence'], packed['observations'] = labels, confidence, observations
    cloud = PointCloud2(header=map_frame.marker.header, height=1, width=len(labels),
                        point_step=28, row_step=len(labels)*28, is_dense=True)
    cloud.fields = [PointField(name=name, offset=offset, datatype=typ, count=1)
                    for name, offset, typ in [('x', 0, PointField.FLOAT32),
                        ('y', 4, PointField.FLOAT32), ('z', 8, PointField.FLOAT32),
                        ('rgb', 12, PointField.FLOAT32), ('label', 16, PointField.UINT8),
                        ('confidence', 20, PointField.FLOAT32), ('observations', 24, PointField.UINT32)]]
    cloud.data = packed.tobytes()
    return cloud


class CameraSemanticVoxel(Node):
    def __init__(self):
        super().__init__('camera_semantic_voxel')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.fixed_frame = param('fixed_frame', 'odom')
        self.resolution = param('resolution', 0.35)
        self.max_range = param('max_range', 10.0)
        self.wait_seconds = param('transform_wait_seconds', 2.0)
        self.slop = param('sync_slop_seconds', 0.05)
        self.min_observations = param('min_observations', 2)
        self.min_confidence = param('min_confidence', 0.6)
        self.strict_voxel_occlusion = param('strict_voxel_occlusion', False)
        if self.resolution <= 0 or self.max_range <= 0 or self.wait_seconds <= 0 or self.slop <= 0:
            raise ValueError('Range, resolution and synchronization limits must be positive')
        checkpoint = Path(param('model_path', str(Path.home()/
            'semantic_map_ws/models/goose/ppliteseg_class_512.pth'))).expanduser()
        config_path = Path(get_package_share_directory('semantic_mapping'))/'config/goose_classes.json'
        classes = json.loads(config_path.read_text())['classes']
        if [value['id'] for value in classes] != list(range(64)):
            raise ValueError('GOOSE class mapping is incomplete or out of order')
        groups = [CLASS_NAMES.index(value['group']) for value in classes]
        self.get_logger().info(f'Loading official GOOSE 64-class model: {checkpoint}')
        self.predictor = GoosePredictor(checkpoint, groups, param('torch_threads', 2))
        self.evidence = SemanticEvidence(self.min_observations, self.min_confidence)
        self.bridge = CvBridge()
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.inference_pool = ThreadPoolExecutor(max_workers=1)
        self.future, self.job = None, None
        self.generation = 0
        self.pending = deque(maxlen=8)
        self.maps = OrderedDict()
        self.current_map = None
        self.stats = dict(paired=0, projected=0, fused_frames=0, dropped=0,
                          last_status='Waiting for image, calibration, LiDAR and occupied map')
        self.last_image_stamp = -1
        self.image_sub = message_filters.Subscriber(self, Image,
            '/pylon_camera_node/image_raw', qos_profile=qos_profile_sensor_data)
        self.info_sub = message_filters.Subscriber(self, CameraInfo,
            '/pylon_camera_node/camera_info', qos_profile=qos_profile_sensor_data)
        self.cloud_sub = message_filters.Subscriber(self, PointCloud2,
            '/os1_cloud_node/points', qos_profile=qos_profile_sensor_data)
        self.camera_filter = message_filters.SimpleFilter()
        self.camera_sync = message_filters.TimeSynchronizer([self.image_sub, self.info_sub], 10)
        self.camera_sync.registerCallback(self.on_camera)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.camera_filter, self.cloud_sub], 10, self.slop)
        self.sync.registerCallback(self.on_pair)
        self.map_sub = self.create_subscription(Marker, '/semantic_voxel_map_cpp', self.on_map, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(PointCloud2, '/semantic_voxel_map', latched)
        self.image_pub = self.create_publisher(Image, '~/segmentation_image', 1)
        self.label_pub = self.create_publisher(Image, '~/labels', 1)
        self.projection_pub = self.create_publisher(Image, '~/projection_image', 1)
        self.status_pub = self.create_publisher(String, '~/status', latched)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.03, self.tick, clock=self.wall_clock)
        self.status_timer = self.create_timer(1.0, self.publish_status, clock=self.wall_clock)
        self.get_logger().info('Ready: camera classes -> measured visible occupied surface -> multi-frame evidence. '
                               'Unknown voxels remain gray; occupancy integration is unchanged.')

    def on_camera(self, image, info):
        if (image.header.frame_id != info.header.frame_id
                or image.width != info.width or image.height != info.height
                or info.k[0] <= 0 or info.k[4] <= 0 or info.distortion_model not in ('', 'plumb_bob')):
            self.stats['last_status'] = 'Rejected mismatched or unsupported raw camera calibration'
            return
        self.camera_filter.signalMessage(CameraFrame(image, info))

    def on_pair(self, camera, cloud):
        self.stats['paired'] += 1
        if (not ns(camera.header.stamp) or not ns(cloud.header.stamp)
                or not camera.header.frame_id or not cloud.header.frame_id):
            self.stats['dropped'] += 1
            return
        if len(self.pending) == self.pending.maxlen:
            self.stats['dropped'] += 1
        self.pending.append((time.monotonic(), camera, cloud))

    def on_map(self, marker):
        if marker.header.frame_id != self.fixed_frame or marker.type != Marker.CUBE_LIST:
            self.stats['last_status'] = 'Rejected map: expected CUBE_LIST in fixed_frame'
            return
        scales = np.array([marker.scale.x, marker.scale.y, marker.scale.z])
        pose = marker.pose
        transform = [pose.position.x, pose.position.y, pose.position.z,
                     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        if (not np.allclose(scales, self.resolution, atol=1e-6)
                or not np.allclose(transform, [0, 0, 0, 0, 0, 0, 1], atol=1e-6)):
            self.stats['last_status'] = 'Rejected map: resolution or marker pose differs from expected OHM grid'
            return
        centres = np.array([[p.x, p.y, p.z] for p in marker.points]).reshape(-1, 3)
        if not np.isfinite(centres).all():
            return
        keys = voxel_keys(centres, self.resolution)
        if not np.allclose(centres, (keys+0.5)*self.resolution, atol=1e-6):
            self.stats['last_status'] = 'Rejected map: unsupported voxel-grid origin'
            return
        stamp = ns(marker.header.stamp)
        if self.current_map is not None and stamp < ns(self.current_map.marker.header.stamp):
            self.generation += 1
            self.maps.clear()
            self.pending.clear()
            self.evidence = SemanticEvidence(self.min_observations, self.min_confidence)
            self.last_image_stamp = -1
        snapshot = MapFrame(marker, centres, keys, frozenset(map(tuple, keys)))
        self.maps[stamp] = snapshot
        while len(self.maps) > 32:
            self.maps.popitem(last=False)
        self.current_map = snapshot
        self.evidence.retain_occupied(snapshot.occupied)
        self.publish_map()

    def tick(self):
        if self.future is not None:
            if not self.future.done():
                return
            try:
                result = self.future.result()
                if self.job == self.generation and self.current_map is not None:
                    self.finish(result)
            except Exception as error:
                self.stats['last_status'] = f'Fusion rejected: {error}'
                self.get_logger().error(str(error), throttle_duration_sec=2.0)
                self.stats['dropped'] += 1
            self.future, self.job = None, None
        if not self.pending:
            return
        # Select the newest pair whose exact LiDAR-time map and TF are available.
        for _ in range(len(self.pending)):
            queued, camera, cloud = self.pending.pop()
            if ns(camera.header.stamp) <= self.last_image_stamp:
                self.stats['dropped'] += 1
                continue
            snapshot = self.maps.get(ns(cloud.header.stamp))
            try:
                if snapshot is None:
                    raise LookupError('Waiting for occupied map at the paired LiDAR timestamp')
                world_camera = transform_matrix(self.buffer.lookup_transform(
                    self.fixed_frame, camera.header.frame_id, Time.from_msg(camera.header.stamp)).transform)
                world_lidar = transform_matrix(self.buffer.lookup_transform(
                    self.fixed_frame, cloud.header.frame_id, Time.from_msg(cloud.header.stamp)).transform)
            except (TransformException, LookupError) as error:
                self.stats['last_status'] = str(error)
                if time.monotonic()-queued < self.wait_seconds:
                    self.pending.appendleft((queued, camera, cloud))
                else:
                    self.stats['dropped'] += 1
                continue
            self.last_image_stamp = ns(camera.header.stamp)
            self.job = self.generation
            self.future = self.inference_pool.submit(self.process, camera, cloud, snapshot,
                                               world_camera, world_lidar)
            break

    def process(self, camera, cloud, snapshot, world_camera, world_lidar):
        started = time.monotonic()
        bgr = self.bridge.imgmsg_to_cv2(camera.image, 'bgr8')
        points = xyz_from_cloud(cloud)
        projection = project_visible_points(points, bgr, camera.info.k, camera.info.d,
            np.linalg.solve(world_camera, world_lidar), self.max_range,
            visibility_cell=4, depth_tolerance=0.0)
        probabilities = self.predictor.predict(bgr)
        height, width = bgr.shape[:2]
        pixels = np.floor((projection.pixels+0.5)*[512/width, 512/height]).astype(int)
        pixels = np.clip(pixels, 0, 511)
        sampled = probabilities[pixels[:, 1], pixels[:, 0]]
        world_points = transform_points(points[projection.source_indices], world_lidar)
        observations, visibility = surface_observations(world_points, sampled,
            world_camera[:3, 3], snapshot.occupied, self.resolution, self.strict_voxel_occlusion)
        labels = probabilities.argmax(axis=2).astype(np.uint8)
        labels[probabilities.max(axis=2) < self.min_confidence] = 0
        labels = cv2.resize(labels, (width, height), interpolation=cv2.INTER_NEAREST)
        overlay = cv2.addWeighted(bgr, 0.5, PALETTE[labels][:, :, ::-1], 0.5, 0)
        # Legend is image output only; the saved voxel attributes retain exact IDs.
        for index, (name, color) in enumerate(zip(CLASS_NAMES, PALETTE)):
            y = 34+index*34
            cv2.rectangle(overlay, (12, y-21), (34, y+1), tuple(int(c) for c in color[::-1]), -1)
            cv2.putText(overlay, name, (44, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return dict(observations=observations, visibility=visibility, labels=labels, overlay=overlay,
                    projection=draw_projection(bgr, projection, self.max_range), header=camera.header,
                    image_stamp_ns=ns(camera.header.stamp), lidar_stamp_ns=ns(cloud.header.stamp),
                    processing_seconds=time.monotonic()-started)

    def finish(self, result):
        updated = self.evidence.update(result['observations'], result['image_stamp_ns'],
                                       self.current_map.occupied)
        for key, encoding, publisher in [('overlay', 'bgr8', self.image_pub),
                                         ('labels', 'mono8', self.label_pub),
                                         ('projection', 'bgr8', self.projection_pub)]:
            message = self.bridge.cv2_to_imgmsg(result[key], encoding)
            message.header = result['header']
            publisher.publish(message)
        self.stats.update(last_status='Semantic observations fused',
            processing_seconds=result['processing_seconds'], updated_voxels=updated,
            visibility=result['visibility'], image_stamp_ns=result['image_stamp_ns'],
            lidar_stamp_ns=result['lidar_stamp_ns'],
            image_minus_lidar_ms=(result['image_stamp_ns']-result['lidar_stamp_ns'])/1e6)
        self.stats['projected'] += 1
        self.stats['fused_frames'] += bool(updated)
        self.publish_map()
        self.publish_status()

    def publish_map(self):
        if self.current_map is None:
            return
        labels, confidence, observations = self.evidence.query(self.current_map.keys)
        self.map_pub.publish(semantic_cloud(self.current_map, labels, confidence, observations))
        self.stats['occupied_voxels'] = len(labels)
        self.stats['class_counts'] = {name: int(np.count_nonzero(labels == i))
                                     for i, name in enumerate(CLASS_NAMES)}
        self.stats['voxels_with_evidence'] = len(self.evidence.voxels)

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(dict(self.stats,
            pending=len(self.pending), inference_busy=self.future is not None,
            model='GOOSE PP-LiteSeg-B 64 classes', min_observations=self.min_observations,
            min_confidence=self.min_confidence,
            visibility_mode='solid_voxel_rays' if self.strict_voxel_occlusion else 'measured_lidar_depth'))))

    def close(self):
        self.inference_pool.shutdown(wait=True, cancel_futures=True)
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraSemanticVoxel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
