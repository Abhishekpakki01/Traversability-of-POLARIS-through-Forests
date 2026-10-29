"""Exercise the real OHM node with known poses, rays and expected occupied cells."""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


class MappingHarness:
    def __init__(self):
        self.node = rclpy.create_node('ohm_pose_test_' + uuid.uuid4().hex)
        prefix = '/ohm_test_' + uuid.uuid4().hex
        self.cloud_pub = self.node.create_publisher(
            PointCloud2, prefix + '/cloud', qos_profile_sensor_data)
        self.tf_pub = self.node.create_publisher(TFMessage, prefix + '/tf', 10)
        static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_pub = self.node.create_publisher(TFMessage, prefix + '/tf_static', static_qos)
        self.markers = []
        self.poses = []
        self.marker_sub = self.node.create_subscription(
            Marker, prefix + '/map', self.markers.append, 10)
        self.pose_sub = self.node.create_subscription(
            PoseStamped, prefix + '/pose', self.poses.append, 10)
        executable = os.environ.get('OHM_NODE_EXECUTABLE', str(
            Path(__file__).resolve().parents[3] / 'build/semantic_mapping/ohm_occupancy_pose_node'))
        self.log = tempfile.TemporaryFile(mode='w+')
        self.process = subprocess.Popen([
            executable, '--ros-args', '-p', 'resolution:=1.0',
            '-r', '/os1_cloud_node/points:=' + prefix + '/cloud',
            '-r', '/semantic_voxel_map_cpp:=' + prefix + '/map',
            '-r', '/vehicle_pose:=' + prefix + '/pose',
            '-r', '/tf:=' + prefix + '/tf', '-r', '/tf_static:=' + prefix + '/tf_static',
        ], stdout=self.log, stderr=subprocess.STDOUT)

    def wait_for(self, predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.process.poll() is None, self.read_log()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError('Timed out waiting for ROS messages\n' + self.read_log())

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def read_log(self):
        self.log.seek(0)
        return self.log.read()

    @staticmethod
    def transform(parent, child, stamp, xyz, yaw=0.0):
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.header.stamp.sec = stamp
        transform.transform.translation.x = float(xyz[0])
        transform.transform.translation.y = float(xyz[1])
        transform.transform.translation.z = float(xyz[2])
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        return transform

    def set_poses(self, poses, sensor_offset=(0.5, 0.0, 0.4)):
        self.static_pub.publish(TFMessage(transforms=[
            self.transform('base_link', 'lidar', 1, sensor_offset)]))
        self.tf_pub.publish(TFMessage(transforms=poses))
        self.spin_for(0.2)

    def send_cloud(self, stamp, points, frame='lidar'):
        header = Header(frame_id=frame)
        header.stamp.sec = stamp
        self.cloud_pub.publish(create_cloud_xyz32(header, points))

    def map_for(self, stamp, points):
        count = len(self.markers)
        self.send_cloud(stamp, points)
        self.wait_for(lambda: len(self.markers) > count)
        marker = self.markers[-1]
        assert marker.header.frame_id == 'odom'
        assert marker.header.stamp.sec == stamp
        assert marker.pose.orientation.w == 1.0
        return {(p.x, p.y, p.z) for p in marker.points}

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.node.destroy_node()
        self.log.close()


@pytest.fixture
def mapping():
    rclpy.init()
    harness = None
    try:
        harness = MappingHarness()
        harness.wait_for(lambda: harness.cloud_pub.get_subscription_count() > 0
                         and harness.tf_pub.get_subscription_count() > 0
                         and harness.static_pub.get_subscription_count() > 0
                         and harness.node.count_publishers(harness.marker_sub.topic_name) > 0
                         and harness.node.count_publishers(harness.pose_sub.topic_name) > 0)
        yield harness
    finally:
        if harness is not None:
            harness.close()
        rclpy.shutdown()


def test_world_alignment_and_occupied_only(mapping):
    # The latest available pose is different: the first cloud must use time 10.
    mapping.set_poses([
        mapping.transform('odom', 'base_link', 10, (10.2, 20.2, 1.2), math.pi / 2),
        mapping.transform('odom', 'base_link', 20, (11.2, 21.2, 1.2)),
    ])
    expected = {(10.5, 22.5, 1.5), (8.5, 20.5, 1.5), (10.5, 20.5, 3.5)}
    # Three separate hits also check that every ray is integrated, including the last.
    result = mapping.map_for(10, [(2, 0, 0), (0, 2, 0), (0, 0, 2),
                                  (float('inf'), 0, 0), (float('nan'), 0, 0), (20, 0, 0)])
    assert result == expected  # No unknown region cells or traversed free cells.
    mapping.wait_for(lambda: len(mapping.poses) == 1)
    pose = mapping.poses[-1]
    assert pose.header.frame_id == 'odom'
    assert pose.header.stamp.sec == 10
    assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (10.2, 20.2, 1.2)
    assert pose.pose.orientation.z == pytest.approx(math.sin(math.pi / 4))

    # Same world surfaces observed after translation AND rotation of the vehicle.
    assert mapping.map_for(20, [(-1.5, 1.5, 0), (-3.5, -0.5, 0), (-1.5, -0.5, 2)]) == expected
    mapping.wait_for(lambda: len(mapping.poses) == 2)
    assert mapping.poses[-1].pose.position.x == pytest.approx(11.2)


def test_missing_pose_skips_scan_without_latest_fallback(mapping):
    mapping.set_poses([
        mapping.transform('odom', 'base_link', 10, (0, 0, 0)),
        mapping.transform('odom', 'base_link', 20, (2, 0, 0)),
    ], sensor_offset=(0, 0, 0))
    for stamp, frame in [(21, 'lidar'), (15, 'missing_sensor'), (0, 'lidar')]:
        mapping.send_cloud(stamp, [(2.2, 0.2, 0.2)], frame)
        mapping.spin_for(0.3)
        assert not mapping.markers
        assert not mapping.poses
    # TF interpolates the pose at time 15; rejected scans must not alter the map.
    assert mapping.map_for(15, [(2.2, 0.2, 0.2)]) == {(3.5, 0.5, 0.5)}
    mapping.wait_for(lambda: len(mapping.poses) == 1)
    assert mapping.poses[-1].pose.position.x == pytest.approx(1.0)


def test_free_space_evidence_removes_occupied_voxel(mapping):
    mapping.set_poses([
        mapping.transform('odom', 'base_link', 10, (0, 0, 0)),
        mapping.transform('odom', 'base_link', 20, (0, 0, 0)),
    ], sensor_offset=(0, 0, 0))
    assert mapping.map_for(10, [(2.2, 0.2, 0.2)]) == {(2.5, 0.5, 0.5)}
    # OHM's default 0.9 hit needs >10 misses at 0.45 to become free.
    for stamp in range(11, 21):
        result = mapping.map_for(stamp, [(4.2, 0.2, 0.2), (4.3, 0.2, 0.2)])
    assert result == {(4.5, 0.5, 0.5)}


def test_free_space_rays_start_at_the_moving_sensor(mapping):
    mapping.set_poses([
        mapping.transform('odom', 'base_link', 10, (0, 0, 0)),
        mapping.transform('odom', 'base_link', 11, (10, 0, 0)),
        mapping.transform('odom', 'base_link', 20, (10, 0, 0)),
    ], sensor_offset=(0, 0, 0))
    assert mapping.map_for(10, [(2.2, 0.2, 0.2)]) == {(2.5, 0.5, 0.5)}
    for stamp in range(11, 21):
        result = mapping.map_for(stamp, [(2.2, 0.2, 0.2)])
    # Starting rays at world zero would incorrectly clear the earlier obstacle.
    assert result == {(2.5, 0.5, 0.5), (12.5, 0.5, 0.5)}
