#!/usr/bin/env python3
"""Offer a local display of saved maps without changing the full map or labels."""
from collections import OrderedDict

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2

from semantic_mapping.offline_demo import local_cloud, stamp_ns


class LocalView(Node):
    def __init__(self):
        super().__init__('offline_demo_local_view')
        self.radius = self.declare_parameter('radius_m', 15.).value
        # Large clouds (e.g. 180k LiDAR-class voxels) make RViz sluggish; cap local updates.
        rate = self.declare_parameter('max_rate_hz', 0.).value  # 0 = publish every map
        self.min_period_ns = int(1e9/rate) if rate > 0 else 0
        self.last_published, self.deferred = {}, {}
        if self.min_period_ns:
            # Publish the newest skipped map shortly after, so the final map is never left out.
            # Wall-clock timer: simulated time stops when the bag ends.
            self.create_timer(self.min_period_ns/1e9, self.flush_deferred, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.poses, self.pending = OrderedDict(), OrderedDict()
        self.last_pose = -1
        self.publishers_by_kind = {}
        self.subscriptions_by_kind = {}
        for kind, topic in [('semantic', '/semantic_voxel_map'), ('elevation', '/offline_demo/elevation'),
                            ('lidar', '/lidar/features')]:
            self.publishers_by_kind[kind] = self.create_publisher(PointCloud2,
                '/offline_demo/local_'+kind, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.subscriptions_by_kind[kind] = self.create_subscription(PointCloud2, topic,
                lambda message, kind=kind: self.on_cloud(kind, message), 5)
        self.pose_sub = self.create_subscription(PoseStamped, '/vehicle_pose', self.on_pose, 10)
        self.get_logger().info(f'Local display: {self.radius:g} m horizontal radius; complete maps remain on their original topics')

    def on_pose(self, pose):
        stamp = stamp_ns(pose)
        if stamp < self.last_pose:
            self.poses.clear(); self.pending.clear(); self.last_published.clear(); self.deferred.clear()
        self.last_pose = stamp
        self.poses[stamp] = pose
        while len(self.poses) > 16:
            self.poses.popitem(last=False)
        for kind, time in list(self.pending):
            if time == stamp:
                self.on_cloud(kind, self.pending.pop((kind, time)))

    def on_cloud(self, kind, cloud):
        stamp = stamp_ns(cloud)
        if stamp not in self.poses:
            self.pending[(kind, stamp)] = cloud
            while len(self.pending) > 32:
                self.pending.popitem(last=False)
            return
        if self.min_period_ns and 0 <= stamp-self.last_published.get(kind, -10**19) < self.min_period_ns:
            self.deferred[kind] = (cloud, self.poses[stamp])
            return
        self.publish(kind, cloud, self.poses[stamp])

    def publish(self, kind, cloud, pose):
        self.last_published[kind] = stamp_ns(cloud)
        self.deferred.pop(kind, None)
        self.publishers_by_kind[kind].publish(local_cloud(cloud, pose, self.radius))

    def flush_deferred(self):
        for kind, (cloud, pose) in list(self.deferred.items()):
            if stamp_ns(cloud) > self.last_published.get(kind, -1):
                self.publish(kind, cloud, pose)


def main():
    rclpy.init()
    node = LocalView()
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
