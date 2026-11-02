#!/usr/bin/env python3
"""Offer a local display of saved maps without changing the full map or labels."""
from collections import OrderedDict

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2

from semantic_mapping.offline_demo import local_cloud, stamp_ns


class LocalView(Node):
    def __init__(self):
        super().__init__('offline_demo_local_view')
        self.radius = self.declare_parameter('radius_m', 15.).value
        self.poses, self.pending = OrderedDict(), OrderedDict()
        self.last_pose = -1
        self.publishers_by_kind = {}
        self.subscriptions_by_kind = {}
        for kind, topic in [('semantic', '/semantic_voxel_map'), ('elevation', '/offline_demo/elevation')]:
            self.publishers_by_kind[kind] = self.create_publisher(PointCloud2,
                '/offline_demo/local_'+kind, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.subscriptions_by_kind[kind] = self.create_subscription(PointCloud2, topic,
                lambda message, kind=kind: self.on_cloud(kind, message), 5)
        self.pose_sub = self.create_subscription(PoseStamped, '/vehicle_pose', self.on_pose, 10)
        self.get_logger().info(f'Local display: {self.radius:g} m horizontal radius; complete maps remain on their original topics')

    def on_pose(self, pose):
        stamp = stamp_ns(pose)
        if stamp < self.last_pose:
            self.poses.clear(); self.pending.clear()
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
        self.publishers_by_kind[kind].publish(local_cloud(cloud, self.poses[stamp], self.radius))


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
