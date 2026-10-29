#!/usr/bin/env python3
"""Headless bag/mapper smoke test with saved camera and occupied-voxel evidence.

Run in a separate ROS_DOMAIN_ID to avoid other running mappers. This checks a
short recording segment; it does not establish mapping or semantic accuracy.
"""

import argparse
import json
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import yaml


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cloud-topic', default='/os1_cloud_node/points')
    parser.add_argument('--image-topic', default='/pylon_camera_node/image_raw')
    parser.add_argument('--fixed-frame', default='odom')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--rate', default=0.5, type=float)
    parser.add_argument('--seconds', default=20, type=float)
    parser.add_argument('--projection', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    qos_path = args.output/'playback_qos.yaml'
    qos_path.write_text(yaml.safe_dump({
        args.cloud_topic: dict(history='keep_last', depth=5, reliability='best_effort', durability='volatile'),
        '/tf_static': dict(history='keep_last', depth=10, reliability='reliable', durability='transient_local'),
    }))
    rclpy.init()
    node = rclpy.create_node('mapping_recording_check')
    bridge = CvBridge()
    counts = dict(cloud=0, marker=0, pose=0, image=0, projection=0, rgb_cloud=0)
    last = {}
    poses = []
    statuses = []
    def receive(name, message):
        counts[name] += 1
        last[name] = message
        if name == 'pose':
            poses.append([ns(message.header.stamp), message.pose.position.x,
                          message.pose.position.y, message.pose.position.z])
    subscriptions = []
    for name, typ, topic, qos in [
        ('cloud', PointCloud2, args.cloud_topic, qos_profile_sensor_data),
        ('image', Image, args.image_topic, qos_profile_sensor_data),
        ('marker', Marker, '/semantic_voxel_map_cpp', 10),
        ('pose', PoseStamped, '/vehicle_pose', 10),
        ('projection', Image, '/camera_lidar_projection/projection_image', 1),
        ('rgb_cloud', PointCloud2, '/camera_lidar_projection/points_rgb', 1),
    ]:
        subscriptions.append(node.create_subscription(
            typ, topic, lambda msg, name=name: receive(name, msg), qos))
    subscriptions.append(node.create_subscription(
        String, '/camera_lidar_projection/status',
        lambda msg: statuses.append(json.loads(msg.data)), 1))
    executable_dir = Path(get_package_prefix('semantic_mapping'))/'lib/semantic_mapping'
    commands = [('mapper', [str(executable_dir/'ohm_occupancy_pose_node'), '--ros-args',
                            '-p', 'use_sim_time:=true', '-p', 'fixed_frame:='+args.fixed_frame,
                            '-p', 'base_frame:='+args.base_frame,
                            '-r', '/os1_cloud_node/points:='+args.cloud_topic])]
    if args.projection:
        commands.append(('projection', [str(executable_dir/'camera_lidar_projection_node.py'),
                                        '--ros-args', '-p', 'use_sim_time:=true']))
    topics = ['/tf', '/tf_static', args.cloud_topic, args.image_topic]
    if args.projection:
        topics.append('/pylon_camera_node/camera_info')
    commands.append(('playback', ['ros2', 'bag', 'play', str(args.bag), '--clock',
                                 '--rate', str(args.rate), '--delay', '2',
                                 '--disable-keyboard-controls', '--qos-profile-overrides-path',
                                 str(qos_path), '--topics']+topics))
    processes, logs = [], []
    report = dict(bag=str(args.bag), fixed_frame=args.fixed_frame,
                  base_frame=args.base_frame, cloud_topic=args.cloud_topic,
                  playback_rate=args.rate, wall_seconds=args.seconds)
    try:
        for label, command in commands:
            log = open(args.output/(label+'.log'), 'w+')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        deadline = time.monotonic()+args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            for process in processes[:-1]:
                if process.poll() is not None:
                    raise RuntimeError('Mapper or projection node exited; inspect saved logs')
        report['received'] = counts
        report['last_projection_status'] = statuses[-1] if statuses else None
        report['poses'] = poses
        if poses:
            report['first_to_last_vehicle_displacement_m'] = float(
                np.linalg.norm(np.array(poses[-1][1:])-np.array(poses[0][1:])))
        if 'image' in last:
            image = last['image']
            cv2.imwrite(str(args.output/'camera_reference.png'), bridge.imgmsg_to_cv2(image, 'bgr8'))
            report['camera_stamp_ns'] = ns(image.header.stamp)
            report['camera_frame'] = image.header.frame_id
        if 'projection' in last:
            cv2.imwrite(str(args.output/'camera_lidar_projection.png'),
                        bridge.imgmsg_to_cv2(last['projection'], 'bgr8'))
        if 'rgb_cloud' in last:
            from sensor_msgs_py.point_cloud2 import read_points_numpy
            rgb_message = last['rgb_cloud']
            report['rgb_surface_point_count'] = rgb_message.width*rgb_message.height
            report['rgb_cloud_frame'] = rgb_message.header.frame_id
            xyz = read_points_numpy(rgb_message, field_names=('x', 'y', 'z'))
            assert np.isfinite(xyz).all(), 'Nonfinite projected world points'
            (args.output/'rgb_surface_points.cdr').write_bytes(
                __import__('rclpy.serialization', fromlist=['serialize_message']).serialize_message(rgb_message))
        if 'marker' in last:
            marker = last['marker']
            xyz = np.array([[p.x, p.y, p.z] for p in marker.points], dtype=np.float64)
            assert len(xyz) and np.isfinite(xyz).all(), 'Empty/nonfinite occupied map'
            assert marker.header.frame_id == args.fixed_frame
            report['occupied_voxels'] = len(xyz)
            report['map_stamp_ns'] = ns(marker.header.stamp)
            np.save(args.output/'occupied_voxel_centers.npy', xyz)
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            figure = plt.figure(figsize=(10, 8))
            axis = figure.add_subplot(111, projection='3d')
            axis.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], s=5, c='#24a148', marker='s')
            if poses:
                path = np.array(poses)[:, 1:]
                axis.plot(path[:, 0], path[:, 1], path[:, 2], color='red', linewidth=2)
                axis.scatter(*path[-1], color='red', s=60, marker='^', label='Vehicle pose')
                axis.legend()
            axis.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)',
                     title='Occupied OHM voxels — '+args.bag.name+'\nHeadless plot of published data')
            axis.set_box_aspect(np.maximum(np.ptp(xyz, axis=0), 1.0))
            figure.tight_layout()
            figure.savefig(args.output/'occupied_voxels.png', dpi=160)
            plt.close(figure)
        report['passed'] = counts['marker'] > 0 and counts['pose'] > 0
        if args.projection:
            report['passed'] &= counts['projection'] > 0 and counts['rgb_cloud'] > 0
        (args.output/'report.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k != 'poses'}, indent=2), flush=True)
        if not report['passed']:
            raise RuntimeError('Recording check did not receive required outputs; inspect report/logs')
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
