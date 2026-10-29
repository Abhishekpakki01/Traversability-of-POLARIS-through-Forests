#!/usr/bin/env python3
"""Run the separate moving TartanDrive terrain integration and save actual semantic outputs."""

import argparse
from collections import OrderedDict
import json
from pathlib import Path
import signal
import subprocess
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from semantic_mapping.semantic_fusion import CLASS_NAMES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=Path.home()/'thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=80)
    parser.add_argument('--rate', type=float, default=0.5)
    parser.add_argument('--strict-voxel-occlusion', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    qos = args.output/'playback_qos.yaml'
    qos.write_text('/velodyne_1/velodyne_points:\n  reliability: best_effort\n  durability: volatile\n'
                   '  history: keep_last\n  depth: 5\n/tf_static:\n  reliability: reliable\n'
                   '  durability: transient_local\n  history: keep_last\n  depth: 10\n')
    rclpy.init()
    node = rclpy.create_node('semantic_integration_check')
    bridge = CvBridge()
    counts, last = {}, {}
    occupied_maps, camera_frames, poses = OrderedDict(), OrderedDict(), []

    def stamp_ns(header):
        return header.stamp.sec*1_000_000_000+header.stamp.nanosec

    def receive(name, message):
        counts[name] = counts.get(name, 0)+1
        last[name] = message
        if name == 'occupied':
            occupied_maps[stamp_ns(message.header)] = message
            while len(occupied_maps) > 8:
                occupied_maps.popitem(last=False)
        if name == 'pose':
            p = message.pose.position
            poses.append([stamp_ns(message.header), p.x, p.y, p.z])
        if name == 'camera':
            camera_frames[stamp_ns(message.header)] = message
            while len(camera_frames) > 32:
                camera_frames.popitem(last=False)
        if name == 'segmentation':
            # Keep the actual camera exposure used by the last prediction.
            last['camera_at_segmentation'] = camera_frames.get(stamp_ns(message.header))

    subscriptions = []
    for name, typ, topic, qos_profile in [
        ('map', PointCloud2, '/semantic_voxel_map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ('status', String, '/camera_semantic_voxel/status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ('segmentation', Image, '/camera_semantic_voxel/segmentation_image', qos_profile_sensor_data),
        ('projection', Image, '/camera_semantic_voxel/projection_image', qos_profile_sensor_data),
        ('camera', Image, '/multisense/left/image_rect_color', qos_profile_sensor_data),
        ('pose', PoseStamped, '/vehicle_pose', 10),
        ('occupied', Marker, '/semantic_voxel_map_cpp', 10),
    ]:
        subscriptions.append(node.create_subscription(typ, topic,
            lambda message, name=name: receive(name, message), qos_profile))
    processes, logs = [], []
    try:
        log = open(args.output/'integration.log', 'w+')
        logs.append(log)
        launch = subprocess.Popen(['ros2', 'launch', 'semantic_mapping',
            'tartan_semantic.launch.py', 'rviz:=false',
            'strict_voxel_occlusion:='+str(args.strict_voxel_occlusion).lower()],
            stdout=log, stderr=subprocess.STDOUT)
        processes.append(launch)
        deadline = time.monotonic()+30
        while node.count_publishers('/semantic_voxel_map') == 0:
            rclpy.spin_once(node, timeout_sec=0.05)
            if launch.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Integration did not start; inspect integration.log')
        log = open(args.output/'playback.log', 'w+')
        logs.append(log)
        player = subprocess.Popen(['ros2', 'bag', 'play', str(args.bag), '--clock',
            '--delay', '2', '--rate', str(args.rate), '--read-ahead-queue-size', '10',
            '--disable-keyboard-controls', '--qos-profile-overrides-path', str(qos),
            '--topics', '/tf', '/tf_static', '/velodyne_1/velodyne_points',
            '/multisense/left/image_rect_color', '/lidar_odometry'],
            stdout=log, stderr=subprocess.STDOUT)
        processes.append(player)
        deadline = time.monotonic()+args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if launch.poll() is not None:
                raise RuntimeError('Integration exited; inspect integration.log')
        report = dict(bag=str(args.bag), rate=args.rate, wall_seconds=args.seconds, received=counts,
                      status=json.loads(last['status'].data) if 'status' in last else None)
        if poses:
            np.save(args.output/'vehicle_poses.npy', np.asarray(poses))
            report['first_to_last_vehicle_displacement_m'] = float(
                np.linalg.norm(np.asarray(poses[-1][1:])-np.asarray(poses[0][1:])))
            report['pose_stamp_span_seconds'] = (poses[-1][0]-poses[0][0])/1e9
        if last.get('camera_at_segmentation') is not None:
            last['camera'] = last['camera_at_segmentation']
        report['saved_image_stamps_ns'] = {}
        for name in ('camera', 'segmentation', 'projection'):
            if name in last:
                cv2.imwrite(str(args.output/(name+'.png')), bridge.imgmsg_to_cv2(last[name], 'bgr8'))
                report['saved_image_stamps_ns'][name] = stamp_ns(last[name].header)
        report['saved_camera_matches_segmentation'] = (
            report['saved_image_stamps_ns'].get('camera') ==
            report['saved_image_stamps_ns'].get('segmentation') and 'segmentation' in last)
        if 'map' in last:
            message = last['map']
            points = read_points(message)
            np.save(args.output/'semantic_voxels.npy', points)
            xyz = np.column_stack([points[name] for name in ('x', 'y', 'z')])
            labels = points['label']
            rgb = points['rgb'].view(np.uint32)
            colors = np.column_stack([(rgb >> 16)&255, (rgb >> 8)&255, rgb&255])/255
            assert message.header.frame_id == 'sensor_init'
            assert np.isfinite(xyz).all()
            occupied = occupied_maps.get(stamp_ns(message.header))
            assert occupied is not None, 'Missing original OHM geometry at colored-map timestamp'
            source = np.array([[p.x, p.y, p.z] for p in occupied.points])
            assert source.shape == xyz.shape and np.allclose(source, xyz, atol=1e-4)
            assert (points['observations'][labels != 0] >= 2).all()
            assert (points['confidence'][labels != 0] >= 0.6).all()
            report['occupied_geometry_matched'] = True
            report['occupied_voxels'] = len(xyz)
            report['classified_voxels'] = int(np.count_nonzero(labels))
            report['max_observations'] = int(points['observations'].max(initial=0))
            report['class_counts'] = {name: int(np.count_nonzero(labels == i))
                                      for i, name in enumerate(CLASS_NAMES)}
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig = plt.figure(figsize=(10, 8))
            axis = fig.add_subplot(111, projection='3d')
            unknown = labels == 0
            axis.scatter(*xyz[unknown].T, c=colors[unknown], marker='s', s=4, alpha=0.15)
            for i, name in enumerate(CLASS_NAMES[1:], 1):
                selected = labels == i
                if selected.any():
                    axis.scatter(*xyz[selected].T, c=colors[selected], marker='s', s=18, label=name)
            if 'pose' in last:
                path = np.asarray(poses)[:, 1:]
                axis.plot(*path.T, c='red', linewidth=1)
                pose = last['pose'].pose.position
                axis.scatter(pose.x, pose.y, pose.z, c='red', marker='^', s=60, label='Vehicle pose')
            axis.legend()
            axis.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)',
                     title='Occupied OHM voxels with fused camera class predictions\nHeadless plot of actual ROS output')
            axis.set_box_aspect(np.maximum(np.ptp(xyz, axis=0), 1))
            fig.tight_layout()
            fig.savefig(args.output/'semantic_voxel_map.png', dpi=160)
            plt.close(fig)
        report['passed'] = (report.get('classified_voxels', 0) > 0
                            and report.get('max_observations', 0) >= 2
                            and counts.get('segmentation', 0) >= 2
                            and report.get('first_to_last_vehicle_displacement_m', 0) > 10
                            and report.get('pose_stamp_span_seconds', 0) > 25
                            and report['saved_camera_matches_segmentation'])
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        if not report['passed']:
            raise RuntimeError('No confirmed multi-frame semantic output; inspect report and logs')
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
