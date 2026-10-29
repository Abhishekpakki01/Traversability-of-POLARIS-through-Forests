#!/usr/bin/env python3
"""Exercise the separate recorded-transform launch over the moving excerpt."""

import argparse
from collections import OrderedDict
import json
from pathlib import Path
import signal
import subprocess
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def stamp_ns(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=80)
    parser.add_argument('--rate', type=float, default=.5)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    package = Path(__file__).resolve().parents[1]
    bag = Path.home()/'thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2'
    rclpy.init()
    node = rclpy.create_node('recorded_projection_check')
    bridge = CvBridge()
    counts, last, images, poses = {}, {}, OrderedDict(), []

    def receive(name, message):
        counts[name] = counts.get(name, 0)+1
        last[name] = message
        if name == 'camera':
            images[stamp_ns(message)] = message
            while len(images) > 32:
                images.popitem(last=False)
        if name == 'projection':
            last['matching_camera'] = images.get(stamp_ns(message))
        if name == 'pose':
            p = message.pose.position
            poses.append([stamp_ns(message), p.x, p.y, p.z])

    subscriptions = []
    for name, typ, topic, qos in [
        ('camera', Image, '/multisense/left/image_rect_color', qos_profile_sensor_data),
        ('projection', Image, '/camera_lidar_projection/projection_image', 1),
        ('status', String, '/camera_lidar_projection/status', 1),
        ('camera_info', CameraInfo, '/tartan_recorded/camera/camera_info', 1),
        ('pose', PoseStamped, '/vehicle_pose', 10),
    ]:
        subscriptions.append(node.create_subscription(typ, topic,
            lambda message, name=name: receive(name, message), qos))
    processes, logs = [], []
    try:
        launch_log = open(args.output/'launch.log', 'w')
        logs.append(launch_log)
        launch = subprocess.Popen(['ros2', 'launch', 'semantic_mapping',
            'tartan_recorded_calibration.launch.py', 'rviz:=false'],
            stdout=launch_log, stderr=subprocess.STDOUT)
        processes.append(launch)
        deadline = time.monotonic()+30
        while not node.count_publishers('/camera_lidar_projection/status'):
            rclpy.spin_once(node, timeout_sec=.05)
            if launch.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Projection launch failed; see launch.log')
        player_log = open(args.output/'playback.log', 'w')
        logs.append(player_log)
        player = subprocess.Popen(['ros2', 'bag', 'play', str(bag), '--clock',
            '--delay', '2', '--rate', str(args.rate), '--read-ahead-queue-size', '10',
            '--disable-keyboard-controls', '--qos-profile-overrides-path',
            str(package/'config/tartan_playback_qos.yaml'), '--topics', '/tf', '/tf_static',
            '/velodyne_1/velodyne_points', '/multisense/left/image_rect_color', '/lidar_odometry'],
            stdout=player_log, stderr=subprocess.STDOUT)
        processes.append(player)
        deadline = time.monotonic()+args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.03)
            if launch.poll() is not None:
                raise RuntimeError('Projection launch exited unexpectedly')
        status = json.loads(last['status'].data) if 'status' in last else {}
        matching = last.get('matching_camera')
        stamps_match = matching is not None and stamp_ns(matching) == stamp_ns(last['projection'])
        report = {'bag': str(bag), 'rate': args.rate, 'received': counts,
            'projection_status': status, 'camera_overlay_stamps_match': stamps_match,
            'physical_lidar_calibration_validated': False}
        if poses:
            array = np.asarray(poses)
            np.save(args.output/'vehicle_poses.npy', array)
            report['pose_span_seconds'] = (poses[-1][0]-poses[0][0])/1e9
            report['endpoint_displacement_m'] = float(np.linalg.norm(array[-1, 1:]-array[0, 1:]))
        if stamps_match:
            cv2.imwrite(str(args.output/'camera.png'), bridge.imgmsg_to_cv2(matching, 'bgr8'))
            cv2.imwrite(str(args.output/'projection.png'), bridge.imgmsg_to_cv2(last['projection'], 'bgr8'))
            report['saved_image_stamp_ns'] = stamp_ns(matching)
        if 'camera_info' in last:
            info = last['camera_info']
            assert info.header.frame_id == 'tartan_recorded_rectified_optical'
            assert np.allclose(info.d, 0) and np.allclose(info.r, np.eye(3).reshape(-1))
            assert (info.width, info.height) == (1024, 544)
        report['passed'] = (stamps_match and status.get('projected', 0) >= 10
            and report.get('pose_span_seconds', 0) > 25
            and report.get('endpoint_displacement_m', 0) > 10)
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        if not report['passed']:
            raise RuntimeError('Moving projection check did not pass')
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
