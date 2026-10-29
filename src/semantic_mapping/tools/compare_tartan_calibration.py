#!/usr/bin/env python3
"""Compare old and recorded mounting transforms on identical acquired frames.

Pixel shift is a difference between models, not a ground-truth reprojection
error. The missing original CameraInfo remains an explicit limitation.
"""

import argparse
import json
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosbags.highlevel import AnyReader
from sensor_msgs.msg import Image, PointCloud2
from tf2_ros import Buffer
import yaml

from semantic_mapping.camera_projection import (
    draw_projection, project_visible_points, transform_matrix, xyz_from_cloud)
from semantic_mapping.recorded_calibration import recorded_geometry
from semantic_mapping.tartan_camera import rectified_geometry


def stamp_ns(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=Path.home()/
        'thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    old = yaml.safe_load((package/'config/tartan_meadows_camera.yaml').read_text())
    recorded = yaml.safe_load((package/'config/tartan_recorded_calibration.yaml').read_text())
    models = {'previous': rectified_geometry(old), 'recorded': recorded_geometry(recorded)}
    args.output.mkdir(parents=True, exist_ok=True)
    poses, frames = [], [None]*5
    targets = None
    with AnyReader([args.bag]) as reader:
        connections = [c for c in reader.connections if c.topic in (
            '/lidar_odometry', '/multisense/left/image_rect_color')]
        for connection, _, raw in reader.messages(connections=connections):
            if connection.topic == '/lidar_odometry':
                poses.append(deserialize_message(raw, Odometry))
                continue
            image = deserialize_message(raw, Image)
            stamp = stamp_ns(image)
            if targets is None:
                targets = stamp+np.array([2, 8, 16, 24, 30], dtype=np.int64)*10**9
            for index, target in enumerate(targets):
                if frames[index] is None or abs(stamp-target) < abs(stamp_ns(frames[index])-target):
                    frames[index] = image
        clouds = [None]*len(frames)
        connections = [c for c in reader.connections if c.topic == '/velodyne_1/velodyne_points']
        for _, _, raw in reader.messages(connections=connections):
            cloud = deserialize_message(raw, PointCloud2)
            stamp = stamp_ns(cloud)
            for index, frame in enumerate(frames):
                if clouds[index] is None or abs(stamp-stamp_ns(frame)) < abs(stamp_ns(clouds[index])-stamp_ns(frame)):
                    clouds[index] = cloud

    buffer = Buffer(cache_time=Duration(seconds=120))
    for pose in poses:
        transform = TransformStamped(header=pose.header, child_frame_id=pose.child_frame_id)
        p, q = pose.pose.pose.position, pose.pose.pose.orientation
        transform.transform.translation.x = p.x
        transform.transform.translation.y = p.y
        transform.transform.translation.z = p.z
        transform.transform.rotation = q
        buffer.set_transform(transform, 'recorded_lidar_odometry')
    bridge = CvBridge()
    report = {'bag': str(args.bag), 'intrinsics_verified': recorded['intrinsics_verified'],
              'ground_truth_reprojection_error_measured': False, 'frames': []}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for index, (image, cloud) in enumerate(zip(frames, clouds)):
        delta_ms = (stamp_ns(image)-stamp_ns(cloud))/1e6
        if abs(delta_ms) > 50:
            raise RuntimeError('No paired LiDAR within 50 ms')
        world_image = transform_matrix(buffer.lookup_transform('sensor_init', 'vehicle',
            Time.from_msg(image.header.stamp)).transform)
        world_cloud = transform_matrix(buffer.lookup_transform('sensor_init', 'vehicle',
            Time.from_msg(cloud.header.stamp)).transform)
        bgr = bridge.imgmsg_to_cv2(image, 'bgr8')
        points = xyz_from_cloud(cloud)
        projections = {}
        for name, (intrinsic, vehicle_camera, vehicle_lidar) in models.items():
            camera_lidar = np.linalg.solve(world_image @ vehicle_camera, world_cloud @ vehicle_lidar)
            projections[name] = project_visible_points(points, bgr, intrinsic, [], camera_lidar,
                                                       max_range=30., visibility_cell=2)
        _, first, second = np.intersect1d(projections['previous'].source_indices,
            projections['recorded'].source_indices, return_indices=True)
        shifts = np.linalg.norm(projections['previous'].pixels[first]-
                               projections['recorded'].pixels[second], axis=1)
        entry = {'image_stamp_ns': stamp_ns(image), 'lidar_stamp_ns': stamp_ns(cloud),
                 'image_minus_lidar_ms': delta_ms,
                 'projected_points': {name: len(p.pixels) for name, p in projections.items()},
                 'common_points': len(shifts),
                 'median_model_pixel_shift': float(np.median(shifts)),
                 'p90_model_pixel_shift': float(np.percentile(shifts, 90))}
        report['frames'].append(entry)
        folder = args.output/f'frame_{index+1:02d}'
        folder.mkdir(exist_ok=True)
        cv2.imwrite(str(folder/'camera.png'), bgr)
        fig, axes = plt.subplots(1, 3, figsize=(18, 4))
        images = [('Camera exposure', bgr)] + [
            (name+' mounting transform', draw_projection(bgr, projection, 30.))
            for name, projection in projections.items()]
        for axis, (title, pixels) in zip(axes, images):
            axis.imshow(pixels[:, :, ::-1])
            axis.set_title(title)
            axis.axis('off')
            if title != 'Camera exposure':
                name = title.split()[0]
                cv2.imwrite(str(folder/(name+'_projection.png')), pixels)
        fig.suptitle('Same exposure and scan, acquisition-time pose compensation; '
                     'intrinsics still need independent verification')
        fig.tight_layout()
        fig.savefig(folder/'comparison.png', dpi=140)
        plt.close(fig)
        print(json.dumps(entry), flush=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
