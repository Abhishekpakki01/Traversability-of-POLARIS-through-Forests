#!/usr/bin/env python3
"""Prepare frames for the clicked camera-LiDAR check (see click_calibration_check.py).

Picks the frames that see the most pole- and trunk-like voxels, then draws each frame's LiDAR view
from the camera's pose at the image time, using raw scans around that moment. The recorded
calibration is used unchanged: nothing is fitted.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_offline_demo import Recording  # noqa: E402
from semantic_mapping.calibration_check import interpolate_pose, lidar_view, project  # noqa: E402
from semantic_mapping.camera_projection import transform_points  # noqa: E402
from semantic_mapping.recorded_calibration import recorded_geometry  # noqa: E402


def main():
    repo = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=repo/'artifacts/offline_v2_2026_11_03')
    parser.add_argument('--features', type=Path, default=repo/'artifacts/lidar_features_demo_v2_2026_11_03/final_features.npz')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=6)
    parser.add_argument('--pick', nargs='+', help='use these frames (e.g. 00283) instead of choosing automatically')
    parser.add_argument('--spacing', type=int, default=30, help='minimum scans between chosen frames')
    parser.add_argument('--window', type=int, default=12, help='scans either side drawn into the LiDAR view')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    run = json.loads((args.run/'report.json').read_text())
    frames = [json.loads(line) for line in (args.run/'frames.jsonl').read_text().splitlines()]
    intrinsic, vehicle_camera, vehicle_lidar = recorded_geometry(yaml.safe_load((args.run/'calibration.yaml').read_text()))
    poses = []
    for frame in frames:
        with np.load(args.run/'frames'/frame['frame']/'voxels.npz') as saved:
            poses.append(saved['world_vehicle'])
    stamps = np.array([f['lidar_stamp_ns'] for f in frames], dtype=np.float64)
    width, height = 1024, 544

    def camera_from_world(index):
        stamp = frames[index].get('image_stamp_ns') or frames[index]['lidar_stamp_ns']
        return np.linalg.inv(interpolate_pose(stamps, poses, stamp) @ vehicle_camera)

    with np.load(args.features) as saved:
        pole = ((saved['principal_verticality_neighbourhood'] > .8) & (saved['linearity_neighbourhood'] > .5) &
                (saved['height_m'] > .8))
        poles = saved['centres'][pole].astype(np.float64)
    scores = []
    for index in range(args.window, len(frames)-args.window, 5):
        _, depths, _ = project(poles, camera_from_world(index), intrinsic, width, height, near=4., far=25.)
        scores.append((len(depths), index))
    chosen = []
    for score, index in sorted(scores, reverse=True):
        if score and all(abs(index-c) >= args.spacing for c in chosen):
            chosen.append(index)
        if len(chosen) == args.frames:
            break
    if args.pick:
        chosen = [i for i, f in enumerate(frames) if f['frame'] in args.pick]
    chosen.sort()
    print('pole-like voxels', len(poles), 'chosen frames', [frames[c]['frame'] for c in chosen])

    needed = {i for c in chosen for i in range(c-args.window, c+args.window+1)}
    index_of = {f['lidar_stamp_ns']: i for i, f in enumerate(frames)}
    box = None if run.get('vehicle_box_m') is None else np.array(run['vehicle_box_m'])
    scans = {}
    for cloud in Recording(Path(run['bag'])).messages('/velodyne_1/velodyne_points', PointCloud2):
        index = index_of.get(cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec)
        if index not in needed:
            continue
        raw = read_points(cloud, field_names=('x', 'y', 'z'))
        points = np.column_stack([raw['x'], raw['y'], raw['z']]).astype(np.float64)
        points = points[np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > .1)]
        vehicle = transform_points(points, vehicle_lidar)
        if box is not None:
            vehicle = vehicle[~((vehicle >= box[0]) & (vehicle <= box[1])).all(axis=1)]
        scans[index] = transform_points(vehicle, poses[index])

    args.output.mkdir(parents=True)
    entries = []
    for index in chosen:
        name = frames[index]['frame']
        points = np.vstack([scans[i] for i in range(index-args.window, index+args.window+1) if i in scans])
        pixels, depths, _ = project(points, camera_from_world(index), intrinsic, width, height)
        colour, depth = lidar_view(pixels, depths, width, height)
        folder = args.output/name
        folder.mkdir()
        shutil.copy(args.run/'frames'/name/'camera.jpg', folder/'camera.jpg')
        cv2.imwrite(str(folder/'lidar.png'), colour)
        np.save(folder/'depth.npy', depth)
        entries.append(dict(frame=name, points=len(depths)))
    (args.output/'frames.json').write_text(json.dumps(dict(
        run=str(args.run), focal_px=float(intrinsic[0, 0]), window_scans=args.window, frames=entries), indent=2))
    print('saved', args.output)


if __name__ == '__main__':
    main()
