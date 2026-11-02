#!/usr/bin/env python3
"""Measure LiDAR returns that stay fixed in the vehicle frame while it drives.

Terrain moves through the vehicle frame during motion; the vehicle's own body
does not. Cells occupied in most sampled scans therefore indicate self-returns.
Prints and saves a proposed exclusion box (persistent cells plus a margin) for
build_offline_demo.py --exclude-vehicle-box. Reads the bag without changing it.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from sensor_msgs.msg import PointCloud2
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_offline_demo import Recording  # noqa: E402
from semantic_mapping.camera_projection import transform_points, xyz_from_cloud  # noqa: E402
from semantic_mapping.recorded_calibration import recorded_geometry  # noqa: E402


def persistence(clouds, vehicle_lidar, cell, max_range):
    counts, scans = {}, 0
    for points in clouds:
        ranges = np.linalg.norm(points, axis=1)
        points = points[np.isfinite(points).all(axis=1) & (ranges > .1) & (ranges <= max_range)]
        for key in map(tuple, np.unique(np.floor(transform_points(points, vehicle_lidar)/cell).astype(int), axis=0)):
            counts[key] = counts.get(key, 0)+1
        scans += 1
    keys = np.array(list(counts), dtype=int).reshape(-1, 3)
    return keys, np.array(list(counts.values()), dtype=float)/max(scans, 1), scans


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=Path.home()/'thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2')
    parser.add_argument('--calibration', type=Path, default=package/'config/tartan_recorded_calibration.yaml')
    parser.add_argument('--output', type=Path, required=True, help='new JSON file')
    parser.add_argument('--every', type=int, default=4, help='sample every Nth scan')
    parser.add_argument('--cell', type=float, default=.2)
    parser.add_argument('--persistent-fraction', type=float, default=.8)
    parser.add_argument('--margin', type=float, default=.2)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists')
    _, _, vehicle_lidar = recorded_geometry(yaml.safe_load(args.calibration.read_text()))
    recording = Recording(args.bag)
    clouds = (xyz_from_cloud(c) for i, c in enumerate(recording.messages('/velodyne_1/velodyne_points', PointCloud2))
              if i % args.every == 0)
    keys, fraction, scans = persistence(clouds, vehicle_lidar, args.cell, 10.)
    persistent = keys[fraction > args.persistent_fraction]*args.cell
    if not len(persistent):
        raise SystemExit('No persistent cells; no exclusion box proposed')
    low = persistent.min(0)-args.margin
    high = persistent.max(0)+args.cell+args.margin
    result = dict(bag=str(args.bag.resolve()), sampled_scans=scans, cell_m=args.cell,
                  persistent_fraction=args.persistent_fraction, persistent_cells=int(len(persistent)),
                  persistent_bounds_m=[persistent.min(0).tolist(), (persistent.max(0)+args.cell).tolist()],
                  margin_m=args.margin, proposed_box_m=[np.round(low, 2).tolist(), np.round(high, 2).tolist()],
                  frame='vehicle',
                  caution='Data-derived for this recording and sensor mount; terrain inside the box is also removed. '
                          'Inspect before reuse; POLARIS needs its own measurement.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    print('--exclude-vehicle-box', *np.round(low, 2), *np.round(high, 2))


if __name__ == '__main__':
    main()
