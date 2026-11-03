#!/usr/bin/env python3
"""Map sharpness: is flat ground as thin in the accumulated map as in one scan?

If poses, LiDAR mounting or timing were wrong, the same ground seen from many
positions would be smeared into a thick layer. For flat, horizontal surface
patches (planar, vertical normal, enough points) the thickness is the standard
deviation along the surface normal (sqrt of the smallest covariance
eigenvalue). It is compared for:
  - single scans (sensor noise floor),
  - all scans accumulated with the recorded poses,
  - all scans with deliberately perturbed poses (shows what an error looks like).
Uses the run's poses, range limit and vehicle box; reads the bag without changing it.
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
from semantic_mapping.semantic_fusion import voxel_keys  # noqa: E402


class Moments:
    def __init__(self, cell):
        self.cell, self.data = cell, {}

    def add(self, points):
        keys, inverse = np.unique(voxel_keys(points, self.cell), axis=0, return_inverse=True)
        n = np.bincount(inverse, minlength=len(keys)).astype(float)
        total = np.zeros((len(keys), 3)); np.add.at(total, inverse, points)
        outer = np.zeros((len(keys), 3, 3)); np.add.at(outer, inverse, points[:, :, None]*points[:, None, :])
        for key, a, b, c in zip(map(tuple, keys), n, total, outer):
            entry = self.data.get(key)
            if entry is None:
                self.data[key] = [a, b, c]
            else:
                entry[0] += a; entry[1] += b; entry[2] += c

    def thickness(self, min_points=20):
        """Normal standard deviation (m) of flat horizontal patches."""
        n = np.array([v[0] for v in self.data.values()])
        keep = n >= min_points
        if not keep.any():
            return np.empty(0)
        values = [v for v, k in zip(self.data.values(), keep) if k]
        n = n[keep]
        mean = np.array([v[1] for v in values])/n[:, None]
        cov = np.array([v[2] for v in values])/n[:, None, None]-mean[:, :, None]*mean[:, None, :]
        eig, vec = np.linalg.eigh(cov)
        l3, l2, l1 = np.clip(eig, 0, None).T
        flat = (np.abs(vec[:, 2, 0]) > .95) & ((l2-l3)/np.maximum(l1, 1e-12) > .5)
        return np.sqrt(l3[flat])


def perturbation(rng, rotation_deg, translation_m):
    angles = np.radians(rng.normal(scale=rotation_deg, size=3))
    cx, cy, cz = np.cos(angles); sx, sy, sz = np.sin(angles)
    r = (np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
         @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]))
    t = np.eye(4); t[:3, :3] = r; t[:3, 3] = rng.normal(scale=translation_m, size=3)
    return t


def stats(values):
    return dict(patches=int(len(values)), median_m=float(np.median(values)), p90_m=float(np.percentile(values, 90))) \
        if len(values) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cell', type=float, default=.4, help='patch size (m)')
    parser.add_argument('--every', type=int, default=2, help='use every Nth scan')
    parser.add_argument('--perturb-deg', type=float, default=.5)
    parser.add_argument('--perturb-m', type=float, default=.05)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder')
    run = json.loads((args.run/'report.json').read_text())
    frames = {f['lidar_stamp_ns']: f for f in map(json.loads, (args.run/'frames.jsonl').read_text().splitlines())}
    _, _, vehicle_lidar = recorded_geometry(yaml.safe_load((args.run/'calibration.yaml').read_text()))
    box = None if run.get('vehicle_box_m') is None else np.array(run['vehicle_box_m'])
    rng = np.random.default_rng(0)
    clean, perturbed, single = Moments(args.cell), Moments(args.cell), []
    used = 0
    for i, cloud in enumerate(Recording(Path(run['bag'])).messages('/velodyne_1/velodyne_points', PointCloud2)):
        frame = frames.get(cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec)
        if frame is None or i % args.every:
            continue
        points = xyz_from_cloud(cloud)
        if box is not None:
            local = transform_points(points, vehicle_lidar)
            points = points[~((local >= box[0]) & (local <= box[1])).all(1)]
        ranges = np.linalg.norm(points, axis=1)
        points = points[np.isfinite(points).all(1) & (ranges > .1) & (ranges <= run['maximum_range_m'])]
        with np.load(args.run/'frames'/frame['frame']/'voxels.npz') as saved:
            world_lidar = saved['world_vehicle'] @ vehicle_lidar
        clean.add(transform_points(points, world_lidar))
        perturbed.add(transform_points(points, world_lidar @ perturbation(rng, args.perturb_deg, args.perturb_m)))
        if used % 10 == 0:
            one = Moments(args.cell); one.add(transform_points(points, world_lidar)); single.append(one.thickness())
        used += 1
    single = np.concatenate(single) if single else np.empty(0)
    report = dict(run=str(args.run.resolve()), scans_used=used, patch_m=args.cell,
                  single_scan=stats(single), accumulated_recorded_poses=stats(clean.thickness()),
                  accumulated_perturbed_poses=stats(perturbed.thickness()),
                  perturbation=dict(rotation_sigma_deg=args.perturb_deg, translation_sigma_m=args.perturb_m),
                  note='Flat horizontal patches only. Thickness near the single-scan value means the scans '
                       'register consistently; it does not measure absolute position accuracy.')
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
