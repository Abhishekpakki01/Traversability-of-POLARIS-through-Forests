#!/usr/bin/env python3
"""Measure the RELLIS-3D camera-LiDAR offset from human labels on both sensors.

Each labelled LiDAR point is projected with the published calibration and compared with the human
label at that image pixel, while the projection is shifted sideways and up or down. The shift with
the best agreement estimates the remaining offset. It uses no outlines, so it is an independent
reference for check_outline_alignment_rellis.py.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from semantic_mapping.calibration_check import project


def main():
    root = Path.home()/'thesis_data/rellis_3d_full'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=root/'data/Rellis-3D')
    parser.add_argument('--calibration', type=Path, default=root/'calib')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--step', type=int, default=25, help='use every nth labelled image')
    parser.add_argument('--max-deg', type=float, default=4.5)
    parser.add_argument('--grid-deg', type=float, default=.25)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    angles = np.arange(-args.max_deg, args.max_deg+1e-6, args.grid_deg)
    rows, per_sequence = [], {}
    for folder in sorted(p for p in args.data.iterdir() if p.is_dir()):
        fx, fy, cx, cy = map(float, (args.calibration/'Rellis-3D'/folder.name/'camera_info.txt').read_text().split())
        data = yaml.safe_load((args.calibration/'Rellis_3D'/folder.name/'transforms.yaml').read_text())
        q, t = data['os1_cloud_node-pylon_camera_node']['q'], data['os1_cloud_node-pylon_camera_node']['t']
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat([q['x'], q['y'], q['z'], q['w']]).as_matrix()
        transform[:3, 3] = [t['x'], t['y'], t['z']]
        camera_lidar = np.linalg.inv(transform)
        intrinsic = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        for path in sorted((folder/'pylon_camera_node_label_id').glob('*.png'))[::args.step]:
            index = int(path.name[5:11])
            image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
            raw = np.fromfile(folder/'os1_cloud_node_kitti_bin'/f'{index:06d}.bin', dtype=np.float32).reshape(-1, 4)[:, :3]
            labels = np.fromfile(folder/'os1_cloud_node_semantickitti_label_id'/f'{index:06d}.label', dtype=np.uint32) & 0xFFFF
            pixels, _, kept = project(raw.astype(np.float64), camera_lidar, intrinsic, image.shape[1], image.shape[0], near=2., far=60.)
            scored = labels[kept] != 0
            pixels, point_labels = pixels[scored], labels[kept][scored]
            if len(pixels) < 2000:
                continue
            table = np.zeros((len(angles), len(angles)))  # rows: vertical, columns: horizontal
            for i, vertical in enumerate(angles):
                v = np.round(pixels[:, 1]-fy*np.tan(np.radians(vertical))).astype(int)
                for j, horizontal in enumerate(angles):
                    u = np.round(pixels[:, 0]-fx*np.tan(np.radians(horizontal))).astype(int)
                    inside = (u >= 0) & (u < image.shape[1]) & (v >= 0) & (v < image.shape[0])
                    table[i, j] = (image[v[inside], u[inside]] == point_labels[inside]).mean() if inside.any() else 0.
            row, column = np.unravel_index(table.argmax(), table.shape)
            rows.append(dict(sequence=folder.name, image=path.name, dx_deg=float(angles[column]), dy_deg=float(angles[row]),
                             best_match=float(table.max()), match_at_zero=float(table[len(angles)//2, len(angles)//2])))
            per_sequence.setdefault(folder.name, []).append(table)
        print(folder.name, len(per_sequence.get(folder.name, [])), 'frames', flush=True)
    summary = {}
    for sequence, tables in per_sequence.items():
        mean = np.mean(tables, axis=0)
        row, column = np.unravel_index(mean.argmax(), mean.shape)
        summary[sequence] = dict(frames=len(tables), dx_deg=float(angles[column]), dy_deg=float(angles[row]),
                                 best_match=float(mean.max()), match_at_zero=float(mean[len(angles)//2, len(angles)//2]))
    mean = np.mean([t for tables in per_sequence.values() for t in tables], axis=0)
    row, column = np.unravel_index(mean.argmax(), mean.shape)
    report = dict(frames=len(rows), dx_deg=float(angles[column]), dy_deg=float(angles[row]), best_match=float(mean.max()),
                  match_at_zero=float(mean[len(angles)//2, len(angles)//2]),
                  sign='Positive dx: true image position lies left of the projected LiDAR point, so LiDAR minus camera is +dx',
                  by_sequence=summary, by_frame=rows)
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != 'by_frame'}, indent=2))


if __name__ == '__main__':
    main()
