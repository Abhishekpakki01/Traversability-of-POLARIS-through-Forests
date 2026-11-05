#!/usr/bin/env python3
"""Control test: run the outline alignment check on RELLIS-3D, which has a published calibration.

The same outline code runs on another camera and LiDAR. Acceptance rules are the Tartan rules
converted to degrees, because the RELLIS lens is much narrower. A result near zero supports the
method. A large offset here would mean the method is biased, or that this calibration is off.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from semantic_mapping.calibration_check import lidar_view, project
from semantic_mapping.camera_projection import transform_points
from semantic_mapping.silhouette_alignment import best_shift, camera_outline, lidar_outline, solid_columns

WIDTH, TARTAN_FOCAL = 1024, 477.6
MAX_SPREAD_DEG, MAX_VERTICAL_DEG, SEARCH_DEG = np.degrees(np.arctan(7/TARTAN_FOCAL)), np.degrees(np.arctan(30/TARTAN_FOCAL)), 4.5


def calibration(root, sequence, scale):
    fx, fy, cx, cy = map(float, (root/'Rellis-3D'/sequence/'camera_info.txt').read_text().split())
    data = yaml.safe_load((root/'Rellis_3D'/sequence/'transforms.yaml').read_text())['os1_cloud_node-pylon_camera_node']
    q, t = data['q'], data['t']
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat([q['x'], q['y'], q['z'], q['w']]).as_matrix()
    transform[:3, 3] = [t['x'], t['y'], t['z']]
    return np.array([[fx*scale, 0, cx*scale], [0, fy*scale, cy*scale], [0, 0, 1]]), transform


def main():
    root = Path.home()/'thesis_data/rellis_3d_full'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=root/'data/Rellis-3D')
    parser.add_argument('--calibration', type=Path, default=root/'calib')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--step', type=int, default=60, help='use every nth image')
    parser.add_argument('--window', type=int, default=6, help='scans either side drawn into the LiDAR view')
    parser.add_argument('--sky-level', type=int, default=150, help='brightness above which a pixel can be sky')
    parser.add_argument('--show', type=int, default=8, help='accepted frames in the figure')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    rows, panels = [], []
    for folder in sorted(p for p in args.data.iterdir() if p.is_dir()):
        images = sorted((folder/'pylon_camera_node').glob('*.jpg'))
        poses = np.loadtxt(folder/'poses.txt').reshape(-1, 3, 4)
        for path in images[::args.step]:
            index = int(path.name[5:11])
            if not args.window <= index < len(poses)-args.window:
                continue
            original = cv2.imread(str(path))
            scale = WIDTH/original.shape[1]
            camera = cv2.resize(original, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            height = camera.shape[0]
            intrinsic, transform = calibration(args.calibration, folder.name, scale)
            camera_lidar = np.linalg.inv(transform)  # human labels match 75% this way, 28% the other way
            world_lidar = np.eye(4)
            world_lidar[:3] = poses[index]
            camera_world = camera_lidar @ np.linalg.inv(world_lidar)
            points = []
            for scan in range(index-args.window, index+args.window+1):
                raw = np.fromfile(folder/'os1_cloud_node_kitti_bin'/f'{scan:06d}.bin', dtype=np.float32).reshape(-1, 4)[:, :3]
                raw = raw[np.isfinite(raw).all(1) & (np.linalg.norm(raw, axis=1) > 1.5)].astype(np.float64)
                pose = np.eye(4)
                pose[:3] = poses[scan]
                points.append(transform_points(raw, pose))
            pixels, depths, _ = project(np.vstack(points), camera_world, intrinsic, WIDTH, height, far=120.)
            depth = lidar_view(pixels, depths, WIDTH, height)[1]
            focal = float(intrinsic[0, 0])
            camera_rows, lidar_rows = camera_outline(camera, args.sky_level), lidar_outline(depth)
            usable = solid_columns(camera, camera_rows, limit=int(.8*height)) & np.isfinite(lidar_rows) & (np.nan_to_num(lidar_rows) > 6)
            search = int(round(focal*np.tan(np.radians(SEARCH_DEG))))
            found = best_shift(camera_rows, lidar_rows, search=search, min_columns=120, usable=usable)
            entry = dict(sequence=folder.name, image=path.name, points_in_view=int(len(depths)), usable_columns=int(usable.sum()))
            if found:
                dx, dy, spread, columns = found
                deg = lambda v: float(np.degrees(np.arctan(v/focal)))
                entry.update(dx_px=dx, dy_px=dy, spread_px=spread, columns=columns, dx_deg=deg(dx), dy_deg=deg(dy), spread_deg=deg(spread))
                entry['accepted'] = bool(deg(spread) <= MAX_SPREAD_DEG and abs(deg(dy)) <= MAX_VERTICAL_DEG and abs(dx) < search)
                if entry['accepted'] and len(panels) < args.show:
                    picture = camera.copy()
                    for x, row in enumerate(lidar_rows):
                        if np.isfinite(row):
                            cv2.circle(picture, (x, int(row)), 1, (0, 255, 0), -1, cv2.LINE_AA)
                    cv2.putText(picture, f'{folder.name} {path.name[:11]} dx {entry["dx_deg"]:+.2f} deg', (8, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, .75, (0, 0, 255), 2, cv2.LINE_AA)
                    panels.append(picture)
            rows.append(entry)
            print(entry['sequence'], path.name[:11], entry['points_in_view'], entry.get('dx_deg'), entry.get('dy_deg'), entry.get('accepted'), flush=True)
    kept = [r for r in rows if r.get('accepted')]
    dx, dy = np.array([r['dx_deg'] for r in kept]), np.array([r['dy_deg'] for r in kept])
    report = dict(frames_tried=len(rows), frames_with_outline=sum('dx_px' in r for r in rows), frames_accepted=len(kept),
                  median_dx_deg=float(np.median(dx)) if len(dx) else None, median_dy_deg=float(np.median(dy)) if len(dy) else None,
                  dx_deg_p10_p90=np.percentile(dx, [10, 90]).tolist() if len(dx) else None,
                  by_sequence={s: float(np.median([r['dx_deg'] for r in kept if r['sequence'] == s]))
                               for s in sorted({r['sequence'] for r in kept})},
                  rules=dict(max_spread_deg=MAX_SPREAD_DEG, max_vertical_deg=MAX_VERTICAL_DEG, search_deg=SEARCH_DEG),
                  frames=rows)
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2))
    if panels:
        while len(panels) % 2:
            panels.append(panels[0]*0)
        cv2.imwrite(str(args.output/'accepted_frames.jpg'), np.vstack([np.hstack(panels[i:i+2]) for i in range(0, len(panels), 2)]),
                    [cv2.IMWRITE_JPEG_QUALITY, 88])
    print(json.dumps({k: v for k, v in report.items() if k != 'frames'}, indent=2))


if __name__ == '__main__':
    main()
