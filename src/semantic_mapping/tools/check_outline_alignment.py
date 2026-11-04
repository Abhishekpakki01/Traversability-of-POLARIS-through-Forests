#!/usr/bin/env python3
"""Measure the camera-LiDAR offset from scene outlines against the sky, with no clicking.

Input is a folder from prepare_calibration_check.py. A frame counts only if the outlines match
closely, the match is inside the search range, and the vertical offset is plausible. Those rules
reject false matches, such as a far ridge the LiDAR cannot reach. Nothing is changed or fitted.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from semantic_mapping.silhouette_alignment import best_shift, camera_outline, lidar_outline, solid_columns

SEARCH, MAX_SPREAD, MAX_VERTICAL, MIN_COLUMNS = 45, 7., 30., 120


def overlay(camera, lidar_rows, shift, title):
    picture = camera.copy()
    for x, row in enumerate(lidar_rows):
        if np.isfinite(row) and 0 <= x-shift < picture.shape[1]:
            cv2.circle(picture, (int(x-shift), int(row)), 1, (0, 255, 0), -1, cv2.LINE_AA)
    cv2.putText(picture, title, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 255), 2, cv2.LINE_AA)
    return picture[:360]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--show', nargs='+', default=['00196', '00262'], help='frames for the before/after figure')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    meta = json.loads((args.input/'frames.json').read_text())
    rows, kept, outlines = [], [], {}
    for name in [f['frame'] for f in meta['frames']]:
        camera = cv2.imread(str(args.input/name/'camera.jpg'))
        depth = np.load(args.input/name/'depth.npy')
        camera_rows, lidar_rows = camera_outline(camera), lidar_outline(depth)
        usable = solid_columns(camera, camera_rows) & np.isfinite(lidar_rows) & (np.nan_to_num(lidar_rows) > 6)
        outlines[name] = camera, lidar_rows
        found = best_shift(camera_rows, lidar_rows, search=SEARCH, min_columns=MIN_COLUMNS, usable=usable)
        if found is None:
            continue
        dx, dy, spread, columns = found
        entry = dict(frame=name, dx_px=dx, dy_px=dy, spread_px=spread, columns=columns)
        entry['accepted'] = spread <= MAX_SPREAD and abs(dy) <= MAX_VERTICAL and abs(dx) < SEARCH
        rows.append(entry)
        kept += [entry] if entry['accepted'] else []
    args.output.mkdir(parents=True)
    focal = meta['focal_px']
    dx = np.array([k['dx_px'] for k in kept], float)
    dy = np.array([k['dy_px'] for k in kept], float)
    report = dict(frames_tried=len(meta['frames']), frames_with_outline=len(rows), frames_accepted=len(kept),
                  median_dx_px=float(np.median(dx)) if len(dx) else None,
                  median_dy_px=float(np.median(dy)) if len(dy) else None,
                  dx_range_px=[float(dx.min()), float(dx.max())] if len(dx) else None,
                  median_dx_deg=float(np.degrees(np.arctan(np.median(dx)/focal))) if len(dx) else None,
                  sign='LiDAR outline minus camera outline: negative dx means LiDAR is drawn left of the camera',
                  frames=rows)
    (args.output/'report.json').write_text(json.dumps(report, indent=2))
    shift = int(round(report['median_dx_px'] or 0))
    panels = [np.hstack([overlay(*outlines[n], 0, f'{n}: recorded calibration'),
                         overlay(*outlines[n], shift, f'{n}: outline moved {-shift:+d} px')])
              for n in args.show if n in outlines]
    if panels:
        cv2.imwrite(str(args.output/'before_after.jpg'), np.vstack(panels), [cv2.IMWRITE_JPEG_QUALITY, 90])
    print(json.dumps({k: v for k, v in report.items() if k != 'frames'}, indent=2))


if __name__ == '__main__':
    main()
