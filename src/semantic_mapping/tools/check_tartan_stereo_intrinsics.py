#!/usr/bin/env python3
"""Check candidate image models against recorded stereo colors at one exposure.

The stereo cloud was generated from camera data. This is an independent source
pipeline consistency check, not a measurement of physical LiDAR calibration.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    previous = yaml.safe_load((package/'config/tartan_meadows_camera.yaml').read_text())
    recorded = yaml.safe_load((package/'config/tartan_recorded_calibration.yaml').read_text())
    cloud = np.load(args.reference/'deep_cloud.npz')
    metadata = json.loads((args.reference/'report.json').read_text())
    source = next(item for item in metadata['messages'] if item['topic'] == '/deep_cloud')
    assert source['frame'] == 'multisense' and int(cloud['point_step']) == 16
    points = np.frombuffer(cloud['data'], dtype=np.dtype([
        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')]))
    # The original stereo node stores forward/right/down in the multisense frame.
    optical = np.column_stack([points[k] for k in ('y', 'z', 'x')]).astype(float)
    bgr = np.column_stack([points['rgb'] & 255, (points['rgb'] >> 8) & 255,
                          (points['rgb'] >> 16) & 255]).astype(float)
    image = cv2.imread(str(args.reference/'deep_cloud_matching_image.png'))
    if image is None or image.shape[:2] != (544, 1024):
        raise ValueError('Missing matching 1024x544 camera exposure')
    intrinsic = np.array(recorded['camera_P']).reshape(3, 4)[:, :3]
    models = [('rectified_P', intrinsic, np.eye(3)),
              ('raw_K', np.array(previous['camera_K_raw']).reshape(3, 3), np.eye(3)),
              ('P_with_later_R', intrinsic, np.array(previous['camera_R']).reshape(3, 3))]
    results = []
    for name, matrix, rotation in models:
        camera = optical @ rotation.T
        uv = camera[:, :2]/camera[:, 2:]
        uv = uv*[matrix[0, 0], matrix[1, 1]]+matrix[:2, 2]
        valid = (np.isfinite(uv).all(axis=1) & (camera[:, 2] > 0)
                 & (uv[:, 0] >= 0) & (uv[:, 0] < image.shape[1]-1)
                 & (uv[:, 1] >= 0) & (uv[:, 1] < image.shape[0]-1))
        projected = uv[valid]
        samples = []
        for start in range(0, len(projected), 20000):
            chunk = projected[start:start+20000].astype(np.float32)
            samples.append(cv2.remap(image, chunk[:, 0][None], chunk[:, 1][None],
                                      interpolation=cv2.INTER_LINEAR)[0])
        error = np.abs(np.concatenate(samples).astype(float)-bgr[valid]).mean(axis=1)
        results.append({'model': name, 'points': int(valid.sum()),
            'median_color_error_0_255': float(np.median(error)),
            'mean_color_error_0_255': float(np.mean(error))})
    report = {'image_and_stereo_cloud_stamp_ns': int(cloud['stamp_ns']),
        'physical_lidar_calibration_validated': False,
        'description': 'Recorded stereo colors versus matching camera pixels; '
                       'candidate-model comparison, not ground-truth reprojection error.',
        'results': results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
