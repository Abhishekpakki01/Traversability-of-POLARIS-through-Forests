#!/usr/bin/env python3
"""Audit saved attributes, timestamps and source accounting without rerunning inference."""
import argparse
import json
from pathlib import Path
import sqlite3

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points

from semantic_mapping.offline_demo import stamp_ns
from semantic_mapping.semantic_fusion import PALETTE, voxel_keys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    args = parser.parse_args()
    report = json.loads((args.input/'report.json').read_text())
    frames = [json.loads(line) for line in (args.input/'frames.jsonl').read_text().splitlines()]
    assert report['passed_functional_checks'] and report['input_accounting_ok']
    assert len(frames) == report['mapped_scans'] == report['saved_frames']
    assert report['scans_seen'] == report['mapped_scans']+report['missing_lidar_pose']+report['empty_cloud']
    assert all(a['lidar_stamp_ns'] < b['lidar_stamp_ns'] for a, b in zip(frames, frames[1:]))
    image_stamps = [f['image_stamp_ns'] for f in frames if f['image_stamp_ns'] is not None]
    assert len(set(image_stamps)) == len(image_stamps) == report['inferred_images']
    assert all(a < b for a, b in zip(image_stamps, image_stamps[1:]))
    assert all(abs(f['image_stamp_ns']-f['lidar_stamp_ns']) <= 50_000_000
               for f in frames if f['image_stamp_ns'] is not None)
    source = list((args.input/'replay').glob('*.db3'))[0]
    db = sqlite3.connect(source.resolve().as_uri()+'?mode=ro', uri=True)
    topics = dict(db.execute('SELECT name,id FROM topics'))
    rows = db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',
                      (topics['/semantic_voxel_map'],))
    elevation_rows = db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',
                               (topics['/offline_demo/elevation'],))
    first_z, last_xyz = None, None
    for frame in frames:
        row = next(rows, None)
        assert row is not None and row[0] == frame['lidar_stamp_ns']
        cloud = deserialize_message(row[1], PointCloud2)
        assert stamp_ns(cloud) == row[0] and cloud.header.frame_id == 'sensor_init'
        points = read_points(cloud)
        elevation_row = next(elevation_rows, None)
        assert elevation_row is not None and elevation_row[0] == row[0]
        elevation_cloud = deserialize_message(elevation_row[1], PointCloud2)
        elevation = read_points(elevation_cloud)
        for key in ['x', 'y', 'z', 'label', 'confidence', 'observations', 'relative_z']:
            np.testing.assert_array_equal(elevation[key], points[key])
        import cv2
        low, high = report['elevation_range_m']
        values = np.rint(np.clip((points['relative_z']-low)/(high-low), 0, 1)*255).astype(np.uint8)
        colors = cv2.applyColorMap(values[:, None], cv2.COLORMAP_TURBO)[:, 0, ::-1].astype(np.uint32)
        expected = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
        np.testing.assert_array_equal(elevation['rgb'].view(np.uint32), expected)
        with np.load(args.input/'frames'/frame['frame']/'voxels.npz') as saved:
            xyz = np.column_stack([points[k] for k in ('x', 'y', 'z')])
            np.testing.assert_array_equal(xyz, saved['centres'])
            assert np.isfinite(xyz).all()
            keys = voxel_keys(xyz, .35)
            assert len(np.unique(keys, axis=0)) == len(keys)
            for key in ('confidence', 'observations', 'relative_z'):
                np.testing.assert_allclose(points[key], saved[key], atol=1e-6)
            np.testing.assert_array_equal(points['label'], saved['labels'])
            labels = points['label']; selected = labels != 0
            assert (points['observations'][selected] >= 2).all()
            assert (points['confidence'][selected] >= .6-1e-7).all()
            assert int(selected.sum()) == frame['classified_voxels']
            expected_rgb = PALETTE[labels].astype(np.uint32)
            expected_rgb = (expected_rgb[:, 0] << 16) | (expected_rgb[:, 1] << 8) | expected_rgb[:, 2]
            np.testing.assert_array_equal(points['rgb'].view(np.uint32), expected_rgb)
            if first_z is None:
                first_z = saved['world_vehicle'][2, 3]
                first_xyz = saved['world_vehicle'][:3, 3].copy()
            np.testing.assert_allclose(saved['relative_z'], saved['centres'][:, 2]-first_z, atol=1e-5)
            last_xyz = saved['world_vehicle'][:3, 3].copy()
    assert next(rows, None) is None
    assert next(elevation_rows, None) is None
    # Every saved camera/label/projection exposure must refer to the same image.
    for topic in ['/offline_demo/camera', '/offline_demo/segmentation',
                  '/offline_demo/projection', '/offline_demo/labels']:
        stamps = [stamp_ns(deserialize_message(raw, Image)) for raw, in db.execute(
            'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id', (topics[topic],))]
        assert stamps == image_stamps, 'Saved image topics are not synchronized: '+topic
    db.close()
    last = frames[-1]
    result = dict(passed=True, checked_map_frames=len(frames), checked_image_exposures=len(image_stamps),
        unique_image_observations=True, matched_image_headers=True, bag_matches_saved_voxel_arrays=True,
        classified_voxels_meet_original_thresholds=True, packed_colors_match_class_palette=True,
        elevation_matches_declared_reference=True, accounted_source_scans=report['scans_seen'],
        elevation_colors_match_declared_range=True,
        excluded_missing_pose=report['missing_lidar_pose'],
        endpoint_displacement_m=float(np.linalg.norm(last_xyz-first_xyz)),
        recorded_pose_span_seconds=(frames[-1]['lidar_stamp_ns']-frames[0]['lidar_stamp_ns'])/1e9,
        occupied_voxels=last['occupied_voxels'], classified_voxels=last['classified_voxels'],
        voxels_with_camera_evidence=last['voxels_with_evidence'],
        classified_fraction_all_occupied=last['classified_voxels']/last['occupied_voxels'],
        classified_fraction_with_camera_evidence=last['classified_voxels']/last['voxels_with_evidence'],
        physical_calibration_validated=False, semantic_accuracy_measured=False,
        navigation_validated=False)
    (args.input/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
