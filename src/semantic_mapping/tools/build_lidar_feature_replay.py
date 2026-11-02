#!/usr/bin/env python3
"""Accumulate LiDAR-only voxel features scan by scan and add them to a replay.

Re-reads the original sensor bag with exactly the poses, range limit and
vehicle box recorded by an offline run, accumulates point moments, raw
intensity and pass-throughs (lidar_features.VoxelStatistics), and at each
saved map timestamp publishes every map voxel classified from LiDAR alone
(ground-layer voxels as ground):

  /lidar/features  all map voxels; rgb = LiDAR-only class, plus fields
                   permeability, linearity, planarity, scattering, verticality,
                   intensity, evidence, height and the camera label
  /lidar/legend    class colours and shares

Every message of the input replay is copied unchanged. Features at a frame use
only scans up to that frame. Thresholds are a priori, not fitted.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import Header
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_offline_demo import Recording  # noqa: E402
from semantic_mapping.camera_projection import transform_points  # noqa: E402
from semantic_mapping.lidar_features import (  # noqa: E402
    LIDAR_CLASSES, LIDAR_PALETTE, LidarClassSettings, VoxelStatistics, lidar_classes)
from semantic_mapping.recorded_calibration import recorded_geometry  # noqa: E402
from semantic_mapping.semantic_fusion import CLASS_NAMES  # noqa: E402
from semantic_mapping.terrain_analysis import analyze  # noqa: E402

FIELDS = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'u4'), ('lidar_class', 'u1'), ('camera_label', 'u1'),
          ('permeability', 'f4'), ('linearity', 'f4'), ('planarity', 'f4'), ('scattering', 'f4'),
          ('verticality', 'f4'), ('intensity', 'f4'), ('evidence', 'f4'), ('height_m', 'f4')]
KINDS = {'f4': PointField.FLOAT32, 'u4': PointField.UINT32, 'u1': PointField.UINT8}


def feature_cloud(header, values):
    dtype = np.dtype([(n, '<'+k if k != 'u1' else k) for n, k in FIELDS])
    data = np.zeros(len(values['x']), dtype=dtype)
    for name, _ in FIELDS:
        data[name] = values[name]
    offsets = [dtype.fields[n][1] for n, _ in FIELDS]
    return PointCloud2(header=header, height=1, width=len(data), is_dense=False, point_step=dtype.itemsize,
                       row_step=dtype.itemsize*len(data), data=data.tobytes(),
                       fields=[PointField(name=n, offset=o, datatype=KINDS[k], count=1)
                               for (n, k), o in zip(FIELDS, offsets)])


def legend(frame, total, shares):
    image = np.full((150, 1050, 3), 27, dtype=np.uint8)
    put = lambda text, x, y, scale, color=(235, 235, 235): cv2.putText(
        image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)
    entries = [('ground', 'ground'), ('low_vegetation', 'low veg'), ('tall_vegetation', 'tall veg'),
               ('trunk_like', 'trunk-like'), ('solid_obstacle', 'solid')]
    for i, (name, text) in enumerate(entries):
        x = 14+i*207
        cv2.rectangle(image, (x, 24), (x+30, 54), tuple(int(c) for c in LIDAR_PALETTE[LIDAR_CLASSES.index(name)][::-1]), -1)
        put(f'{text} {100*shares.get(name, 0):.0f}%', x+38, 50, .7)
    put(f'LiDAR ONLY (no camera)  scan {frame}/{total}  |  a-priori thresholds', 14, 118, .8, (25, 178, 250))
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='offline_demo run folder (poses, map, settings)')
    parser.add_argument('--replay', type=Path, required=True, help='bag folder to copy, e.g. a terrain demo replay')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    run = json.loads((args.run/'report.json').read_text())
    frames = [json.loads(line) for line in (args.run/'frames.jsonl').read_text().splitlines()]
    by_stamp = {f['lidar_stamp_ns']: f for f in frames}
    cell, max_range = run['resolution_m'], run['maximum_range_m']
    box = None if run.get('vehicle_box_m') is None else np.array(run['vehicle_box_m'])
    _, _, vehicle_lidar = recorded_geometry(yaml.safe_load((args.run/'calibration.yaml').read_text()))
    recording = Recording(Path(run['bag']))

    args.output.mkdir(parents=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.output/'replay'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.replay), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    source_topics = reader.get_all_topics_and_types()
    if any(t.name.startswith('/lidar/') for t in source_topics):
        parser.error('Input replay already contains LiDAR feature topics')
    for topic in source_topics:
        writer.create_topic(topic)
    for name, kind in [('/lidar/features', 'sensor_msgs/msg/PointCloud2'), ('/lidar/legend', 'sensor_msgs/msg/Image')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format='cdr'))

    stats, bridge, settings = VoxelStatistics(cell), CvBridge(), LidarClassSettings()
    done, summaries, final = 0, [], None
    for cloud in recording.messages('/velodyne_1/velodyne_points', PointCloud2):
        stamp = cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec
        frame = by_stamp.get(stamp)
        if frame is None:  # scans the run excluded (no pose) stay excluded
            continue
        raw = read_points(cloud, field_names=('x', 'y', 'z', 'intensity'))
        points = np.column_stack([raw['x'], raw['y'], raw['z']]).astype(np.float64)
        intensity = raw['intensity'].astype(np.float64)
        if box is not None:
            inside = ((transform_points(points, vehicle_lidar) >= box[0]) &
                      (transform_points(points, vehicle_lidar) <= box[1])).all(axis=1)
            points, intensity = points[~inside], intensity[~inside]
        ranges = np.linalg.norm(points, axis=1)
        valid = np.isfinite(points).all(axis=1) & (ranges > .1) & (ranges <= max_range)
        with np.load(args.run/'frames'/frame['frame']/'voxels.npz') as saved:
            world_lidar = saved['world_vehicle'] @ vehicle_lidar
            centres, labels = saved['centres'].astype(np.float64), saved['labels']
        world = transform_points(points[valid], world_lidar)
        stats.add_points(world, intensity[valid])
        stats.add_rays(world_lidar[:3, 3], world)

        features = stats.features(centres)
        height = analyze(centres, labels).voxel_height_m
        classes = lidar_classes(features, height, cell, settings)
        shown = np.ones(len(height), dtype=bool)  # ground too: a self-contained LiDAR-only picture
        rgb = LIDAR_PALETTE[classes[shown]].astype(np.uint32)
        values = dict(x=centres[shown, 0], y=centres[shown, 1], z=centres[shown, 2],
                      rgb=(rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2], lidar_class=classes[shown],
                      camera_label=labels[shown], permeability=features['permeability'][shown],
                      linearity=features['linearity_neighbourhood'][shown],
                      planarity=features['planarity_neighbourhood'][shown],
                      scattering=features['scattering_neighbourhood'][shown],
                      verticality=features['principal_verticality_neighbourhood'][shown],
                      intensity=features['intensity'][shown],
                      evidence=(features['points']+features['passes'])[shown], height_m=height[shown])
        header = Header(frame_id='sensor_init')
        header.stamp.sec, header.stamp.nanosec = divmod(stamp, 10**9)
        counts = Counter(LIDAR_CLASSES[c] for c in classes[shown])
        shares = {k: v/max(int(shown.sum()), 1) for k, v in counts.items()}
        writer.write('/lidar/features', serialize_message(feature_cloud(header, values)), stamp)
        image = bridge.cv2_to_imgmsg(legend(frame['frame'], len(frames), shares), 'bgr8')
        image.header = header
        writer.write('/lidar/legend', serialize_message(image), stamp)
        summaries.append(dict(frame=frame['frame'], lidar_stamp_ns=stamp, published_voxels=int(shown.sum()),
                              classes=dict(counts)))
        final = (centres, labels, classes, height, features)
        done += 1
        if done % 50 == 0:
            print(f'{done}/{len(frames)} frames, {len(stats.codes)} feature voxels', flush=True)
    recording.db.close()
    if done != len(frames):
        raise ValueError(f'Processed {done} of {len(frames)} saved frames')
    copied = 0
    while reader.has_next():
        writer.write(*reader.read_next())
        copied += 1
    del writer

    centres, labels, classes, height, features = final
    above = ~(height <= .5*cell)
    cross = {}
    for i, name in enumerate(CLASS_NAMES):
        mask = above & (labels == i)
        if mask.any():
            c = np.bincount(classes[mask], minlength=len(LIDAR_CLASSES))
            cross[name] = dict(voxels=int(mask.sum()),
                               lidar_class_fraction={k: float(c[j]/mask.sum()) for j, k in enumerate(LIDAR_CLASSES)},
                               median_permeability=float(np.nanmedian(features['permeability'][mask])),
                               median_linearity=float(np.nanmedian(features['linearity_neighbourhood'][mask])),
                               median_scattering=float(np.nanmedian(features['scattering_neighbourhood'][mask])))
    np.savez_compressed(args.output/'final_features.npz', centres=centres, camera_label=labels,
                        lidar_class=classes, height_m=height, **{k: v for k, v in features.items()})
    report = dict(run=str(args.run.resolve()), source_bag=run['bag'], copied_messages=copied,
                  frames=done, vehicle_box_m=run.get('vehicle_box_m'), maximum_range_m=max_range,
                  class_settings=vars(settings), lidar_classes=LIDAR_CLASSES,
                  final_above_ground_voxels=int(above.sum()),
                  final_class_counts={k: int(np.sum(classes[above] == i)) for i, k in enumerate(LIDAR_CLASSES)},
                  final_permeability_evidence_voxels=int(np.sum((features['points']+features['passes'] >= 5) & above)),
                  camera_label_vs_lidar_class_above_ground=cross,
                  note='LiDAR-only classes use a-priori thresholds; no labelled Tartan data exists, so the '
                       'camera comparison measures agreement between two unvalidated sources, not accuracy.')
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    with open(args.output/'frames.jsonl', 'w') as f:
        f.writelines(json.dumps(s)+'\n' for s in summaries)
    print(json.dumps({k: report[k] for k in ('frames', 'copied_messages', 'final_above_ground_voxels',
                                             'final_class_counts')}, indent=2))


if __name__ == '__main__':
    main()
