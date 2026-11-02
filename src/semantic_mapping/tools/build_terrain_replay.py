#!/usr/bin/env python3
"""Add a scan-by-scan traversability layer to a saved offline replay.

Copies every message of <input>/replay unchanged into a new bag and, at each
saved map's timestamp, adds terrain clouds computed from that same map:

  /terrain/traversability            ground cells coloured by geometry + semantics
  /terrain/geometric_traversability  ground cells coloured by geometry only
  /terrain/above_ground              occupied voxels above the ground layer, class colours
  /terrain/status_image              legend, counts and the placeholder-limits note

Vehicle limits come from a YAML file whose default values are placeholders,
not POLARIS specifications. The input run is never modified.
"""
import argparse
import json
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import yaml

from semantic_mapping.offline_demo import cloud_message
from semantic_mapping.terrain_analysis import STATES, TerrainSettings, analyze, carpet

# RGB per state; unobserved columns are never drawn.
STATE_RGB = np.array([[0, 0, 0], [137, 135, 129], [12, 163, 12], [250, 178, 25], [208, 59, 59]], dtype=np.uint32)
STATE_TEXT = ['unobserved', 'no ground ref.', 'traversable', 'caution', 'non-traversable']
TOPICS = {'/terrain/traversability': 'sensor_msgs/msg/PointCloud2',
          '/terrain/geometric_traversability': 'sensor_msgs/msg/PointCloud2',
          '/terrain/above_ground': 'sensor_msgs/msg/PointCloud2',
          '/terrain/status_image': 'sensor_msgs/msg/Image'}


def terrain_cloud(header, xyz, state, cost, slope):
    names = ['x', 'y', 'z', 'rgb', 'state', 'cost', 'slope_deg']
    offsets = [0, 4, 8, 12, 16, 20, 24]
    dtype = np.dtype(dict(names=names, formats=['<f4']*3+['<u4', 'u1', '<f4', '<f4'],
                          offsets=offsets, itemsize=28))
    data = np.zeros(len(xyz), dtype=dtype)
    for i, name in enumerate('xyz'):
        data[name] = xyz[:, i]
    colors = STATE_RGB[state]
    data['rgb'] = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
    data['state'], data['cost'], data['slope_deg'] = state, cost, np.nan_to_num(slope, nan=-1.)
    kinds = [PointField.FLOAT32]*4+[PointField.UINT8, PointField.FLOAT32, PointField.FLOAT32]
    return PointCloud2(header=header, height=1, width=len(xyz), is_dense=True, point_step=28,
                       row_step=28*len(xyz), data=data.tobytes(),
                       fields=[PointField(name=n, offset=o, datatype=k, count=1)
                               for n, o, k in zip(names, offsets, kinds)])


def status_image(frame, total, elapsed, shares, verified):
    """Two large lines: the dock in the RViz layout is short and wide."""
    image = np.full((150, 1050, 3), 27, dtype=np.uint8)
    put = lambda text, x, y, scale, color=(235, 235, 235), weight=2: cv2.putText(
        image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, weight, cv2.LINE_AA)
    for x, state in zip((14, 350, 640), (2, 3, 4)):
        cv2.rectangle(image, (x, 22), (x+34, 56), tuple(int(c) for c in STATE_RGB[state][::-1]), -1)
        put(f'{STATE_TEXT[state]} {100*shares[STATES[state]]:.0f}%', x+44, 52, 1.0)
    note = 'PLACEHOLDER limits, not POLARIS specs' if not verified else 'Vehicle limits from config'
    put(f'scan {frame}/{total}  t={elapsed:4.1f}s  |  {note}', 14, 118, .9, (25, 178, 250) if not verified else (235, 235, 235))
    return image


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True, help='offline_demo run folder')
    parser.add_argument('--output', type=Path, required=True, help='new folder')
    parser.add_argument('--config', type=Path, default=package/'config/traversability_placeholder.yaml')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    config = yaml.safe_load(args.config.read_text())
    settings = TerrainSettings.from_config(config)
    verified = bool(config.get('vehicle_parameters_verified', False))
    run = json.loads((args.input/'report.json').read_text())
    if abs(settings.cell_m-run['resolution_m']) > 1e-9:
        parser.error('Terrain cell size must equal the mapping resolution')
    frames = {f['lidar_stamp_ns']: f for f in map(json.loads, (args.input/'frames.jsonl').read_text().splitlines())}

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.input/'replay'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    source_topics = reader.get_all_topics_and_types()
    if any(t.name in TOPICS for t in source_topics):
        parser.error('Input replay already contains terrain topics')
    args.output.mkdir(parents=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.output/'replay'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    for topic in source_topics:
        writer.create_topic(topic)
    for name, kind in TOPICS.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format='cdr'))

    bridge, copied, added, summaries = CvBridge(), 0, 0, []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        writer.write(topic, data, stamp)
        copied += 1
        if topic != '/semantic_voxel_map':
            continue
        frame = frames.get(stamp)
        if frame is None:
            raise ValueError(f'Map at {stamp} has no saved frame')
        with np.load(args.input/'frames'/frame['frame']/'voxels.npz') as saved:
            centres, labels = saved['centres'], saved['labels']
            confidence, observations, relative_z = saved['confidence'], saved['observations'], saved['relative_z']
        terrain = analyze(centres, labels, settings)
        header = Header(frame_id='sensor_init')
        header.stamp.sec, header.stamp.nanosec = divmod(stamp, 10**9)
        xyz, state, cost, slope = carpet(terrain)
        shares = {name: float(np.mean(state == i)) if len(state) else 0. for i, name in enumerate(STATES)}
        shares['cells'] = int(len(state))
        geometric = carpet(terrain, combined=False)
        above = ~(terrain.voxel_height_m <= .5*settings.cell_m)  # includes columns without ground reference
        messages = [
            ('/terrain/traversability', terrain_cloud(header, xyz, state, cost, slope)),
            ('/terrain/geometric_traversability', terrain_cloud(header, *geometric)),
            ('/terrain/above_ground', cloud_message(header, centres[above], labels[above],
                confidence[above], observations[above], relative_z[above]))]
        status = bridge.cv2_to_imgmsg(status_image(frame['frame'], len(frames), frame['elapsed_recording_seconds'],
                                                   shares, verified), 'bgr8')
        status.header = header
        messages.append(('/terrain/status_image', status))
        for name, message in messages:
            writer.write(name, serialize_message(message), stamp)
            added += 1
        summaries.append(dict(frame=frame['frame'], lidar_stamp_ns=stamp, ground_cells=shares['cells'],
                              above_ground_voxels=int(above.sum()),
                              **{name: shares[name] for name in STATES[1:]}))
    del writer  # finalize bag metadata
    if len(summaries) != len(frames):
        raise ValueError('Not every saved frame received a terrain layer')
    report = dict(source_run=str(args.input.resolve()), config=str(args.config.resolve()),
                  vehicle_parameters_verified=verified, settings=vars(settings),
                  copied_messages=copied, added_messages=added, frames=len(summaries),
                  final=summaries[-1], navigation_validated=False)
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    with open(args.output/'frames.jsonl', 'w') as f:
        f.writelines(json.dumps(s)+'\n' for s in summaries)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
