#!/usr/bin/env python3
"""Project saved camera classes onto the mapped ground and add a water layer.

For every saved frame, sampled pixels of the saved class image (the same
0.6-confidence labels as the offline run) are cast from the camera pose at the
image timestamp onto that frame's ground surface (terrain_analysis, small LiDAR
holes filled from surrounding height). Occluded rays are discarded. Evidence
accumulates over frames up to the current one:

  /water/cells          ground columns whose camera pixels are mostly water;
                        bright blue = no LiDAR return there, light blue = LiDAR return
  /water/camera_ground  majority camera class per ground column (optional view)
  /water/legend         colour key and counts

Every message of the input replay is copied unchanged. The input run is not modified.
"""
import argparse
import json
from pathlib import Path
import sqlite3
import sys

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
import rosbag2_py
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_offline_demo import Recording  # noqa: E402
from semantic_mapping.camera_projection import transform_matrix  # noqa: E402
from semantic_mapping.lidar_features import LIDAR_CLASSES, LIDAR_PALETTE, encode  # noqa: E402
from semantic_mapping.recorded_calibration import recorded_geometry  # noqa: E402
from semantic_mapping.semantic_fusion import PALETTE, voxel_keys  # noqa: E402
from semantic_mapping.terrain_analysis import analyze  # noqa: E402
from semantic_mapping.water_detection import (  # noqa: E402
    WATER_RGB, GroundEvidence, camera_directions, fill_ground, ground_hits, pixel_grid)


def cloud(header, xyz, rgb, extra):
    names = ['x', 'y', 'z', 'rgb']+[n for n, _ in extra]
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')]+[(n, '<f4') for n, _ in extra])
    data = np.zeros(len(xyz), dtype=dtype)
    data['x'], data['y'], data['z'], data['rgb'] = xyz[:, 0], xyz[:, 1], xyz[:, 2], rgb
    for name, values in extra:
        data[name] = values
    fields = [PointField(name=n, offset=dtype.fields[n][1],
                         datatype=PointField.UINT32 if n == 'rgb' else PointField.FLOAT32, count=1) for n in names]
    return PointCloud2(header=header, height=1, width=len(data), is_dense=True, point_step=dtype.itemsize,
                       row_step=dtype.itemsize*len(data), data=data.tobytes(), fields=fields)


def packed(rgb):
    rgb = np.asarray(rgb, dtype=np.uint32).reshape(-1, 3)
    return (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]


def legend(frame, total, no_return, with_return, cell):
    image = np.full((150, 1050, 3), 27, dtype=np.uint8)
    put = lambda text, x, y, scale, color=(235, 235, 235): cv2.putText(
        image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)
    for i, (name, text) in enumerate([('ground', 'ground'), ('low_vegetation', 'low veg'),
                                      ('tall_vegetation', 'tall veg'), ('trunk_like', 'trunk'),
                                      ('solid_obstacle', 'solid')]):
        x = 14+i*150
        cv2.rectangle(image, (x, 16), (x+24, 40), tuple(int(c) for c in LIDAR_PALETTE[LIDAR_CLASSES.index(name)][::-1]), -1)
        put(text, x+30, 36, .62)
    put('= LiDAR only', 770, 36, .62, (160, 160, 160))
    area = cell*cell
    for x, state, text in [(14, 1, f'water, no LiDAR return {no_return*area:.1f} m2'),
                           (540, 2, f'water, LiDAR return {with_return*area:.1f} m2')]:
        cv2.rectangle(image, (x, 60), (x+24, 84), tuple(int(c) for c in WATER_RGB[state][::-1]), -1)
        put(text, x+30, 80, .62)
    put(f'Water = camera pixels projected on ground | scan {frame}/{total} | unvalidated', 14, 126, .62, (25, 178, 250))
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='offline_demo run folder')
    parser.add_argument('--replay', type=Path, required=True, help='bag folder to copy, e.g. the LiDAR feature demo')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stride', type=int, default=8, help='pixel sampling stride')
    parser.add_argument('--fill-iterations', type=int, default=8, help='LiDAR hole filling, cells')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    run = json.loads((args.run/'report.json').read_text())
    frames = [json.loads(line) for line in (args.run/'frames.jsonl').read_text().splitlines()]
    cell, max_range = run['resolution_m'], run['maximum_range_m']
    _, vehicle_camera, _ = recorded_geometry(yaml.safe_load((args.run/'calibration.yaml').read_text()))

    recording = Recording(Path(run['bag']))
    poses = list(recording.messages('/lidar_odometry', Odometry))
    recording.db.close()
    buffer = Buffer(cache_time=Duration(seconds=(poses[-1].header.stamp.sec-poses[0].header.stamp.sec)+5))
    for pose in poses:
        t = TransformStamped(header=pose.header, child_frame_id=pose.child_frame_id)
        p = pose.pose.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = pose.pose.pose.orientation
        buffer.set_transform(t, 'recorded_odometry')

    source = sqlite3.connect((list((args.run/'replay').glob('*.db3'))[0]).resolve().as_uri()+'?mode=ro', uri=True)
    topics = dict(source.execute('SELECT name,id FROM topics'))
    load = lambda name, kind: {ts: deserialize_message(raw, kind) for ts, raw in source.execute(
        'SELECT timestamp,data FROM messages WHERE topic_id=?', (topics[name],))}
    label_images, infos = load('/offline_demo/labels', Image), load('/offline_demo/camera_info', CameraInfo)
    source.close()

    args.output.mkdir(parents=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.output/'replay'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.replay), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    source_topics = reader.get_all_topics_and_types()
    if any(t.name.startswith('/water/') for t in source_topics):
        parser.error('Input replay already contains water topics')
    for topic in source_topics:
        writer.create_topic(topic)
    for name, kind in [('/water/cells', 'sensor_msgs/msg/PointCloud2'),
                       ('/water/camera_ground', 'sensor_msgs/msg/PointCloud2'),
                       ('/water/legend', 'sensor_msgs/msg/Image')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format='cdr'))

    bridge, evidence, summaries = CvBridge(), GroundEvidence(), []
    rays_cast = rays_hit = missing_pose = 0
    directions_camera = u = v = None
    for number, frame in enumerate(frames):
        stamp = frame['lidar_stamp_ns']
        with np.load(args.run/'frames'/frame['frame']/'voxels.npz') as saved:
            centres, labels = saved['centres'].astype(np.float64), saved['labels']
        terrain = analyze(centres, labels)
        image = label_images.get(stamp)
        if image is not None:
            info = infos[stamp]
            if directions_camera is None:
                u, v = pixel_grid(image.width, image.height, args.stride)
                directions_camera = camera_directions(np.array(info.k).reshape(3, 3), u, v)
            try:
                world_camera = transform_matrix(buffer.lookup_transform('sensor_init', 'vehicle',
                    Time.from_msg(image.header.stamp)).transform) @ vehicle_camera
            except TransformException:
                missing_pose += 1
                image = None
        if image is not None:
            classes = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width)
            ground = fill_ground(terrain.ground_z, args.fill_iterations)
            above = ~(terrain.voxel_height_m <= .5*cell)
            above_codes = np.sort(encode(voxel_keys(centres[above], cell)))
            directions = directions_camera @ world_camera[:3, :3].T
            hit_cells, points, hit = ground_hits(world_camera[:3, 3], directions, ground, terrain.origin,
                                                 cell, above_codes, max_range)
            rays_cast += len(hit); rays_hit += int(hit.sum())
            pixel_labels = classes[v.astype(int), u.astype(int)]
            evidence.add(hit_cells[hit]+terrain.origin, pixel_labels[hit], points[hit, 2], number)

        keys, fraction, seen, height = evidence.water()
        index = keys-terrain.origin
        inside = ((index >= 0) & (index < terrain.observed.shape)).all(1) if len(keys) else np.zeros(0, dtype=bool)
        returned = np.zeros(len(keys), dtype=bool)
        returned[inside] = terrain.observed[index[inside, 0], index[inside, 1]]
        state = np.where(returned, 2, 1)
        xyz = np.column_stack([(keys+.5)*cell, height+.2]) if len(keys) else np.zeros((0, 3))
        header = Header(frame_id='sensor_init')
        header.stamp.sec, header.stamp.nanosec = divmod(stamp, 10**9)
        writer.write('/water/cells', serialize_message(cloud(header, xyz, packed(WATER_RGB[state]),
                     [('water_fraction', fraction), ('frames', seen), ('lidar_return', returned.astype(float))])), stamp)
        ground_keys, ground_labels, ground_height = evidence.majority()
        gxyz = np.column_stack([(ground_keys+.5)*cell, ground_height]) if len(ground_keys) else np.zeros((0, 3))
        writer.write('/water/camera_ground', serialize_message(cloud(header, gxyz, packed(PALETTE[ground_labels]),
                     [('camera_label', ground_labels.astype(float))])), stamp)
        no_return, with_return = int((state == 1).sum()), int((state == 2).sum())
        image_msg = bridge.cv2_to_imgmsg(legend(frame['frame'], len(frames), no_return, with_return, cell), 'bgr8')
        image_msg.header = header
        writer.write('/water/legend', serialize_message(image_msg), stamp)
        summaries.append(dict(frame=frame['frame'], lidar_stamp_ns=stamp, water_no_lidar_return=no_return,
                              water_with_lidar_return=with_return, camera_ground_cells=int(len(ground_keys))))
        if (number+1) % 50 == 0:
            print(f'{number+1}/{len(frames)} frames: water cells {no_return}+{with_return}', flush=True)
    copied = 0
    while reader.has_next():
        writer.write(*reader.read_next())
        copied += 1
    del writer
    final = summaries[-1]
    report = dict(run=str(args.run.resolve()), copied_messages=copied, frames=len(frames),
                  pixel_stride=args.stride, fill_iterations=args.fill_iterations, max_range_m=max_range,
                  rays_cast=rays_cast, rays_hit_ground=rays_hit, frames_without_camera_pose=missing_pose,
                  thresholds=dict(min_frames=3, min_pixels=5, min_water_fraction=.5, camera_confidence=.6),
                  final_water_cells_no_lidar_return=final['water_no_lidar_return'],
                  final_water_cells_with_lidar_return=final['water_with_lidar_return'],
                  final_water_area_m2=(final['water_no_lidar_return']+final['water_with_lidar_return'])*cell*cell,
                  fused_water_voxels_in_run=run['last_frame']['class_counts'].get('water'),
                  note='Water comes from camera predictions (unvalidated) projected onto LiDAR ground; '
                       'heights in filled LiDAR holes are interpolated.')
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    with open(args.output/'frames.jsonl', 'w') as f:
        f.writelines(json.dumps(s)+'\n' for s in summaries)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
