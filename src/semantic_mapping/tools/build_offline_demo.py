#!/usr/bin/env python3
"""Process the Tartan excerpt in order, save an inspectable bag and video inputs.

This runs offline: no ROS nodes, DDS delivery, wall-time timeouts or latest-frame
selection. Original acquisition timestamps drive matching and pose lookup.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sqlite3
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
import numpy as np
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
import rosbag2_py
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException
import yaml

from semantic_mapping.camera_projection import (
    draw_projection, project_visible_points, transform_matrix, transform_points, xyz_from_cloud)
from semantic_mapping.goose_inference import AspectPredictor, aspect_size, polygon_mask
from semantic_mapping.goose_model import GoosePredictor
from semantic_mapping.offline_demo import ELEVATION_RANGE, NearestImage, OfflineOhm, cloud_message, stamp_ns
from semantic_mapping.recorded_calibration import recorded_geometry
from semantic_mapping.semantic_fusion import (
    CLASS_NAMES, PALETTE, SemanticEvidence, surface_observations, voxel_keys)
from semantic_mapping.tartan_camera import quaternion_from_matrix


class Recording:
    """Read the local single-file SQLite excerpt without modifying it."""
    def __init__(self, path):
        databases = list(path.glob('*.db3'))
        if len(databases) != 1:
            raise ValueError('This runner supports one SQLite .db3 file per recording')
        self.file = databases[0]
        self.db = sqlite3.connect(self.file.resolve().as_uri()+'?mode=ro', uri=True)
        self.topics = dict(self.db.execute('SELECT name,id FROM topics'))

    def count(self, topic):
        return self.db.execute('SELECT count(*) FROM messages WHERE topic_id=?',
                               (self.topics[topic],)).fetchone()[0]

    def messages(self, topic, kind):
        rows = self.db.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id',
                               (self.topics[topic],))
        for raw, in rows:
            yield deserialize_message(raw, kind)


def transform(header, child, matrix):
    result = TransformStamped(header=header, child_frame_id=child)
    t, q = matrix[:3, 3], quaternion_from_matrix(matrix[:3, :3])
    (result.transform.translation.x, result.transform.translation.y,
     result.transform.translation.z) = map(float, t)
    (result.transform.rotation.x, result.transform.rotation.y,
     result.transform.rotation.z, result.transform.rotation.w) = map(float, q)
    return result


def sha(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def status_image(status):
    image = np.full((240, 800, 3), 27, dtype=np.uint8)
    lines = ['OFFLINE PROCESSED RECORDING | TartanDrive down_meadows',
             f"Scan {status['scans_seen']}/{status['source_scans']} | fused {status['fused_frames']} | t={status['elapsed_recording_seconds']:.2f}s",
             f"Occupied {status['occupied_voxels']} | classified {status['classified_voxels']} | with evidence {status['voxels_with_evidence']}",
             'Calibration accuracy unverified | predictions, no navigation decision',
             'Elevation view: world Z minus starting vehicle Z (not ground clearance)']
    for i, line in enumerate(lines):
        cv2.putText(image, line, (12, 27+i*31), cv2.FONT_HERSHEY_SIMPLEX, .55,
                    (235, 235, 235), 1, cv2.LINE_AA)
    for i, (name, rgb) in enumerate(zip(CLASS_NAMES, PALETTE)):
        x = 12+i*130
        cv2.rectangle(image, (x, 188), (x+15, 203), tuple(map(int, rgb[::-1])), -1)
        cv2.putText(image, name, (x+20, 201), cv2.FONT_HERSHEY_SIMPLEX, .46,
                    (235, 235, 235), 1, cv2.LINE_AA)
    return image


def main():
    shared = Path(get_package_share_directory('semantic_mapping'))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=Path.home()/'thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, default=shared/'config/tartan_recorded_calibration.yaml')
    parser.add_argument('--model', type=Path, default=Path.home()/'semantic_map_ws/models/goose/ppliteseg_class_512.pth')
    parser.add_argument('--max-scans', type=int, default=0, help='0 processes the whole recording; positive values are explicitly partial tests')
    parser.add_argument('--strict-voxel-occlusion', action='store_true')
    parser.add_argument('--exclude-vehicle-box', type=float, nargs=6, metavar=('XMIN', 'YMIN', 'ZMIN', 'XMAX', 'YMAX', 'ZMAX'),
                        help='drop LiDAR returns inside this vehicle-frame box (ego-vehicle self-returns); off by default')
    parser.add_argument('--resolution', type=float, default=.35, help='voxel size in metres')
    parser.add_argument('--max-range', type=float, default=10., help='LiDAR and projection range in metres')
    parser.add_argument('--inference-short-side', type=int, default=0,
                        help='0 keeps the original 512x512 squash; e.g. 512 keeps the aspect ratio')
    parser.add_argument('--mirror-average', action='store_true', help='average predictions with the mirrored image')
    parser.add_argument('--image-mask', type=Path, help='polygon JSON of image pixels to ignore (e.g. hood)')
    parser.add_argument('--threads', type=int, default=2, help='CPU threads for inference')
    args = parser.parse_args()
    if args.max_scans < 0:
        parser.error('--max-scans must be nonnegative')
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    vehicle_box = None
    if args.exclude_vehicle_box:
        vehicle_box = np.array(args.exclude_vehicle_box).reshape(2, 3)
        if not np.isfinite(vehicle_box).all() or (vehicle_box[0] >= vehicle_box[1]).any():
            parser.error('--exclude-vehicle-box needs finite minimum values below maximum values')
    config = yaml.safe_load(args.calibration.read_text())
    intrinsic, vehicle_camera, vehicle_lidar = recorded_geometry(config)
    recording = Recording(args.bag)
    cloud_topic = '/velodyne_1/velodyne_points'
    source_scans = recording.count(cloud_topic)
    # Preload recorded poses for interpolation at both acquisition times. No extrapolation.
    poses = list(recording.messages('/lidar_odometry', Odometry))
    if len(poses) < 2:
        raise ValueError('Need at least two recorded poses')
    buffer = Buffer(cache_time=Duration(seconds=(stamp_ns(poses[-1])-stamp_ns(poses[0]))/1e9+2))
    for pose in poses:
        if pose.header.frame_id != 'sensor_init' or pose.child_frame_id != 'vehicle':
            raise ValueError('Unexpected odometry frames')
        t = TransformStamped(header=pose.header, child_frame_id=pose.child_frame_id)
        p = pose.pose.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = pose.pose.pose.orientation
        buffer.set_transform(t, 'recorded_odometry')
    classes_file = shared/'config/goose_classes.json'
    classes = json.loads(classes_file.read_text())['classes']
    groups = [CLASS_NAMES.index(c['group']) for c in classes]
    print('Loading the existing pretrained model; thresholds remain 2 observations / 0.6 confidence.', flush=True)
    legacy = not (args.inference_short_side or args.mirror_average or args.image_mask)
    if legacy:
        predictor = GoosePredictor(args.model, groups, args.threads)
        inference = dict(input='512x512 squash', mirror_average=False, image_mask=None)
    else:
        size = (aspect_size(config['width'], config['height'], args.inference_short_side)
                if args.inference_short_side else (512, 512))
        mask = polygon_mask(args.image_mask, config['width'], config['height']) if args.image_mask else None
        predictor = AspectPredictor(args.model, groups, args.threads, size, args.mirror_average, mask)
        inference = dict(input=f'{size[0]}x{size[1]}', mirror_average=args.mirror_average,
                         image_mask=None if mask is None else str(args.image_mask.resolve()),
                         masked_pixels=0 if mask is None else int(mask.sum()))
    if args.resolution <= 0 or args.max_range <= 0:
        parser.error('Resolution and range must be positive')
    executable = Path(get_package_prefix('semantic_mapping'))/'lib/semantic_mapping/ohm_offline_mapper'
    mapper = OfflineOhm(executable, args.resolution)
    args.output.mkdir(parents=True)
    (args.output/'frames').mkdir()
    (args.output/'calibration.yaml').write_text(args.calibration.read_text())
    (args.output/'class_mapping.json').write_text(classes_file.read_text())
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.output/'replay'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topics = {
        '/tf': 'tf2_msgs/msg/TFMessage', '/vehicle_pose': 'geometry_msgs/msg/PoseStamped',
        '/offline_demo/vehicle_path': 'nav_msgs/msg/Path',
        '/semantic_voxel_map': 'sensor_msgs/msg/PointCloud2',
        '/offline_demo/elevation': 'sensor_msgs/msg/PointCloud2',
        '/offline_demo/camera': 'sensor_msgs/msg/Image',
        '/offline_demo/camera_info': 'sensor_msgs/msg/CameraInfo',
        '/offline_demo/labels': 'sensor_msgs/msg/Image',
        '/offline_demo/segmentation': 'sensor_msgs/msg/Image',
        '/offline_demo/projection': 'sensor_msgs/msg/Image',
        '/offline_demo/status_image': 'sensor_msgs/msg/Image',
        '/offline_demo/status': 'std_msgs/msg/String'}
    for topic, kind in topics.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=kind, serialization_format='cdr'))
    def write(topic, message, timestamp):
        writer.write(topic, serialize_message(message), timestamp)
    bridge, evidence = CvBridge(), SemanticEvidence()
    matcher = NearestImage(recording.messages(config['image_topic'], Image))
    counters = Counter(scans_seen=0, mapped_scans=0, matched_images=0, inferred_images=0,
                       fused_frames=0, missing_lidar_pose=0, missing_camera_pose=0,
                       no_unique_image=0, empty_cloud=0, vehicle_box_points_removed=0)
    trajectory, frames, durations = [], [], []
    start = time.monotonic()
    first_stamp, first_z = None, None
    report = dict(mode='offline_sequential', source_scans=source_scans,
                  source_images=recording.count(config['image_topic']),
                  bag=str(args.bag.resolve()), calibration_sha256=sha(args.calibration),
                  checkpoint_sha256=sha(args.model), source_db_sha256=sha(recording.file),
                  resolution_m=args.resolution, maximum_range_m=args.max_range, inference=inference, min_observations=2, min_confidence=.6,
                  physical_calibration_validated=False, semantic_accuracy_measured=False,
                  traversability_implemented=False, elevation_reference='world_z_minus_first_mapped_vehicle_z',
                  elevation_range_m=list(ELEVATION_RANGE),
                  visibility_mode='solid_voxel_rays' if args.strict_voxel_occlusion else 'measured_lidar_depth',
                  requested_scan_limit=args.max_scans, completed_requested_input=False,
                  vehicle_box_m=None if vehicle_box is None else vehicle_box.tolist())
    journal = open(args.output/'frames.jsonl', 'w')
    try:
        for cloud in recording.messages(cloud_topic, PointCloud2):
            if args.max_scans and counters['scans_seen'] >= args.max_scans:
                break
            tick = time.monotonic()
            counters['scans_seen'] += 1
            stamp = stamp_ns(cloud)
            if first_stamp is None:
                first_stamp = stamp
            image = matcher.match(stamp)
            if image is None:
                counters['no_unique_image'] += 1
            else:
                counters['matched_images'] += 1
            if cloud.header.frame_id != 'velodyne_1':
                raise ValueError('Unexpected LiDAR frame')
            try:
                world_vehicle = transform_matrix(buffer.lookup_transform('sensor_init', 'vehicle',
                    Time.from_msg(cloud.header.stamp)).transform)
            except TransformException:
                counters['missing_lidar_pose'] += 1
                continue
            world_lidar = world_vehicle @ vehicle_lidar
            points = xyz_from_cloud(cloud)
            if vehicle_box is not None:
                # Self-returns are not terrain: remove them from geometry and camera depth alike.
                in_vehicle = transform_points(points, vehicle_lidar)
                inside = ((in_vehicle >= vehicle_box[0]) & (in_vehicle <= vehicle_box[1])).all(axis=1)
                counters['vehicle_box_points_removed'] += int(inside.sum())
                points = points[~inside]
            ranges = np.linalg.norm(points, axis=1)
            valid = np.isfinite(points).all(axis=1) & (ranges > .1) & (ranges <= args.max_range)
            if not valid.any():
                counters['empty_cloud'] += 1
                continue
            centres = mapper.integrate(world_lidar[:3, 3], transform_points(points[valid], world_lidar))
            counters['mapped_scans'] += 1
            keys = voxel_keys(centres, args.resolution)
            occupied = frozenset(map(tuple, keys))
            evidence.retain_occupied(occupied)
            header = Header(stamp=cloud.header.stamp, frame_id='sensor_init')
            tf = transform(header, 'vehicle', world_vehicle)
            vehicle_pose = PoseStamped(header=header)
            vehicle_pose.pose.position.x, vehicle_pose.pose.position.y, vehicle_pose.pose.position.z = map(float, world_vehicle[:3, 3])
            vehicle_pose.pose.orientation = tf.transform.rotation
            trajectory.append(vehicle_pose)
            if first_z is None:
                first_z = float(world_vehicle[2, 3])
            write('/tf', TFMessage(transforms=[tf]), stamp)
            write('/vehicle_pose', vehicle_pose, stamp)
            write('/offline_demo/vehicle_path', RosPath(header=header, poses=trajectory), stamp)
            frame_id = f"{counters['mapped_scans']:05d}"
            folder = args.output/'frames'/frame_id
            folder.mkdir()
            entry = dict(frame=frame_id, lidar_stamp_ns=stamp, image_stamp_ns=None,
                         elapsed_recording_seconds=(stamp-first_stamp)/1e9,
                         projection_points=0, updated_voxels=0)
            if image is not None:
                if (image.header.frame_id != config['image_frame'] or image.width != config['width']
                        or image.height != config['height']):
                    raise ValueError('Image is incompatible with the selected calibration')
                try:
                    world_camera = transform_matrix(buffer.lookup_transform('sensor_init', 'vehicle',
                        Time.from_msg(image.header.stamp)).transform) @ vehicle_camera
                except TransformException:
                    counters['missing_camera_pose'] += 1
                    image = None
            if image is not None:
                bgr = bridge.imgmsg_to_cv2(image, 'bgr8')
                projection = project_visible_points(points, bgr, intrinsic, [],
                    np.linalg.solve(world_camera, world_lidar), max_range=args.max_range, visibility_cell=4, depth_tolerance=0.)
                probabilities = predictor.predict(bgr)
                rows, columns = probabilities.shape[:2]
                pixels = np.floor((projection.pixels+.5)*[columns/image.width, rows/image.height]).astype(int)
                pixels = np.clip(pixels, 0, [columns-1, rows-1])
                sampled = probabilities[pixels[:, 1], pixels[:, 0]]
                usable = np.isfinite(sampled).all(axis=1)  # masked pixels (e.g. hood) give no observation
                observations, visibility = surface_observations(
                    transform_points(points[projection.source_indices[usable]], world_lidar),
                    sampled[usable], world_camera[:3, 3], occupied,
                    args.resolution, args.strict_voxel_occlusion)
                visibility['masked_samples'] = int((~usable).sum())
                updated = evidence.update(observations, stamp_ns(image), occupied)
                counters['inferred_images'] += 1
                counters['fused_frames'] += int(updated > 0)
                masked = ~np.isfinite(probabilities).all(axis=2)
                labels_image = np.nan_to_num(probabilities, nan=0.).argmax(axis=2).astype(np.uint8)
                labels_image[(np.nan_to_num(probabilities, nan=0.).max(axis=2) < .6) | masked] = 0
                labels_image = cv2.resize(labels_image, (image.width, image.height), interpolation=cv2.INTER_NEAREST)
                segmentation = cv2.addWeighted(bgr, .4, PALETTE[labels_image][:, :, ::-1], .6, 0)
                projected = draw_projection(bgr, projection, args.max_range)
                write('/offline_demo/camera', image, stamp)
                info = CameraInfo(header=image.header, width=image.width, height=image.height,
                    distortion_model='plumb_bob', d=[0.]*5, k=intrinsic.reshape(-1).tolist(),
                    r=np.eye(3).reshape(-1).tolist(),
                    p=np.column_stack([intrinsic, np.zeros(3)]).reshape(-1).tolist())
                write('/offline_demo/camera_info', info, stamp)
                for name, value, encoding in [('segmentation', segmentation, 'bgr8'),
                    ('projection', projected, 'bgr8'), ('labels', labels_image, 'mono8')]:
                    message = bridge.cv2_to_imgmsg(value, encoding)
                    message.header = image.header
                    write('/offline_demo/'+name, message, stamp)
                for name, value in [('camera', bgr), ('segmentation', segmentation), ('projection', projected)]:
                    cv2.imwrite(str(folder/(name+'.jpg')), value, [cv2.IMWRITE_JPEG_QUALITY, 94])
                entry.update(image_stamp_ns=stamp_ns(image), image_minus_lidar_ms=(stamp_ns(image)-stamp)/1e6,
                             projection_points=len(projection.pixels), updated_voxels=updated, visibility=visibility)
            labels, confidence, counts = evidence.query(keys)
            relative_z = centres[:, 2]-first_z
            semantic = cloud_message(header, centres, labels, confidence, counts, relative_z)
            elevation = cloud_message(header, centres, labels, confidence, counts, relative_z, True)
            write('/semantic_voxel_map', semantic, stamp)
            write('/offline_demo/elevation', elevation, stamp)
            seen = counts > 0
            status = dict(counters, source_scans=source_scans,
                elapsed_recording_seconds=(stamp-first_stamp)/1e9, occupied_voxels=len(centres),
                classified_voxels=int(np.count_nonzero(labels)), voxels_with_evidence=int(seen.sum()),
                class_counts={name: int(np.count_nonzero(labels == i)) for i, name in enumerate(CLASS_NAMES)},
                physical_calibration_validated=False, offline_processed=True)
            write('/offline_demo/status', String(data=json.dumps(status)), stamp)
            status_pixels = status_image(status)
            status_msg = bridge.cv2_to_imgmsg(status_pixels, 'bgr8'); status_msg.header = header
            write('/offline_demo/status_image', status_msg, stamp)
            np.savez_compressed(folder/'voxels.npz', centres=centres.astype(np.float32), labels=labels,
                confidence=confidence, observations=counts, relative_z=relative_z.astype(np.float32),
                world_vehicle=world_vehicle, trajectory=np.array([[p.pose.position.x,
                p.pose.position.y, p.pose.position.z] for p in trajectory]))
            entry.update(status)
            entry['processing_seconds'] = time.monotonic()-tick
            durations.append(entry['processing_seconds'])
            frames.append(entry)
            journal.write(json.dumps(entry)+'\n'); journal.flush()
            if counters['mapped_scans'] % 10 == 0:
                print(f"Scan {counters['scans_seen']}/{source_scans}: fused {counters['fused_frames']}, "
                      f"classified {status['classified_voxels']}/{len(centres)}, {entry['processing_seconds']:.2f}s", flush=True)
        report['completed_requested_input'] = True
    finally:
        mapper.close(); journal.close(); recording.db.close()
        del writer  # finalize rosbag metadata before producing the report
        report.update(counters)
        report['wall_seconds'] = time.monotonic()-start
        report['full_recording_processed'] = report['completed_requested_input'] and counters['scans_seen'] == source_scans
        report['last_frame'] = frames[-1] if frames else None
        report['processing_seconds_median'] = float(np.median(durations)) if durations else None
        report['processing_seconds_p95'] = float(np.percentile(durations, 95)) if durations else None
        report['saved_frames'] = len(frames)
        report['input_accounting_ok'] = counters['scans_seen'] == counters['mapped_scans']+counters['missing_lidar_pose']+counters['empty_cloud']
        report['passed_functional_checks'] = (report['completed_requested_input'] and report['input_accounting_ok']
            and counters['fused_frames'] > 1 and bool(frames) and frames[-1]['classified_voxels'] > 0)
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)
    if not report['passed_functional_checks']:
        raise RuntimeError('Functional checks failed; see report.json')


if __name__ == '__main__':
    main()
