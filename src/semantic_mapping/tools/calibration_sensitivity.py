#!/usr/bin/python3
"""How do the camera's intrinsic and extrinsic parameters affect the camera-classified voxel map?

Replays a finished camera-cone run (build_offline_demo.py, --fusion cone): the same LiDAR voxel
map per scan, the same images, poses, network and fusion rule (2 views, 0.6 confidence). The
network runs once per image; the cone projection and fusion are then repeated for the dataset's
parameters ("dataset") and for changed ones, one change at a time:
  - extrinsics: camera tilted, panned or rolled, or moved, in its own optical frame;
  - intrinsics: focal length scaled, image centre shifted, raw (unrectified) K instead of P;
  - the publisher's calibration file (other mounting values) instead of the recorded TF chain.
For each change it reports, against "dataset":
  - pixel shift of the voxels the camera sees (median and 95th percentile);
  - voxels whose final class differs (changed, lost or newly classified);
  - share of the voxels the camera sees that are classified.
The "dataset" replay must reproduce the saved run's final classes; this is checked and reported.
TartanDrive has no labelled 3D data, so this measures sensitivity, not accuracy.
"""
import argparse
import json
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer
import yaml

from semantic_mapping.camera_projection import camera_offset, transform_matrix
from semantic_mapping.goose_inference import AspectPredictor, aspect_size, polygon_mask
from semantic_mapping.offline_demo import stamp_ns
from semantic_mapping.recorded_calibration import recorded_geometry
from semantic_mapping.semantic_fusion import CLASS_NAMES, SemanticEvidence, visible_in_cone, voxel_keys
from semantic_mapping.tartan_camera import pose_matrix

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_offline_demo import Recording  # noqa: E402  (same read-only bag reader as the build)


def variants(intrinsic, vehicle_camera, published):
    """(name, what changed, intrinsic, vehicle_from_camera); 'dataset' first."""
    def extrinsic(rotation=(0, 0, 0), shift=(0, 0, 0)):
        return intrinsic, vehicle_camera @ camera_offset(rotation, shift)
    def lens(fx_scale=1., cx_shift=0.):
        k = intrinsic.copy(); k[0, 0] *= fx_scale; k[1, 1] *= fx_scale; k[0, 2] += cx_shift
        return k, vehicle_camera
    raw = np.asarray(published['camera_K_raw'], dtype=float).reshape(3, 3)
    mounting = pose_matrix(published['vehicle_from_head']) @ pose_matrix(published['head_from_camera'])
    rows = [('dataset', 'none', intrinsic, vehicle_camera)]
    for degrees in (.5, 1., 2., -1.):
        rows.append((f'tilt {degrees:+g} deg', 'extrinsic rotation', *extrinsic((degrees, 0, 0))))
    for degrees in (1., 2.):
        rows.append((f'pan {degrees:+g} deg', 'extrinsic rotation', *extrinsic((0, degrees, 0))))
    rows.append(('roll +2 deg', 'extrinsic rotation', *extrinsic((0, 0, 2.))))
    for name, shift in (('right 5 cm', (.05, 0, 0)), ('right 10 cm', (.1, 0, 0)),
                        ('down 10 cm', (0, .1, 0)), ('forward 10 cm', (0, 0, .1))):
        rows.append((name, 'extrinsic translation', *extrinsic(shift=shift)))
    for scale in (.98, .95, 1.05):
        rows.append((f'focal x{scale:g}', 'intrinsic', *lens(fx_scale=scale)))
    rows.append(('cx +10 px', 'intrinsic', *lens(cx_shift=10.)))
    rows.append(('raw K instead of rectified P', 'intrinsic', raw, vehicle_camera))
    rows.append(('published mounting (extrinsics.yaml)', 'extrinsic, other source', intrinsic, mounting))
    return rows


def pixels(centres, world_camera, intrinsic):
    camera_world = np.linalg.inv(world_camera)
    camera = centres @ camera_world[:3, :3].T + camera_world[:3, 3]
    with np.errstate(divide='ignore', invalid='ignore'):
        return np.column_stack([intrinsic[0, 0]*camera[:, 0]/camera[:, 2] + intrinsic[0, 2],
                                intrinsic[1, 1]*camera[:, 1]/camera[:, 2] + intrinsic[1, 2]])


def main():
    shared = Path(get_package_share_directory('semantic_mapping'))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run', type=Path, required=True, help='finished build_offline_demo.py run with --fusion cone')
    parser.add_argument('--bag', type=Path, default=Path.home()/'thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2')
    parser.add_argument('--published-calibration', type=Path, default=shared/'config/tartan_meadows_camera.yaml')
    parser.add_argument('--model', type=Path, default=Path.home()/'semantic_map_ws/models/goose/ppliteseg_class_512.pth')
    parser.add_argument('--image-mask', type=Path, default=shared/'config/tartan_image_mask.json')
    parser.add_argument('--inference-short-side', type=int, default=512)
    parser.add_argument('--threads', type=int, default=6)
    parser.add_argument('--every', type=int, default=10, help='sample every n-th image for pixel shift and coverage')
    parser.add_argument('--max-frames', type=int, default=0, help='0 = the whole run; positive = partial test')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder')
    built = json.loads((args.run/'report.json').read_text())
    if built.get('fusion') != 'cone':
        parser.error('--run must be built with --fusion cone')
    resolution, reach = built['resolution_m'], built['cone_reach_m']
    config = yaml.safe_load((args.run/'calibration.yaml').read_text())
    width, height = config['width'], config['height']
    intrinsic, vehicle_camera, _ = recorded_geometry(config)
    rows = variants(intrinsic, vehicle_camera, yaml.safe_load(args.published_calibration.read_text()))

    recording = Recording(args.bag)
    poses = list(recording.messages('/lidar_odometry', Odometry))
    buffer = Buffer(cache_time=Duration(seconds=(stamp_ns(poses[-1])-stamp_ns(poses[0]))/1e9+2))
    for pose in poses:
        t = TransformStamped(header=pose.header, child_frame_id=pose.child_frame_id)
        p = pose.pose.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = pose.pose.pose.orientation
        buffer.set_transform(t, 'recorded_odometry')
    classes = json.loads((shared/'config/goose_classes.json').read_text())['classes']
    groups = [CLASS_NAMES.index(c['group']) for c in classes]
    mask = polygon_mask(args.image_mask, width, height)
    predictor = AspectPredictor(args.model, groups, args.threads,
                                aspect_size(width, height, args.inference_short_side), False, mask)
    bridge = CvBridge()
    images = recording.messages(config['image_topic'], Image)

    frames = [json.loads(line) for line in (args.run/'frames.jsonl').read_text().splitlines()]
    if args.max_frames:
        frames = frames[:args.max_frames]
    evidence = [SemanticEvidence() for _ in rows]
    shift = [[] for _ in rows]
    seen = np.zeros(len(rows)); classified = np.zeros(len(rows))
    start, image, used = time.monotonic(), None, 0
    for number, frame in enumerate(frames):
        with np.load(args.run/'frames'/frame['frame']/'voxels.npz') as saved:
            centres = saved['centres'].astype(float)
        keys = voxel_keys(centres, resolution)
        occupied = frozenset(map(tuple, keys))
        for store in evidence:
            store.retain_occupied(occupied)
        if frame['image_stamp_ns'] is None:
            continue
        while image is None or stamp_ns(image) < frame['image_stamp_ns']:
            image = next(images)
        if stamp_ns(image) != frame['image_stamp_ns']:
            raise ValueError(f"Image for frame {frame['frame']} not found in the recording")
        probabilities = predictor.predict(bridge.imgmsg_to_cv2(image, 'bgr8'))
        image_rows, image_columns = probabilities.shape[:2]
        world_vehicle = transform_matrix(buffer.lookup_transform(
            'sensor_init', 'vehicle', Time(nanoseconds=frame['image_stamp_ns'])).transform)
        sample = used % args.every == 0
        used += 1
        base_visible = None
        for index, (_, _, k, mount) in enumerate(rows):
            world_camera = world_vehicle @ mount
            near = np.flatnonzero(np.linalg.norm(centres-world_camera[:3, 3], axis=1) <= reach)
            shown, at = visible_in_cone(centres[near], world_camera, k, width, height, reach, resolution, mask)
            at = np.clip(np.floor((at+.5)*[image_columns/width, image_rows/height]).astype(int),
                         0, [image_columns-1, image_rows-1])
            values = probabilities[at[:, 1], at[:, 0]]
            good = np.isfinite(values).all(axis=1)
            visible_keys = keys[near][shown]
            evidence[index].update({tuple(key): p for key, p in zip(visible_keys[good].tolist(), values[good])},
                                   frame['image_stamp_ns'], occupied)
            if sample:
                labels, _, _ = evidence[index].query(visible_keys)
                seen[index] += len(labels); classified[index] += int((labels > 0).sum())
                if index == 0:
                    base_visible = centres[near][shown]
                    base_pixels = pixels(base_visible, world_camera, k)
                else:
                    moved = np.linalg.norm(pixels(base_visible, world_camera, k)-base_pixels, axis=1)
                    shift[index].extend(moved[np.isfinite(moved)].tolist())
        if used % 20 == 0:
            print(f'{used} images, {time.monotonic()-start:.0f} s', flush=True)

    with np.load(args.run/'frames'/frames[-1]['frame']/'voxels.npz') as saved:
        centres, saved_labels = saved['centres'].astype(float), saved['labels']
    keys = voxel_keys(centres, resolution)
    final = [store.query(keys)[0] for store in evidence]
    base = final[0]
    report = dict(run=str(args.run.resolve()), images_used=used, frames=len(frames), resolution_m=resolution,
                  cone_reach_m=reach,
                  dataset_replay_matches_saved_run=(float((base == saved_labels).mean())
                                                    if not args.max_frames else None),
                  note='Sensitivity to the camera parameters on TartanDrive, not accuracy (no labelled 3D data).',
                  variants=[])
    for index, (name, kind, _, _) in enumerate(rows):
        labels = final[index]
        both = (base > 0) & (labels > 0)
        entry = dict(name=name, changed=kind, classified_voxels=int((labels > 0).sum()),
                     in_view_classified_share=classified[index]/seen[index] if seen[index] else None)
        if index:
            moved = np.array(shift[index])
            entry.update(
                pixel_shift_median=float(np.median(moved)) if len(moved) else None,
                pixel_shift_p95=float(np.percentile(moved, 95)) if len(moved) else None,
                different_class_where_both_classified=float((labels[both] != base[both]).mean()) if both.any() else None,
                dataset_classified_now_unclassified=float(((base > 0) & (labels == 0)).sum()/max((base > 0).sum(), 1)),
                newly_classified_voxels=int(((base == 0) & (labels > 0)).sum()))
        report['variants'].append(entry)
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    lines = ['| Change | Pixel shift median / p95 | Different class (both classified) | Classified voxels | In view classified |',
             '|---|---|---|---|---|']
    for entry in report['variants']:
        lines.append('| {} | {} | {} | {} | {:.0%} |'.format(
            entry['name'],
            '—' if 'pixel_shift_median' not in entry else f"{entry['pixel_shift_median']:.1f} / {entry['pixel_shift_p95']:.1f} px",
            '—' if 'different_class_where_both_classified' not in entry else f"{entry['different_class_where_both_classified']:.1%}",
            entry['classified_voxels'], entry['in_view_classified_share'] or 0))
    (args.output/'results.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))
    print('dataset replay matches saved run:', report['dataset_replay_matches_saved_run'])


if __name__ == '__main__':
    main()
