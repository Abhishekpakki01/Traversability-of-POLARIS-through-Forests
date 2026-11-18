#!/usr/bin/python3
"""Compare two ways of giving camera classes to the voxel map, on the same drive.

  cone          every map voxel the camera can see is projected into the image
                (build_offline_demo.py --fusion cone, the default)
  lidar-points  only voxels that a LiDAR point of the same scan lands in
                (build_offline_demo.py --fusion lidar-points)

Both runs use the same recording, poses, calibration, network and fusion rule (2 views,
0.6 confidence), so differences come from the projection method only. Reported per run:
  - whole map: share of occupied voxels with a confirmed class, class counts;
  - in view: share of the voxels the camera sees at a scan (same visibility test for both)
    that are classified, overall and by distance from the camera;
  - stability: of voxels classified at a sampled scan and still in the map n scans later, the
    share whose class changed, and the share that lost confirmation;
  - agreement: for voxels classified by both runs at the end, share with the same class; for
    voxels only one run classified, how many lie under another occupied voxel (inside
    vegetation or below the surface, where the camera cannot see them directly).
No labelled 3D ground truth exists for TartanDrive, so this measures coverage and
consistency, not accuracy.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

from semantic_mapping.goose_inference import polygon_mask
from semantic_mapping.recorded_calibration import recorded_geometry
from semantic_mapping.semantic_fusion import CLASS_NAMES, visible_in_cone

RINGS = [(0, 5), (5, 10), (10, 15), (15, 20)]


def frames(run):
    return sorted((run/'frames').glob('*/voxels.npz'))


def keyed(saved, resolution):
    keys = np.floor(saved['centres']/resolution).astype(np.int64)
    return {tuple(k): int(label) for k, label in zip(keys.tolist(), saved['labels'].tolist())}


def summarise(run, mask_path, every, resolution, reach):
    config = yaml.safe_load((run/'calibration.yaml').read_text())
    intrinsic, vehicle_camera, _ = recorded_geometry(config)
    mask = polygon_mask(mask_path, config['width'], config['height']) if mask_path else None
    files = frames(run)
    seen = classified = 0
    rings = np.zeros((len(RINGS), 2))
    changed = lost = compared = 0
    previous = None
    for index, path in enumerate(files):
        if index % every:
            continue
        with np.load(path) as saved:
            current = keyed(saved, resolution)
            if previous is not None:
                for key, label in previous.items():
                    if label and key in current:
                        compared += 1
                        changed += current[key] not in (0, label)
                        lost += current[key] == 0
            centres, labels = saved['centres'], saved['labels']
            camera = saved['world_vehicle'] @ vehicle_camera
            shown, _ = visible_in_cone(centres, camera, intrinsic, config['width'], config['height'],
                                       reach, resolution, mask)
            distance = np.linalg.norm(centres[shown]-camera[:3, 3], axis=1)
            good = labels[shown] > 0
            seen += len(shown); classified += int(good.sum())
            for i, (low, high) in enumerate(RINGS):
                ring = (distance >= low) & (distance < high)
                rings[i] += [ring.sum(), (good & ring).sum()]
        previous = current
    with np.load(files[-1]) as last:
        labels = last['labels']
        final = keyed(last, resolution)
        whole = dict(occupied_voxels=int(len(labels)), classified_voxels=int((labels > 0).sum()),
                     classified_share=float((labels > 0).mean()),
                     class_counts={name: int((labels == i).sum()) for i, name in enumerate(CLASS_NAMES) if i})
    return final, dict(
        whole_map_at_end=whole,
        in_view=dict(sampled_every_n_scans=every, visible_voxel_samples=int(seen),
                     classified_share=classified/seen if seen else None,
                     by_distance_m={f'{low}-{high}': (count and hit/count) for (low, high), (count, hit)
                                    in zip(RINGS, rings.tolist())}),
        stability=dict(scans_apart=every, voxel_pairs_compared=int(compared),
                       class_changed_share=changed/compared if compared else None,
                       lost_confirmation_share=lost/compared if compared else None))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--cone', type=Path, required=True, help='run built with --fusion cone')
    parser.add_argument('--lidar-points', type=Path, required=True, help='run built with --fusion lidar-points')
    parser.add_argument('--image-mask', type=Path, help='same hood mask as the runs used')
    parser.add_argument('--every', type=int, default=10, help='sample every n-th scan for the in-view numbers')
    parser.add_argument('--resolution', type=float, default=.2)
    parser.add_argument('--reach', type=float, default=20.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder')
    report, finals = {}, {}
    for name, run in (('cone', args.cone), ('lidar_points', args.lidar_points)):
        built = json.loads((run/'report.json').read_text())
        expected = 'cone' if name == 'cone' else 'lidar-points'
        if built.get('fusion', 'lidar-points') != expected:
            parser.error(f'{run} was not built with --fusion {expected}')
        finals[name], report[name] = summarise(run, args.image_mask, args.every, args.resolution, args.reach)
    both = [key for key, label in finals['cone'].items() if label and finals['lidar_points'].get(key, 0)]
    same = sum(finals['cone'][key] == finals['lidar_points'][key] for key in both)
    occupied = set(finals['cone'])
    def group(keys):
        keys = list(keys)
        labels = np.bincount([finals['cone'].get(k) or finals['lidar_points'].get(k) for k in keys], minlength=6)
        return dict(voxels=len(keys), under_another_voxel_share=float(np.mean(
                        [(k[0], k[1], k[2]+1) in occupied for k in keys])) if keys else None,
                    class_counts={name: int(labels[i]) for i, name in enumerate(CLASS_NAMES) if i})
    report['agreement_at_end'] = dict(
        same_class_share_where_both_classified=same/len(both) if both else None,
        classified_by_both=group(both),
        classified_only_by_cone=group(k for k, label in finals['cone'].items()
                                      if label and not finals['lidar_points'].get(k, 0)),
        classified_only_by_lidar_points=group(k for k, label in finals['lidar_points'].items()
                                              if label and not finals['cone'].get(k, 0)))
    report['note'] = 'Coverage and consistency on TartanDrive; no labelled 3D ground truth, so not accuracy.'
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
