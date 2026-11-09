#!/usr/bin/env python3
"""Score LiDAR-only voxel classes against human point labels in RELLIS-3D.

Short stretches of each sequence are accumulated with the published poses into voxel maps, using
the v2 settings: 0.2 m voxels, 20 m range and cloth ground. Each voxel takes the majority human
label of its points, and a voxel counts as consistent if its LiDAR class is one that
rellis_lidar_expectations.json accepts for that label. That file and the class thresholds were
set before this test. The result measures agreement with labels, not traversability.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import yaml

from semantic_mapping.camera_projection import transform_points
from semantic_mapping.lidar_features import LIDAR_CLASSES, VoxelStatistics, fill_sparse, lidar_classes
from semantic_mapping.semantic_fusion import voxel_keys
from semantic_mapping.terrain_analysis import analyze, settings_for_run


def read_poses(path):
    rows = np.loadtxt(path).reshape(-1, 3, 4)
    poses = np.tile(np.eye(4), (len(rows), 1, 1))
    poses[:, :3] = rows
    return poses


def stretch(folder, poses, first, scans, stride, cell, max_range, settings, details=False):
    """Return RELLIS label and LiDAR class per voxel for one stretch of a sequence.

    With details, also return the features and height above ground.
    """
    stats = VoxelStatistics(cell)
    points_all, labels_all = [], []
    for index in range(first, first+scans*stride, stride):
        name = f'{index:06d}'
        raw = np.fromfile(folder/'os1_cloud_node_kitti_bin'/f'{name}.bin', dtype=np.float32).reshape(-1, 4)
        labels = np.fromfile(folder/'os1_cloud_node_semantickitti_label_id'/f'{name}.label', dtype=np.uint32) & 0xFFFF
        ranges = np.linalg.norm(raw[:, :3], axis=1)
        keep = np.isfinite(raw).all(1) & (ranges > 1.5) & (ranges <= max_range)
        world = transform_points(raw[keep, :3].astype(np.float64), poses[index])
        stats.add_points(world, raw[keep, 3].astype(np.float64))
        stats.add_rays(poses[index][:3, 3], world)
        points_all.append(world)
        labels_all.append(labels[keep])
    points, labels = np.vstack(points_all), np.concatenate(labels_all)
    keys, inverse = np.unique(voxel_keys(points, cell), axis=0, return_inverse=True)
    inverse = inverse.ravel()
    centres = (keys+.5)*cell
    height = analyze(centres, np.zeros(len(centres), np.uint8), settings).voxel_height_m
    features = stats.features(centres)
    classes = lidar_classes(features, height, cell)
    # Majority label per voxel.
    order = np.lexsort((labels, inverse))
    voxel, label = inverse[order], labels[order]
    starts = np.flatnonzero(np.r_[True, (voxel[1:] != voxel[:-1]) | (label[1:] != label[:-1])])
    counts = np.diff(np.r_[starts, len(voxel)])
    best = np.zeros(len(centres), np.int64)
    majority = np.zeros(len(centres), np.int64)
    for start, count in zip(starts, counts):
        if count > best[voxel[start]]:
            best[voxel[start]], majority[voxel[start]] = count, label[start]
    return (majority, classes, features, height, centres) if details else (majority, classes)


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path.home()/'thesis_data/rellis_3d_full/data/Rellis-3D')
    parser.add_argument('--expectations', type=Path, default=package/'config/rellis_lidar_expectations.json')
    parser.add_argument('--terrain-config', type=Path, default=package/'config/traversability_v2.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resolution', type=float, default=.2)
    parser.add_argument('--max-range', type=float, default=20.)
    parser.add_argument('--stretches', type=int, default=3, help='evenly spaced stretches per sequence')
    parser.add_argument('--scans', type=int, default=30, help='scans per stretch')
    parser.add_argument('--stride', type=int, default=3, help='use every nth scan')
    parser.add_argument('--sequences', nargs='+', help='only these sequences, e.g. 00002 00003 00004')
    parser.add_argument('--fill-sparse', action='store_true',
                        help='also score the rule that fills sparse voxels from classified neighbours')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    expectations = {int(k): v for k, v in json.loads(args.expectations.read_text())['classes'].items()}
    settings = settings_for_run(yaml.safe_load(args.terrain_config.read_text()), args.resolution)
    variants = ['as_measured', 'sparse_filled'] if args.fill_sparse else ['as_measured']
    tables = {v: {label: np.zeros(len(LIDAR_CLASSES), np.int64) for label in expectations} for v in variants}
    filled_tables = {label: np.zeros(len(LIDAR_CLASSES), np.int64) for label in expectations}
    stretches, started = [], time.time()
    for folder in sorted(p for p in args.data.iterdir() if p.is_dir() and (not args.sequences or p.name in args.sequences)):
        poses = read_poses(folder/'poses.txt')
        scans = len(list((folder/'os1_cloud_node_kitti_bin').glob('*.bin')))
        span = args.scans*args.stride
        for first in np.linspace(span, min(scans, len(poses))-2*span, args.stretches).astype(int):
            majority, classes, features, _, centres = stretch(folder, poses, int(first), args.scans, args.stride,
                                                              args.resolution, args.max_range, settings, True)
            versions = {'as_measured': classes}
            if args.fill_sparse:
                versions['sparse_filled'], filled = fill_sparse(centres, classes, features, args.resolution)
                for label in filled_tables:
                    filled_tables[label] += np.bincount(versions['sparse_filled'][filled & (majority == label)],
                                                        minlength=len(LIDAR_CLASSES))
            for name, values in versions.items():
                for label in tables[name]:
                    tables[name][label] += np.bincount(values[majority == label], minlength=len(LIDAR_CLASSES))
            stretches.append(dict(sequence=folder.name, first_scan=int(first), voxels=len(classes)))
            print(f'{folder.name} scan {first}: {len(classes)} voxels, {time.time()-started:.0f} s', flush=True)
    def score(table):
        per_class, voxels, consistent = {}, 0, 0
        for label, counts in table.items():
            total = int(counts.sum())
            if not total:
                continue
            accept = [LIDAR_CLASSES.index(c) for c in expectations[label]['accept']]
            ok = int(counts[accept].sum())
            voxels += total
            consistent += ok
            per_class[expectations[label]['name']] = dict(
                voxels=total, accepted=expectations[label]['accept'], consistent_fraction=ok/total,
                lidar_class_fraction={c: float(counts[i]/total) for i, c in enumerate(LIDAR_CLASSES)})
        return dict(scored_voxels=voxels, consistent_fraction=consistent/max(voxels, 1),
                    mean_class_consistent_fraction=float(np.mean([v['consistent_fraction'] for v in per_class.values()])),
                    per_rellis_class=per_class)
    results = {name: score(table) for name, table in tables.items()}
    report = dict(resolution_m=args.resolution, max_range_m=args.max_range, ground_method=settings.ground_method,
                  sequences=args.sequences or 'all', stretches=stretches, **results['as_measured'],
                  note='Agreement of rule-based LiDAR classes with majority human labels per voxel.')
    if args.fill_sparse:
        report['sparse_filled'] = results['sparse_filled']
        report['filled_voxels_only'] = score(filled_tables)
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    for name, result in results.items():
        print(f"\n{name}: scored voxels {result['scored_voxels']}, consistent {result['consistent_fraction']:.3f}, "
              f"class mean {result['mean_class_consistent_fraction']:.3f}")
        for label, value in sorted(result['per_rellis_class'].items(), key=lambda kv: -kv[1]['voxels'])[:6]:
            print(f"  {label:10s} {value['voxels']:8d} voxels  consistent {value['consistent_fraction']:.3f}  "
                  f"unknown {value['lidar_class_fraction']['unknown']:.3f}")
    if args.fill_sparse:
        only = report['filled_voxels_only']
        print(f"\nfilled voxels only: {only['scored_voxels']}, consistent {only['consistent_fraction']:.3f}")


if __name__ == '__main__':
    main()
