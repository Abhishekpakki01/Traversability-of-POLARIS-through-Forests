#!/usr/bin/env python3
"""Find out why LiDAR-only classes leave tree and bush voxels unknown, on development sequences.

Only sequences 00000 and 00001 are used here, so 00002 to 00004 stay unseen for the final test.
Each stretch covers the same time span at several scan densities, which separates two causes:
too little data, and rules that are too strict.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluate_rellis_lidar_map import read_poses, stretch  # noqa: E402
from semantic_mapping.lidar_features import LIDAR_CLASSES  # noqa: E402
from semantic_mapping.terrain_analysis import settings_for_run  # noqa: E402

UNKNOWN = LIDAR_CLASSES.index('unknown')
TARGETS = {4: 'tree', 19: 'bush'}


def reasons(features, height, cell, unknown):
    """Why each unknown voxel failed, as exclusive counts."""
    points, passes = features['points'], features['passes']
    few_points = unknown & (points < 3)
    few_evidence = unknown & ~few_points & (points+passes < 5)
    rest = unknown & ~few_points & ~few_evidence
    low = rest & (height <= .3)
    return dict(under_3_points=int(few_points.sum()), under_5_hits_and_passes=int(few_evidence.sum()),
                solid_but_under_0_3_m=int(low.sum()), other=int((rest & ~low).sum()))


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path.home()/'thesis_data/rellis_3d_full/data/Rellis-3D')
    parser.add_argument('--terrain-config', type=Path, default=package/'config/traversability_v2.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sequences', nargs='+', default=['00000', '00001'])
    parser.add_argument('--stretches', type=int, default=2)
    parser.add_argument('--span', type=int, default=90, help='scans covered by a stretch')
    parser.add_argument('--strides', nargs='+', type=int, default=[3, 1])
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    if {'00002', '00003', '00004'} & set(args.sequences):
        parser.error('00002 to 00004 are held out for the final test')
    cell = .2
    settings = settings_for_run(yaml.safe_load(args.terrain_config.read_text()), cell)
    rows = []
    for sequence in args.sequences:
        folder = args.data/sequence
        poses = read_poses(folder/'poses.txt')
        # Offset from the published evaluation's stretches, so these are different scans.
        for first in np.linspace(2*args.span, len(poses)-3*args.span, args.stretches+2).astype(int)[1:-1]:
            for stride in args.strides:
                majority, classes, features, height, _ = stretch(folder, poses, int(first), args.span//stride, stride, cell, 20., settings, True)
                for label, name in TARGETS.items():
                    mask = majority == label
                    unknown = mask & (classes == UNKNOWN)
                    rows.append(dict(sequence=sequence, first_scan=int(first), stride=stride, label=name, voxels=int(mask.sum()),
                                     unknown=int(unknown.sum()), reasons=reasons(features, height, cell, unknown),
                                     classes={c: int((mask & (classes == i)).sum()) for i, c in enumerate(LIDAR_CLASSES)},
                                     median_points=float(np.median(features['points'][mask])) if mask.any() else None))
                print(sequence, first, 'stride', stride, [(r['label'], r['voxels'], round(r['unknown']/max(r['voxels'], 1), 2)) for r in rows[-2:]], flush=True)
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(rows, indent=2))
    for stride in args.strides:
        for name in TARGETS.values():
            part = [r for r in rows if r['stride'] == stride and r['label'] == name]
            total = sum(r['voxels'] for r in part)
            summed = {k: sum(r['reasons'][k] for r in part) for k in part[0]['reasons']}
            print(f'stride {stride} {name}: {total} voxels, unknown {sum(r["unknown"] for r in part)/total:.2f}',
                  {k: round(v/total, 3) for k, v in summed.items()})


if __name__ == '__main__':
    main()
