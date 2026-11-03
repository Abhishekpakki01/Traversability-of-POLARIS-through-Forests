#!/usr/bin/env python3
"""Compare LiDAR-only features and classes with a human-labelled RELLIS scan.

Uses the example Ouster scan (000104.bin + SemanticKITTI-style 000104.label).
One scan from one viewpoint: permeability evidence is sparse, and the result
is a sanity check of whether the features separate classes, not accuracy.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib.patches import Patch
import numpy as np

from semantic_mapping.lidar_features import (
    LIDAR_CLASSES, LIDAR_PALETTE, LidarClassSettings, VoxelStatistics, lidar_classes)
from semantic_mapping.semantic_fusion import voxel_keys
from semantic_mapping.terrain_analysis import analyze, settings_for_run
import yaml

EXAMPLE = Path.home()/'RELLIS-3D/utils/example'
FEATURES = ['permeability', 'linearity_neighbourhood', 'planarity_neighbourhood',
            'scattering_neighbourhood', 'height_above_ground_m']
INK, MUTED = '#0b0b0b', '#898781'


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan', type=Path, default=EXAMPLE/'000104.bin')
    parser.add_argument('--labels', type=Path, default=EXAMPLE/'000104.label')
    parser.add_argument('--expectations', type=Path, default=package/'config/rellis_lidar_expectations.json')
    parser.add_argument('--max-range', type=float, default=10., help='same 10 m limit as the mapping pipeline')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--terrain-config', type=Path,
                        default=package/'config/traversability_placeholder.yaml',
                        help='ground/height settings (traversability_v2.yaml = cloth ground)')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    raw = np.fromfile(args.scan, dtype=np.float32).reshape(-1, 4)
    labels = np.fromfile(args.labels, dtype=np.uint32) & 0xFFFF
    if len(raw) != len(labels):
        raise ValueError('Scan and label files have different point counts')
    expectations = {int(k): v for k, v in json.loads(args.expectations.read_text())['classes'].items()}
    ranges = np.linalg.norm(raw[:, :3], axis=1)
    keep = np.isfinite(raw).all(1) & (ranges > .1) & (ranges <= args.max_range)
    points, intensity, labels = raw[keep, :3].astype(np.float64), raw[keep, 3], labels[keep]

    cell = .35
    stats = VoxelStatistics(cell)
    stats.add_points(points, intensity)
    stats.add_rays(np.zeros(3), points)
    keys, inverse = np.unique(voxel_keys(points, cell), axis=0, return_inverse=True)
    centres = (keys+.5)*cell
    features = stats.features(centres)
    terrain_settings = settings_for_run(yaml.safe_load(args.terrain_config.read_text()), cell)
    terrain = analyze(centres, np.zeros(len(centres), dtype=np.uint8), terrain_settings)
    height = terrain.voxel_height_m
    classes = lidar_classes(features, height, cell)
    point_class = classes[inverse]
    features['height_above_ground_m'] = height

    table, scored, consistent = {}, 0, 0
    for label in np.unique(labels).tolist():
        if label not in expectations:
            continue
        mask = labels == label
        voxels = np.unique(inverse[mask])
        counts = np.bincount(point_class[mask], minlength=len(LIDAR_CLASSES))
        accept = [LIDAR_CLASSES.index(c) for c in expectations[label]['accept']]
        ok = int(counts[accept].sum())
        scored += int(mask.sum()); consistent += ok
        table[expectations[label]['name']] = dict(
            rellis_id=label, points=int(mask.sum()), voxels=int(len(voxels)),
            accepted=expectations[label]['accept'], consistent_fraction=ok/int(mask.sum()),
            lidar_class_fraction={c: float(counts[i]/mask.sum()) for i, c in enumerate(LIDAR_CLASSES)},
            median_features={f: (float(np.nanmedian(features[f][voxels]))
                                 if np.isfinite(features[f][voxels]).any() else None) for f in FEATURES})
    report = dict(scan=str(args.scan.resolve()), labels=str(args.labels.resolve()), frames_evaluated=1,
                  max_range_m=args.max_range, points_used=int(len(points)), voxels=int(len(centres)),
                  class_settings=vars(LidarClassSettings()), ground_method=terrain_settings.ground_method,
                  note='One labelled scan from one viewpoint: a sanity check, not accuracy. '
                       'Thresholds are a priori, not fitted to this scan.',
                  consistent_fraction_scored_points=consistent/max(scored, 1), scored_points=scored,
                  per_rellis_class=table)
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')

    # Median features per RELLIS class, one panel per feature (single hue).
    names = sorted(table, key=lambda n: -table[n]['points'])
    fig, axes = plt.subplots(1, len(FEATURES), figsize=(15, 3.6), sharey=True, constrained_layout=True)
    for ax, feature in zip(axes, FEATURES):
        values = [table[n]['median_features'][feature] for n in names]
        ax.barh(range(len(names)), [v if v is not None else 0 for v in values], color='#2a78d6', height=.6)
        for i, v in enumerate(values):
            ax.text((v or 0)+.01, i, 'n/a' if v is None else f'{v:.2f}', va='center', fontsize=7, color=INK)
        ax.set_title(feature.replace('_neighbourhood', ' (1.05 m)').replace('_', ' '), loc='left', fontsize=9, color=INK)
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.grid(axis='x', color='#e1e0d9', lw=.6)
        for spine in ax.spines.values():
            spine.set_color('#c3c2b7')
    axes[0].set_yticks(range(len(names)), [f'{n} ({table[n]["points"]})' for n in names])
    axes[0].invert_yaxis()
    fig.suptitle('RELLIS scan 000104: median LiDAR features per labelled class (points in brackets)',
                 fontsize=10, color=INK, x=.01, ha='left')
    fig.savefig(args.output/'feature_medians.png', dpi=160, facecolor='#fcfcfb')
    plt.close(fig)

    # Top-down: LiDAR-only classes beside the human labels.
    fig, axes = plt.subplots(1, 2, figsize=(13, 6.5), constrained_layout=True)
    colours = LIDAR_PALETTE/255.
    axes[0].scatter(points[:, 0], points[:, 1], s=1, c=colours[point_class], linewidths=0)
    axes[0].set_title('LiDAR-only classes (no camera)', loc='left', fontsize=10, color=INK)
    # Display colour of each human label: its first accepted LiDAR class
    # (grass therefore shows as ground, tree as tall vegetation).
    expected = np.zeros(len(points), dtype=int)
    for label, entry in expectations.items():
        expected[labels == label] = LIDAR_CLASSES.index(entry['accept'][0])
    axes[1].scatter(points[:, 0], points[:, 1], s=1, c=colours[expected], linewidths=0)
    axes[1].set_title('RELLIS human labels, grouped (grass shown as ground)', loc='left', fontsize=10, color=INK)
    for ax in axes:
        ax.set_aspect('equal'); ax.tick_params(colors=MUTED, labelsize=7)
        ax.set_xlabel('x (m)', color=MUTED, fontsize=8); ax.set_ylabel('y (m)', color=MUTED, fontsize=8)
    fig.legend([Patch(color=c) for c in colours], [c.replace('_', ' ') for c in LIDAR_CLASSES],
               loc='lower center', ncol=6, frameon=False, fontsize=8, bbox_to_anchor=(.5, -.05))
    fig.savefig(args.output/'top_down_comparison.png', dpi=160, facecolor='#fcfcfb', bbox_inches='tight')
    plt.close(fig)
    print(json.dumps({k: report[k] for k in ('points_used', 'voxels', 'consistent_fraction_scored_points')}, indent=2))
    for name in names:
        t = table[name]
        print(f"{name:9s} {t['points']:6d} consistent {t['consistent_fraction']:.2f}",
              {c: round(f, 2) for c, f in t['lidar_class_fraction'].items() if f >= .05},
              {f: (round(v, 2) if v is not None else None) for f, v in t['median_features'].items()})


if __name__ == '__main__':
    main()
