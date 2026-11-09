#!/usr/bin/env python3
"""Collect the thesis result numbers and two figures, each traced to its source report.

Every value is read from a saved report, so nothing is typed by hand. Use results.csv to check a
number before quoting it. This collects results only: it contains no thesis text.
"""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import matplotlib.ticker
from matplotlib.patches import Patch
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_offline_demo import Recording  # noqa: E402

INK, MUTED, GRID, SURFACE = '#0b0b0b', '#52514e', '#e1e0d9', '#fcfcfb'
SERIES = ['#2a78d6', '#eb6834', '#1baf7a']  # slots 1-3 of the dataviz reference palette
SOURCES = dict(
    run='artifacts/offline_v2_2026_11_03/report.json',
    gps='artifacts/trajectory_vs_gps_2026_11_03/report.json',
    sharpness='artifacts/map_sharpness_2026_11_03/report.json',
    terrain='artifacts/terrain_analysis_v2_2026_11_03/report.json',
    terrain_v1_unfiltered='artifacts/terrain_analysis_2026_11_02/report.json',
    terrain_v1_filtered='artifacts/terrain_analysis_ego_filtered_2026_11_02/report.json',
    lidar='artifacts/lidar_features_demo_v2_2026_11_03/report.json',
    water='artifacts/water_demo_v2_2026_11_03/report.json',
    camera='artifacts/rellis_test_split_camera_2026_11_04/report.json',
    voxels='artifacts/rellis_lidar_voxels_2026_11_04/report.json',
    labels='artifacts/calibration_outline_2026_11_04/rellis_label_reference/report.json',
    outline='artifacts/calibration_outline_2026_11_04/rellis_control/report.json',
    held_out='artifacts/rellis_tree_coverage_2026_11_06/held_out/report.json',
)
# section, measure, source, path into the report, unit, scale, decimals
ROWS = [
    ('Mapping', 'Source scans', 'run', 'source_scans', 'scans', 1, 0),
    ('Mapping', 'Mapped scans', 'run', 'mapped_scans', 'scans', 1, 0),
    ('Mapping', 'Voxel size', 'run', 'resolution_m', 'm', 1, 2),
    ('Mapping', 'Maximum range', 'run', 'maximum_range_m', 'm', 1, 0),
    ('Mapping', 'Median processing time per scan, offline', 'run', 'processing_seconds_median', 's', 1, 1),
    ('Self-returns', 'Driven path non-traversable before filter, geometry, v1', 'terrain_v1_unfiltered',
     'driven_path.geometric_worst_state_fraction.non_traversable', '%', 100, 1),
    ('Self-returns', 'Driven path non-traversable after filter, geometry, v1', 'terrain_v1_filtered',
     'driven_path.geometric_worst_state_fraction.non_traversable', '%', 100, 1),
    ('Traversability', 'Driven path non-traversable, camera semantics, v2', 'lidar',
     'driven_path_worst_state_camera_semantics.non_traversable', '%', 100, 1),
    ('Traversability', 'Driven path non-traversable, camera + LiDAR, v2', 'lidar',
     'driven_path_worst_state_camera_plus_lidar_compliance.non_traversable', '%', 100, 1),
    ('Pose check', 'Path length, LiDAR odometry', 'gps', 'path_length_m.lidar_odometry', 'm', 1, 1),
    ('Pose check', 'Path length, GPS', 'gps', 'path_length_m.gps', 'm', 1, 1),
    ('Pose check', 'Position RMSE, rigid fit', 'gps', 'ate_rigid_m.rmse', 'm', 1, 2),
    ('Pose check', 'Position RMSE, with antenna offset', 'gps', 'ate_with_lever_arm_m.rmse', 'm', 1, 2),
    ('Pose check', 'Map vertical against gravity', 'gps', 'map_z_vs_gravity_tilt_deg', 'deg', 1, 2),
    ('Pose check', 'Distance error per 10 m, median', 'gps', 'relative_distance_error_per_segment.median', '%', 100, 1),
    ('Map sharpness', 'Ground thickness, single scan', 'sharpness', 'single_scan.median_m', 'cm', 100, 1),
    ('Map sharpness', 'Ground thickness, recorded poses', 'sharpness', 'accumulated_recorded_poses.median_m', 'cm', 100, 1),
    ('Map sharpness', 'Ground thickness, perturbed poses', 'sharpness', 'accumulated_perturbed_poses.median_m', 'cm', 100, 1),
    ('Water', 'Water cells, no LiDAR return', 'water', 'final_water_cells_no_lidar_return', 'cells', 1, 0),
    ('Water', 'Water cells, LiDAR return', 'water', 'final_water_cells_with_lidar_return', 'cells', 1, 0),
    ('RELLIS camera', 'Test images', 'camera', 'images', 'images', 1, 0),
    ('RELLIS camera', 'Pixel accuracy', 'camera', 'pixel_accuracy', '', 1, 2),
    ('RELLIS camera', 'Mean IoU', 'camera', 'mean_iou', '', 1, 2),
    ('RELLIS camera', 'Vegetation IoU', 'camera', 'per_group.vegetation.iou', '', 1, 2),
    ('RELLIS camera', 'Ground IoU', 'camera', 'per_group.ground.iou', '', 1, 2),
    ('RELLIS camera', 'Water IoU', 'camera', 'per_group.water.iou', '', 1, 2),
    ('RELLIS camera', 'Obstacle IoU', 'camera', 'per_group.obstacle.iou', '', 1, 2),
    ('RELLIS LiDAR', 'Scored voxels', 'voxels', 'scored_voxels', 'voxels', 1, 0),
    ('RELLIS LiDAR', 'Voxels with an accepted class', 'voxels', 'consistent_fraction', '%', 100, 1),
    ('RELLIS LiDAR', 'Mean over labelled classes', 'voxels', 'mean_class_consistent_fraction', '%', 100, 1),
    ('RELLIS LiDAR', 'Tree voxels with an accepted class', 'voxels', 'per_rellis_class.tree.consistent_fraction', '%', 100, 1),
    ('RELLIS LiDAR', 'Tree voxels left unknown', 'voxels', 'per_rellis_class.tree.lidar_class_fraction.unknown', '%', 100, 1),
    ('RELLIS LiDAR', 'Bush voxels with an accepted class', 'voxels', 'per_rellis_class.bush.consistent_fraction', '%', 100, 1),
    ('RELLIS sparse fill', 'Held-out voxels with an accepted class, as measured', 'held_out', 'consistent_fraction', '%', 100, 1),
    ('RELLIS sparse fill', 'Held-out voxels with an accepted class, with fill', 'held_out', 'sparse_filled.consistent_fraction', '%', 100, 1),
    ('RELLIS sparse fill', 'Held-out tree voxels accepted, as measured', 'held_out', 'per_rellis_class.tree.consistent_fraction', '%', 100, 1),
    ('RELLIS sparse fill', 'Held-out tree voxels accepted, with fill', 'held_out', 'sparse_filled.per_rellis_class.tree.consistent_fraction', '%', 100, 1),
    ('RELLIS sparse fill', 'Filled voxels accepted', 'held_out', 'filled_voxels_only.consistent_fraction', '%', 100, 1),
    ('Calibration control', 'RELLIS label reference, horizontal offset', 'labels', 'dx_deg', 'deg', 1, 2),
    ('Calibration control', 'RELLIS label reference, vertical offset', 'labels', 'dy_deg', 'deg', 1, 2),
    ('Calibration control', 'Outline method on RELLIS, median offset', 'outline', 'median_dx_deg', 'deg', 1, 2),
    ('Calibration control', 'Outline method, accepted frames', 'outline', 'frames_accepted', 'frames', 1, 0),
]


def lookup(report, path):
    value = report
    for key in path.split('.'):
        value = value[key]
    return value


def style(axis):
    axis.tick_params(colors=MUTED, labelsize=10, length=0)
    axis.grid(axis='x', color=GRID, lw=.7)
    axis.set_axisbelow(True)
    for spine in axis.spines.values():
        spine.set_visible(False)


def camera_figure(report, path):
    names = ['vegetation', 'ground', 'water', 'obstacle']
    values = [report['per_group'][n]['iou'] for n in names]
    figure, axis = plt.subplots(figsize=(7, 2.8), constrained_layout=True, facecolor=SURFACE)
    axis.set_facecolor(SURFACE)
    axis.barh(names, values, color=SERIES[0], height=.55)
    axis.invert_yaxis()
    axis.set_xlim(0, 1)
    for index, value in enumerate(values):
        axis.text(value+.015, index, f'{value:.2f}', va='center', fontsize=10, color=INK)
    axis.set_xlabel('IoU', color=MUTED, fontsize=10)
    axis.set_title(f"Camera classes on RELLIS-3D ({report['images']:,} test images)", loc='left', fontsize=11, color=INK)
    style(axis)
    figure.savefig(path, dpi=200, facecolor=SURFACE)
    plt.close(figure)


def lidar_figure(report, path):
    names = ['grass', 'concrete', 'bush', 'tree', 'person']
    figure, axis = plt.subplots(figsize=(7.5, 3.4), constrained_layout=True, facecolor=SURFACE)
    axis.set_facecolor(SURFACE)
    for index, name in enumerate(names):
        entry = report['per_rellis_class'][name]
        accepted, unknown = entry['consistent_fraction'], entry['lidar_class_fraction']['unknown']
        other = max(1-accepted-unknown, 0)
        left = 0
        for value, colour in zip((accepted, other, unknown), SERIES):
            axis.barh(index, value-.004, left=left, color=colour, height=.58)
            if value >= .09:
                axis.text(left+value/2, index, f'{100*value:.0f}%', ha='center', va='center', fontsize=9, color='white')
            left += value
    axis.set_yticks(range(len(names)), [f"{n} ({report['per_rellis_class'][n]['voxels']:,})" for n in names])
    axis.invert_yaxis()
    axis.set_xlim(0, 1)
    axis.xaxis.set_major_formatter(matplotlib.ticker.PercentFormatter(1, decimals=0))
    axis.set_title('LiDAR-only classes against human labels (voxels in brackets)', loc='left', fontsize=11, color=INK)
    axis.legend([Patch(color=c) for c in SERIES], ['accepted class', 'another class', 'no class'], ncol=3, frameon=False,
                loc='upper center', bbox_to_anchor=(.5, -.1), fontsize=9, labelcolor=MUTED)
    style(axis)
    figure.savefig(path, dpi=200, facecolor=SURFACE)
    plt.close(figure)


def main():
    repo = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    reports = {k: json.loads((repo/v).read_text()) for k, v in SOURCES.items()}
    frames = [json.loads(line) for line in (repo/'artifacts/offline_v2_2026_11_03/frames.jsonl').read_text().splitlines()]
    records = []
    for section, measure, source, path, unit, scale, decimals in ROWS:
        value = lookup(reports[source], path)*scale
        records.append(dict(section=section, measure=measure, value=f'{value:,.{decimals}f}', unit=unit,
                            source=SOURCES[source], key=path))
    final = frames[-1]
    for measure, key in (('Occupied voxels', 'occupied_voxels'), ('Voxels with a confirmed camera class', 'classified_voxels')):
        records.append(dict(section='Mapping', measure=measure, value=f'{final[key]:,}', unit='voxels',
                            source='artifacts/offline_v2_2026_11_03/frames.jsonl', key=f'last line, {key}'))
    box = np.array(reports['run']['vehicle_box_m'])
    counts = np.zeros(6)  # all, within 10 m, within 20 m, and the same three inside the vehicle box
    recording = Recording(Path(reports['run']['bag']))
    for index, cloud in enumerate(recording.messages('/velodyne_1/velodyne_points', PointCloud2)):
        if index % 8:
            continue
        raw = read_points(cloud, field_names=('x', 'y', 'z'))
        points = np.column_stack([raw['x'], raw['y'], raw['z']]).astype(np.float64)
        points = points[np.isfinite(points).all(1)]
        ranges = np.linalg.norm(points, axis=1)
        points, ranges = points[ranges > .1], ranges[ranges > .1]
        inside = ((points >= box[0]) & (points <= box[1])).all(1)
        for slot, near in enumerate((np.ones(len(ranges), bool), ranges <= 10, ranges <= 20)):
            counts[slot] += near.sum()
            counts[slot+3] += (near & inside).sum()
    for slot, name in enumerate(('all returns', 'returns within 10 m', 'returns within 20 m')):
        records.append(dict(section='Self-returns', measure=f'Share of {name} inside the vehicle box', value=f'{100*counts[slot+3]/counts[slot]:.1f}',
                            unit='%', source=reports['run']['bag'], key='measured from every 8th scan; vehicle and LiDAR frames coincide'))
    args.output.mkdir(parents=True)
    with open(args.output/'results.csv', 'w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    lines = ['# Thesis results', '', 'Generated by `tools/build_results_pack.py`. Sources are in `results.csv`.', '']
    for section in dict.fromkeys(r['section'] for r in records):
        lines += [f'## {section}', '', '| Measure | Value |', '|---|---:|']
        lines += [f"| {r['measure']} | {r['value']} {r['unit']} |".replace(' |\n', ' |') for r in records if r['section'] == section]
        lines.append('')
    (args.output/'results.md').write_text('\n'.join(lines))
    camera_figure(reports['camera'], args.output/'camera_iou.png')
    lidar_figure(reports['voxels'], args.output/'lidar_classes.png')
    print(f'{len(records)} results in {args.output}')


if __name__ == '__main__':
    main()
