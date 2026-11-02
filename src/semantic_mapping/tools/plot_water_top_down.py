#!/usr/bin/env python3
"""Top-down figure of a water demo: camera classes on ground, water cells, path.

Reads the final messages of <input>/replay and the run it was built from;
writes <input>/water_top_down.png and prints the water-to-path distance.
"""
import argparse
import json
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib.patches import Patch
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points

from semantic_mapping.semantic_fusion import CLASS_NAMES, PALETTE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True, help='water demo folder')
    args = parser.parse_args()
    output = args.input/'water_top_down.png'
    if output.exists():
        parser.error(f'{output} exists; move it first to preserve it')
    db = sqlite3.connect(next((args.input/'replay').glob('*.db3')).resolve().as_uri()+'?mode=ro', uri=True)
    topics = dict(db.execute('SELECT name,id FROM topics'))
    last = lambda name: read_points(deserialize_message(db.execute(
        'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp DESC LIMIT 1', (topics[name],)).fetchone()[0],
        PointCloud2))
    water, ground = last('/water/cells'), last('/water/camera_ground')
    run = Path(json.loads((args.input/'report.json').read_text())['run'])
    final = json.loads((run/'frames.jsonl').read_text().splitlines()[-1])['frame']
    path = np.load(run/'frames'/final/'voxels.npz')['trajectory']
    distance = np.min(np.hypot(water['x'][:, None]-path[None, :, 0], water['y'][:, None]-path[None, :, 1]), axis=1)
    print(f'water cells {len(distance)}; distance to driven path: median {np.median(distance):.2f} m, '
          f'90% within {np.percentile(distance, 90):.2f} m, {100*np.mean(distance < 2):.0f}% within 2 m')

    fig, ax = plt.subplots(figsize=(8, 9), constrained_layout=True)
    labels = ground['camera_label'].astype(int)
    ax.scatter(ground['x'], ground['y'], s=2, c=PALETTE[labels]/255., marker='s', linewidths=0)
    for state, colour, name in [(0, '#1e78ff', 'water, no LiDAR return'), (1, '#82c8ff', 'water, LiDAR return')]:
        mask = water['lidar_return'] == state
        ax.scatter(water['x'][mask], water['y'][mask], s=14, c=colour, marker='s',
                   edgecolors='#0b0b0b', linewidths=.3, label=name)
    ax.plot(path[:, 0], path[:, 1], color='#0b0b0b', lw=1.2, label='driven path')
    ax.set_aspect('equal')
    ax.set_title('Camera classes projected on mapped ground, and water cells', loc='left', fontsize=10)
    ax.set_xlabel('x in sensor_init (m)', color='#898781', fontsize=8)
    ax.set_ylabel('y (m)', color='#898781', fontsize=8)
    ax.tick_params(colors='#898781', labelsize=7)
    handles, names = ax.get_legend_handles_labels()
    handles += [Patch(color=PALETTE[i]/255.) for i in (1, 2, 3, 5)]
    names += [f'camera: {CLASS_NAMES[i]}' for i in (1, 2, 3, 5)]
    ax.legend(handles, names, fontsize=7, frameon=False, loc='upper right')
    fig.savefig(output, dpi=160, facecolor='#fcfcfb')


if __name__ == '__main__':
    main()
