#!/usr/bin/env python3
"""Terrain geometry and traversability states from a saved offline map.

Reads an offline_demo run folder without changing it. Writes a new folder with
terrain.npz, report.json and figures. Vehicle limits come from a YAML file whose
default values are placeholders, not POLARIS specifications.
"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap, LinearSegmentedColormap
from matplotlib.patches import Patch
import numpy as np
import yaml

from semantic_mapping.semantic_fusion import CLASS_NAMES
from semantic_mapping.terrain_analysis import (
    STATES, TerrainSettings, analyze, path_states, vehicle_consistency)

# Status colours carry state with a text legend; unobserved uses the surface.
STATE_COLORS = ['#fcfcfb', '#c3c2b7', '#0ca30c', '#fab219', '#d03b3b']
SEQUENTIAL = LinearSegmentedColormap.from_list('blue', ['#cde2fb', '#86b6ef', '#3987e5', '#1c5cab', '#0d366b'])
INK, MUTED = '#0b0b0b', '#898781'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fractions(states):
    return {name: float(np.mean(states == i)) for i, name in enumerate(STATES)}


def summary(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return dict(count=int(len(values)), median=float(np.median(values)),
                p05=float(np.percentile(values, 5)), p95=float(np.percentile(values, 95)),
                min=float(values.min()), max=float(values.max()))


def style(ax, title):
    ax.set_title(title, loc='left', fontsize=10, color=INK)
    ax.set_xlabel('x in sensor_init (m)', color=MUTED, fontsize=8)
    ax.set_ylabel('y (m)', color=MUTED, fontsize=8)
    ax.tick_params(colors=MUTED, labelsize=7)
    for spine in ax.spines.values():
        spine.set_color('#c3c2b7')
    ax.set_aspect('equal')


def draw_path(ax, positions):
    ax.plot(positions[:, 0], positions[:, 1], color=INK, lw=1.2)
    ax.plot(*positions[0, :2], 'o', ms=5, color=INK)
    ax.annotate('start', positions[0, :2], textcoords='offset points', xytext=(6, 4), fontsize=7, color=INK)


def state_figure(terrain, positions, verified):
    fig, axes = plt.subplots(1, 2, figsize=(12, 6), constrained_layout=True)
    cmap = ListedColormap(STATE_COLORS)
    for ax, state, title in [(axes[0], terrain.geometric_state, 'Geometry only'),
                             (axes[1], terrain.combined_state, 'Geometry + confirmed semantics')]:
        ax.imshow(state.T, origin='lower', extent=terrain.extent(), cmap=cmap,
                  vmin=-.5, vmax=len(STATES)-.5, interpolation='nearest')
        draw_path(ax, positions)
        style(ax, title)
    labels = ['unobserved', 'occupied, no ground reference', 'traversable', 'caution', 'non-traversable']
    fig.legend([Patch(facecolor=c, edgecolor='#c3c2b7') for c in STATE_COLORS], labels,
               loc='lower center', ncol=5, frameon=False, fontsize=8, bbox_to_anchor=(.5, -.06))
    note = ('Vehicle limits are placeholders, not POLARIS specifications. ' if not verified else '') + \
        'Black line: recorded driven path (TartanDrive vehicle).'
    fig.suptitle(note, fontsize=8, color=MUTED, x=.01, ha='left')
    return fig


def descriptor_figure(terrain, positions):
    fig, axes = plt.subplots(1, 2, figsize=(12, 6), constrained_layout=True)
    for ax, values, title, unit, top in [
            (axes[0], terrain.slope_deg, 'Local ground slope', 'degrees', 35.),
            (axes[1], terrain.obstacle_height_m, 'Tallest voxel above ground (below overhang)', 'metres',
             terrain.settings.overhang_clearance_m)]:
        image = ax.imshow(values.T, origin='lower', extent=terrain.extent(), cmap=SEQUENTIAL,
                          vmin=0, vmax=top, interpolation='nearest')
        draw_path(ax, positions)
        style(ax, title)
        bar = fig.colorbar(image, ax=ax, shrink=.7)
        bar.set_label(unit, color=MUTED, fontsize=8)
        bar.ax.tick_params(colors=MUTED, labelsize=7)
    fig.suptitle('Blank cells: unobserved or no ground plane. Heights quantized to 0.35 m voxel layers.',
                 fontsize=8, color=MUTED, x=.01, ha='left')
    return fig


def consistency_figure(elapsed, heights, angles):
    fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True, constrained_layout=True)
    for ax, values, title in [(axes[0], heights, 'Vehicle origin height above mapped ground (m)'),
                              (axes[1], angles, 'Vehicle up-axis vs. ground normal (degrees)')]:
        ax.plot(elapsed, values, color='#2a78d6', lw=2)
        ax.set_title(title, loc='left', fontsize=10, color=INK)
        ax.grid(color='#e1e0d9', lw=.6)
        ax.tick_params(colors=MUTED, labelsize=7)
        for spine in ax.spines.values():
            spine.set_color('#c3c2b7')
    axes[1].set_xlabel('recorded time since first mapped scan (s)', color=MUTED, fontsize=8)
    return fig


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True, help='offline_demo run folder')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=package/'config/traversability_placeholder.yaml')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder to preserve previous results')
    run = json.loads((args.input/'report.json').read_text())
    frames = [json.loads(line) for line in (args.input/'frames.jsonl').read_text().splitlines()]
    if not frames:
        parser.error('Run has no saved frames')
    config = yaml.safe_load(args.config.read_text())
    settings = TerrainSettings.from_config(config)
    if abs(settings.cell_m-run['resolution_m']) > 1e-9:
        parser.error('Terrain cell size must equal the mapping resolution')
    footprint = float(config['vehicle']['footprint_radius_m'])
    verified = bool(config.get('vehicle_parameters_verified', False))

    with np.load(args.input/'frames'/frames[-1]['frame']/'voxels.npz') as final:
        centres, labels = final['centres'], final['labels']
        observations = final['observations']
    poses, elapsed = [], []
    for frame in frames:
        with np.load(args.input/'frames'/frame['frame']/'voxels.npz') as saved:
            poses.append(saved['world_vehicle'])
        elapsed.append(frame['elapsed_recording_seconds'])
    poses, elapsed = np.array(poses), np.array(elapsed)
    positions = poses[:, :3, 3]

    terrain = analyze(centres, labels, settings)
    geometric_path = path_states(terrain, positions, footprint, combined=False)
    combined_path = path_states(terrain, positions, footprint)
    heights, angles = vehicle_consistency(terrain, poses, footprint)
    observed = terrain.observed

    agreement = {}
    for i, name in enumerate(CLASS_NAMES[1:], 1):
        h = terrain.voxel_height_m[labels == i]
        h = h[np.isfinite(h)]
        if len(h):
            agreement[name] = dict(voxels=int(len(h)), median_height_above_ground_m=float(np.median(h)),
                                   fraction_in_ground_layer=float(np.mean(h <= .5*settings.cell_m)))
    report = dict(
        source_run=str(args.input.resolve()), source_report_sha256=sha(args.input/'report.json'),
        source_final_frame=frames[-1]['frame'], config=str(args.config.resolve()), config_sha256=sha(args.config),
        source_full_recording_processed=bool(run.get('full_recording_processed')),
        vehicle_parameters_verified=verified, settings={k: v for k, v in vars(settings).items()},
        footprint_radius_m=footprint,
        assumptions=['map Z treated as gravity-up (not independently verified)',
                     'ground = lowest occupied voxel per column unless far above the local minimum',
                     'semantics change a column only with confirmed labels (>=2 observations, >=0.6 confidence)',
                     'confirmed vegetation below compliant height is assumed drivable at reduced cost'],
        resolution_limits=dict(height_quantum_m=settings.cell_m,
                               slope_error_deg='median within ~2, single windows up to ~5.5 (synthetic test)',
                               roughness_floor_m=round(settings.cell_m/np.sqrt(12), 3)),
        grid_shape=list(observed.shape), observed_columns=int(observed.sum()),
        ground_measured_columns=int(terrain.ground_measured.sum()),
        ground_interpolated_columns=int((np.isfinite(terrain.ground_z) & ~terrain.ground_measured).sum()),
        geometric_state_fraction_of_observed=fractions(terrain.geometric_state[observed]),
        combined_state_fraction_of_observed=fractions(terrain.combined_state[observed]),
        semantic_effects=dict(rigid_columns=int(terrain.rigid.sum()), hazard_columns=int(terrain.hazard.sum()),
                              vegetation_relaxed_columns=int(terrain.compliant.sum())),
        slope_deg=summary(terrain.slope_deg), roughness_m=summary(terrain.roughness_m),
        obstacle_height_m=summary(terrain.obstacle_height_m[observed]),
        driven_path=dict(
            poses=len(positions),
            note='The recording vehicle drove these cells, so they are positive (traversable) examples for '
                 'that vehicle only. This measures false alarms, not hazard detection.',
            geometric_worst_state_fraction=fractions(geometric_path),
            combined_worst_state_fraction=fractions(combined_path)),
        vehicle_consistency=dict(
            note='Internal consistency of map and poses; not absolute accuracy.',
            origin_height_above_ground_m=summary(heights), tilt_vs_ground_normal_deg=summary(angles)),
        confirmed_label_height_agreement=agreement,
        confirmed_labels_with_observations=int(((labels > 0) & (observations >= 2)).sum()),
        navigation_validated=False)
    args.output.mkdir(parents=True)
    np.savez_compressed(args.output/'terrain.npz', origin=terrain.origin, cell_m=settings.cell_m,
        observed=observed, ground_measured=terrain.ground_measured, ground_z=terrain.ground_z,
        gradient=terrain.gradient, slope_deg=terrain.slope_deg, roughness_m=terrain.roughness_m,
        obstacle_height_m=terrain.obstacle_height_m, rigid=terrain.rigid, hazard=terrain.hazard,
        compliant=terrain.compliant, geometric_cost=terrain.geometric_cost,
        combined_cost=terrain.combined_cost, geometric_state=terrain.geometric_state,
        combined_state=terrain.combined_state, voxel_height_m=terrain.voxel_height_m,
        path_geometric_state=geometric_path, path_combined_state=combined_path,
        vehicle_height_above_ground_m=heights, vehicle_tilt_vs_ground_deg=angles,
        elapsed_s=elapsed, states=np.array(STATES))
    for name, fig in [('traversability_states.png', state_figure(terrain, positions, verified)),
                      ('terrain_descriptors.png', descriptor_figure(terrain, positions)),
                      ('vehicle_consistency.png', consistency_figure(elapsed, heights, angles))]:
        fig.savefig(args.output/name, dpi=160, facecolor='#fcfcfb', bbox_inches='tight')
        plt.close(fig)
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
