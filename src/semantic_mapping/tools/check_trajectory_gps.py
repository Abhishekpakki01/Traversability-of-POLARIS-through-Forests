#!/usr/bin/env python3
"""Compare the mapping poses (recorded LiDAR odometry) with independent GPS/INS.

The map is only as accurate as its poses. TartanDrive's NovAtel GPS/INS
odometry (gps_odom.npy: x, y, z, qx, qy, qz, qw, ...; metric, z = altitude) is
independent of the LiDAR odometry. After a rigid best fit (no scale), this
reports:
  - absolute trajectory error (ATE), with and without an estimated antenna
    lever arm (the GPS reference point is not the vehicle frame origin);
  - path length and segment drift (scale/drift of LiDAR odometry);
  - the tilt between the map's Z axis and gravity (GPS up), which tests the
    "map Z is gravity-up" assumption used for heights and slopes.
GPS has its own error (RTK status is not in this sample), so this bounds
agreement rather than measuring truth.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import numpy as np

SAMPLE = Path.home()/'thesis_data/tartan_drive_2_sample'


def rigid_fit(source, target):
    """Rotation R and translation t minimising |R source + t - target| (Kabsch)."""
    ms, mt = source.mean(0), target.mean(0)
    u, _, vt = np.linalg.svd((source-ms).T @ (target-mt))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rotation = vt.T @ np.diag([1, 1, d]) @ u.T
    return rotation, mt-rotation @ ms


def fit_with_lever_arm(positions, rotations, target, iterations=2000, tolerance=1e-7):
    """Jointly fit the frame alignment and a body-fixed offset to the GPS point
    (alternating least squares until the offset stops changing)."""
    arm = np.zeros(3)
    for _ in range(iterations):
        rotation, translation = rigid_fit(positions+rotations @ arm, target)
        residual = target-(positions @ rotation.T+translation)
        # target = R (p + R_body arm) + t  ->  R R_body arm = residual
        a = np.einsum('ij,njk->nik', rotation, rotations).reshape(-1, 3)
        previous, arm = arm, np.linalg.lstsq(a, residual.reshape(-1), rcond=None)[0]
        if np.abs(arm-previous).max() < tolerance:
            break
    rotation, translation = rigid_fit(positions+rotations @ arm, target)
    return rotation, translation, arm


def summary(errors):
    return dict(rmse=float(np.sqrt(np.mean(errors**2))), median=float(np.median(errors)),
                p95=float(np.percentile(errors, 95)), max=float(errors.max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='offline run folder (poses per frame)')
    parser.add_argument('--gps', type=Path, default=SAMPLE/'gps_odom.npy')
    parser.add_argument('--gps-stamps', type=Path, default=SAMPLE/'gps_odom_timestamps.txt')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--segment-m', type=float, default=10., help='segment length for relative drift')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new folder')
    frames = [json.loads(line) for line in (args.run/'frames.jsonl').read_text().splitlines()]
    stamps = np.array([f['lidar_stamp_ns'] for f in frames])/1e9
    poses = []
    for frame in frames:
        with np.load(args.run/'frames'/frame['frame']/'voxels.npz') as saved:
            poses.append(saved['world_vehicle'])
    poses = np.array(poses)
    gps = np.load(args.gps)
    gps_t = np.loadtxt(args.gps_stamps)
    if len(gps) != len(gps_t) or not (gps_t[0] <= stamps[0] and stamps[-1] <= gps_t[-1]):
        raise ValueError('GPS does not cover the mapped interval')
    origin = gps[0, :3]
    target = np.column_stack([np.interp(stamps, gps_t, gps[:, i]-origin[i]) for i in range(3)])
    nearest = np.min(np.abs(gps_t[None, :]-stamps[:, None]), axis=1) if len(stamps) < 5000 else None

    positions, rotations = poses[:, :3, 3], poses[:, :3, :3]
    rotation, translation = rigid_fit(positions, target)
    plain = np.linalg.norm(positions @ rotation.T+translation-target, axis=1)
    rotation_l, translation_l, arm = fit_with_lever_arm(positions, rotations, target)
    aligned = (positions+np.einsum('nij,j->ni', rotations, arm)) @ rotation_l.T+translation_l
    lever = np.linalg.norm(aligned-target, axis=1)
    tilt = np.degrees(np.arccos(np.clip((rotation_l @ [0, 0, 1])[2], -1, 1)))

    length = lambda p: float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
    travelled = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(target, axis=0), axis=1))])
    drift = []
    for i in range(len(target)):
        j = np.searchsorted(travelled, travelled[i]+args.segment_m)
        if j >= len(target):
            break
        drift.append(abs(np.linalg.norm(aligned[j]-aligned[i])-np.linalg.norm(target[j]-target[i]))/args.segment_m)
    report = dict(
        run=str(args.run.resolve()), gps=str(args.gps.resolve()), poses=len(positions),
        mapped_seconds=float(stamps[-1]-stamps[0]),
        max_gps_sample_gap_s=float(nearest.max()) if nearest is not None else None,
        path_length_m=dict(lidar_odometry=length(positions), gps=length(target)),
        height_change_m=dict(lidar_odometry=float(positions[-1, 2]-positions[0, 2]), gps=float(target[-1, 2]-target[0, 2])),
        ate_rigid_m=summary(plain), ate_with_lever_arm_m=summary(lever),
        estimated_lever_arm_vehicle_frame_m=arm.tolist(),
        map_z_vs_gravity_tilt_deg=float(tilt),
        relative_distance_error_per_segment=dict(segment_m=args.segment_m, median=float(np.median(drift)),
                                                 p95=float(np.percentile(drift, 95))) if drift else None,
        note='GPS/INS is an independent reference with its own error; agreement, not ground truth.')
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5), constrained_layout=True,
                             gridspec_kw=dict(width_ratios=[1, 1.4]))
    axes[0].plot(target[:, 0], target[:, 1], color='#0b0b0b', lw=2, label='GPS/INS (independent)')
    axes[0].plot(aligned[:, 0], aligned[:, 1], color='#2a78d6', lw=1.5, ls='--', label='LiDAR odometry (map poses), aligned')
    axes[0].set_aspect('equal'); axes[0].legend(fontsize=8, frameon=False)
    axes[0].set_title('Trajectories, top view (m)', loc='left', fontsize=10)
    axes[1].plot(stamps-stamps[0], lever, color='#2a78d6', lw=2)
    axes[1].set_title(f'Position difference after alignment (RMSE {report["ate_with_lever_arm_m"]["rmse"]:.2f} m)',
                      loc='left', fontsize=10)
    axes[1].set_xlabel('recorded time (s)', color='#898781', fontsize=8); axes[1].set_ylabel('m', color='#898781', fontsize=8)
    for ax in axes:
        ax.tick_params(colors='#898781', labelsize=7); ax.grid(color='#e1e0d9', lw=.6)
        for spine in ax.spines.values():
            spine.set_color('#c3c2b7')
    fig.savefig(args.output/'trajectory_vs_gps.png', dpi=160, facecolor='#fcfcfb')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
