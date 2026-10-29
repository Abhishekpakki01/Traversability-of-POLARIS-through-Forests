"""Occupied-surface semantic evidence, independent of OHM integration."""

from dataclasses import dataclass

import numpy as np


CLASS_NAMES = ('unknown', 'ground', 'vegetation', 'trunk', 'water', 'obstacle')
PALETTE = np.array([[150, 150, 150], [139, 90, 43], [45, 180, 45],
                    [100, 55, 20], [30, 120, 255], [230, 50, 50]], dtype=np.uint8)


def voxel_keys(points, resolution):
    return np.floor(np.asarray(points, dtype=np.float64)/resolution).astype(np.int64)


def first_occupied(origin, endpoint, occupied, resolution):
    """Amanatides-Woo traversal; stop at the first occupied cell on the segment.

    Unknown/free cells are traversed but never labeled. Include the camera's
    starting cell: an occupied camera/ego cell must conservatively block the ray.
    Crossing an exact edge advances tied axes together (zero-volume touches
    do not create a surface).
    """
    origin, endpoint = np.asarray(origin, dtype=float), np.asarray(endpoint, dtype=float)
    cell = voxel_keys(origin, resolution)
    target = voxel_keys(endpoint, resolution)
    direction = endpoint-origin
    step = np.sign(direction).astype(np.int64)
    t_max, t_delta = np.full(3, np.inf), np.full(3, np.inf)
    active = direction != 0
    boundary = (cell + (step > 0))*resolution
    t_max[active] = (boundary[active]-origin[active])/direction[active]
    t_delta[active] = resolution/np.abs(direction[active])
    limit = int(np.abs(target-cell).sum())+4
    cell, target, step = cell.tolist(), target.tolist(), step.tolist()
    t_max, t_delta = t_max.tolist(), t_delta.tolist()
    for _ in range(limit):
        key = tuple(cell)
        if key in occupied:
            return key
        if cell == target:
            return None
        crossing = min(t_max)
        if crossing > 1+1e-10:
            return None
        for axis in range(3):
            if abs(t_max[axis]-crossing) <= 1e-12:
                cell[axis] += step[axis]
                t_max[axis] += t_delta[axis]
    return None


def surface_observations(world_points, probabilities, camera_origin, occupied, resolution,
                         strict_voxel_occlusion=True):
    """One mean class distribution per visible measured endpoint voxel/frame.

    Inputs must already pass camera-space measured-depth visibility filtering.
    Optional solid-cube traversal is stricter than the measured surface and can
    reject grazing ground rays because a coarse occupied cube protrudes above
    its actual LiDAR surface. Neither mode labels free/unknown endpoint cells.
    """
    world_points = np.asarray(world_points, dtype=np.float64).reshape(-1, 3)
    probabilities = np.asarray(probabilities, dtype=np.float64).reshape(-1, 6)
    if len(world_points) != len(probabilities):
        raise ValueError('Point/probability counts differ')
    if not np.isfinite(world_points).all() or not np.isfinite(probabilities).all():
        raise ValueError('Nonfinite semantic observations')
    if np.any(probabilities < 0) or not np.allclose(probabilities.sum(1), 1, atol=1e-4):
        raise ValueError('Expected normalized class probabilities')
    keys = voxel_keys(world_points, resolution)
    groups = {}
    stats = dict(samples=len(keys), missing_surface=0, occluded=0, visible_voxels=0)
    # Aggregate only rays that reach their own measured occupied surface.
    for point, key, probability in zip(world_points, keys, probabilities):
        key = tuple(key)
        if key not in occupied:
            stats['missing_surface'] += 1
            continue
        if strict_voxel_occlusion and first_occupied(camera_origin, point, occupied, resolution) != key:
            stats['occluded'] += 1
            continue
        groups.setdefault(key, []).append(probability)
    means = {key: np.mean(values, axis=0) for key, values in groups.items()}
    stats['visible_voxels'] = len(means)
    return means, stats


@dataclass
class Evidence:
    counts: np.ndarray
    observations: int
    last_stamp: int


class SemanticEvidence:
    """Bounded Dirichlet pseudo-counts; at most one update per voxel/image."""
    def __init__(self, min_observations=2, min_confidence=0.6, prior=0.05):
        if min_observations < 1 or not 0 < min_confidence <= 1 or prior <= 0:
            raise ValueError('Invalid fusion settings')
        self.min_observations, self.min_confidence, self.prior = (
            min_observations, min_confidence, prior)
        self.voxels = {}

    def retain_occupied(self, occupied):
        self.voxels = {key: value for key, value in self.voxels.items() if key in occupied}

    def update(self, observations, stamp, occupied):
        updated = 0
        for key, probability in observations.items():
            if key not in occupied:
                continue
            value = self.voxels.get(key)
            if value is not None and stamp <= value.last_stamp:
                continue
            if value is None:
                value = Evidence(np.full(6, self.prior), 0, -1)
                self.voxels[key] = value
            value.counts += probability
            value.observations += 1
            value.last_stamp = stamp
            # Limit confidence inertia during a long drive while retaining a prior.
            evidence = value.counts-self.prior
            if evidence.sum() > 100:
                value.counts = self.prior+evidence*(100/evidence.sum())
            updated += 1
        return updated

    def query(self, keys):
        labels = np.zeros(len(keys), dtype=np.uint8)
        confidence = np.zeros(len(keys), dtype=np.float32)
        observations = np.zeros(len(keys), dtype=np.uint32)
        for index, key in enumerate(keys):
            value = self.voxels.get(tuple(key))
            if value is None:
                continue
            posterior = value.counts/value.counts.sum()
            label = int(posterior.argmax())
            confidence[index] = posterior[label]
            observations[index] = value.observations
            if value.observations >= self.min_observations and posterior[label] >= self.min_confidence:
                labels[index] = label
        return labels, confidence, observations
