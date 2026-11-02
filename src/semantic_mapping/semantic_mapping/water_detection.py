"""Camera classes projected onto the mapped ground, for water in LiDAR gaps.

Water returns few or no LiDAR points (absorbed or mirrored away), so camera
water pixels rarely have an occupied voxel to label. Here each sampled pixel ray
is intersected with the ground surface from terrain_analysis; small LiDAR
holes are filled from surrounding ground height. A ray that first enters an
above-ground occupied voxel is occluded and discarded. Evidence accumulates per
ground column; a column with camera water and no LiDAR return is the strongest
water cue. The ground height in filled holes is an interpolation, not a
measurement.
"""

import numpy as np

from semantic_mapping.lidar_features import encode
from semantic_mapping.semantic_fusion import CLASS_NAMES, voxel_keys

WATER = CLASS_NAMES.index('water')
WATER_STATES = ('none', 'water_no_lidar_return', 'water_with_lidar_return')
WATER_RGB = np.array([[0, 0, 0], [30, 120, 255], [130, 200, 255]], dtype=np.uint32)


def fill_ground(ground, iterations):
    """Fill NaN cells from the mean of finite 3x3 neighbours, one ring per iteration."""
    filled = np.array(ground, dtype=np.float64)
    for _ in range(iterations):
        missing = ~np.isfinite(filled)
        if not missing.any():
            break
        padded = np.pad(filled, 1, constant_values=np.nan)
        windows = np.lib.stride_tricks.sliding_window_view(padded, (3, 3))
        finite = np.isfinite(windows)
        count = finite.sum(axis=(-2, -1))
        total = np.where(finite, windows, 0.).sum(axis=(-2, -1))
        grow = missing & (count > 0)
        if not grow.any():
            break
        filled[grow] = total[grow]/count[grow]
    return filled


def pixel_grid(width, height, stride):
    u, v = np.meshgrid(np.arange(stride/2, width, stride), np.arange(stride/2, height, stride))
    return u.ravel(), v.ravel()


def camera_directions(intrinsic, u, v):
    """Unit rays in the optical frame (x right, y down, z forward)."""
    rays = np.linalg.solve(intrinsic, np.vstack([u, v, np.ones_like(u)])).T
    return rays/np.linalg.norm(rays, axis=1, keepdims=True)


def ground_hits(origin, directions, ground, grid_origin, cell, above_codes, max_range=10., step=.1):
    """First ground intersection of each world ray; None-like (-1) where missed or occluded.

    Returns grid indices (n, 2) of the hit column (-1 rows for no hit) and the hit points.
    """
    origin = np.asarray(origin, dtype=np.float64)
    t = np.arange(step, max_range+step/2, step)
    samples = origin+directions[:, None, :]*t[None, :, None]  # rays x samples x 3
    flat = samples.reshape(-1, 3)
    index = voxel_keys(flat[:, :2], cell)-grid_origin
    inside = ((index >= 0) & (index < ground.shape)).all(1)
    level = np.full(len(flat), np.nan)
    level[inside] = ground[index[inside, 0], index[inside, 1]]
    below = (flat[:, 2] <= level).reshape(samples.shape[:2])
    occluded = np.zeros(len(flat), dtype=bool)
    if len(above_codes):
        codes = encode(voxel_keys(flat, cell))
        position = np.clip(np.searchsorted(above_codes, codes), 0, len(above_codes)-1)
        occluded = above_codes[position] == codes
    occluded = occluded.reshape(samples.shape[:2])
    stop = below | occluded
    first = stop.argmax(axis=1)
    hit = stop.any(axis=1) & below[np.arange(len(first)), first] & ~occluded[np.arange(len(first)), first]
    points = samples[np.arange(len(first)), first]
    cells = np.full((len(first), 2), -1, dtype=np.int64)
    cells[hit] = (voxel_keys(points[hit, :2], cell)-grid_origin)
    return cells, points, hit


class GroundEvidence:
    """Per ground column: camera pixel votes per class, frames seen, mean height."""

    def __init__(self, classes=len(CLASS_NAMES)):
        self.cells = {}
        self.classes = classes

    def add(self, keys, labels, heights, frame):
        """keys: global 2-D column keys of hits; labels: their camera group IDs."""
        if not len(keys):
            return
        unique, inverse = np.unique(keys, axis=0, return_inverse=True)
        votes = np.zeros((len(unique), self.classes), dtype=np.int64)
        np.add.at(votes, (inverse, labels), 1)
        height = np.zeros(len(unique))
        np.add.at(height, inverse, heights)
        count = np.bincount(inverse, minlength=len(unique))
        for key, vote, z, n in zip(map(tuple, unique), votes, height, count):
            entry = self.cells.setdefault(key, [np.zeros(self.classes, dtype=np.int64), 0, 0., 0, -1])
            entry[0] += vote
            entry[2] += z
            entry[3] += n
            if entry[4] != frame:
                entry[1] += 1
                entry[4] = frame

    def water(self, min_frames=3, min_pixels=5, min_fraction=.5):
        """Columns where labelled camera pixels are mostly water."""
        keys, fraction, frames, height = [], [], [], []
        for key, (votes, seen, z, n, _) in self.cells.items():
            labelled = votes[1:].sum()
            if seen < min_frames or labelled < min_pixels:
                continue
            share = votes[WATER]/labelled
            if share >= min_fraction:
                keys.append(key); fraction.append(share); frames.append(seen); height.append(z/n)
        return (np.array(keys, dtype=np.int64).reshape(-1, 2), np.array(fraction), np.array(frames),
                np.array(height))

    def majority(self, min_frames=3, min_pixels=5):
        """Most frequent labelled camera class per sufficiently observed column."""
        keys, labels, height = [], [], []
        for key, (votes, seen, z, n, _) in self.cells.items():
            if seen < min_frames or votes[1:].sum() < min_pixels:
                continue
            keys.append(key); labels.append(1+int(votes[1:].argmax())); height.append(z/n)
        return np.array(keys, dtype=np.int64).reshape(-1, 2), np.array(labels, dtype=np.uint8), np.array(height)
