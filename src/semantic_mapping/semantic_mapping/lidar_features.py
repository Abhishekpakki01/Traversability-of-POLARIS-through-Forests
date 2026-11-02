"""LiDAR-only voxel features: shape, permeability and reflectance.

Each voxel keeps the moments (count, sum, outer products) of the measured
points that fell into it, a reflectance sum, and a pass-through count. A ray
counts as passing through a voxel only if it travels close to that voxel's
measured point distribution (Mahalanobis distance) and ends beyond it, similar
in spirit to OHM's NDT traversability mode: grazing a voxel corner above a
thin ground plane does not count, a ray threading foliage does.

Shape uses the covariance eigenvalues l1 >= l2 >= l3 at two scales: the voxel
itself and its 3x3x3 neighbourhood. Intensity is raw sensor reflectance and is
not range/incidence calibrated.
"""

from dataclasses import dataclass

import numpy as np

from semantic_mapping.semantic_fusion import voxel_keys

FIELD = 21
BIAS = 1 << (FIELD-1)
LIDAR_CLASSES = ('unknown', 'ground', 'low_vegetation', 'tall_vegetation', 'trunk_like', 'solid_obstacle')
LIDAR_PALETTE = np.array([[150, 150, 150], [139, 90, 43], [154, 205, 50],
                          [30, 123, 40], [144, 133, 233], [227, 73, 72]], dtype=np.uint8)
NEIGHBOURS = np.stack(np.meshgrid(*[np.arange(-1, 2)]*3, indexing='ij'), -1).reshape(-1, 3)


def encode(keys):
    """Pack integer voxel keys into sortable int64 codes; addition stays exact."""
    keys = np.asarray(keys, dtype=np.int64).reshape(-1, 3)
    if np.any(np.abs(keys) >= BIAS):
        raise ValueError('Voxel key outside the encodable range')
    return ((keys[:, 0]+BIAS) << (2*FIELD)) + ((keys[:, 1]+BIAS) << FIELD) + (keys[:, 2]+BIAS)


def shape_features(n, total, outer, min_points):
    """Eigenvalue shape descriptors; NaN where fewer than min_points points."""
    n = np.asarray(n, dtype=np.float64)
    result = {name: np.full(len(n), np.nan) for name in
              ('linearity', 'planarity', 'scattering', 'normal_verticality', 'principal_verticality')}
    valid = n >= min_points
    if not valid.any():
        return result
    mean = total[valid]/n[valid, None]
    cov = outer[valid]/n[valid, None, None]-mean[:, :, None]*mean[:, None, :]
    values, vectors = np.linalg.eigh(cov)  # ascending
    l3, l2, l1 = np.clip(values, 0, None).T
    l1 = np.where(l1 > 0, l1, np.nan)
    result['linearity'][valid] = (l1-l2)/l1
    result['planarity'][valid] = (l2-l3)/l1
    result['scattering'][valid] = l3/l1
    result['normal_verticality'][valid] = np.abs(vectors[:, 2, 0])
    result['principal_verticality'][valid] = np.abs(vectors[:, 2, 2])
    return result


class VoxelStatistics:
    """Incremental per-voxel point moments, reflectance and pass-throughs."""

    def __init__(self, resolution, min_points=4, regularization_m=.05, pass_distance=3.):
        if resolution <= 0 or min_points < 3 or regularization_m <= 0 or pass_distance <= 0:
            raise ValueError('Invalid LiDAR feature settings')
        self.resolution, self.min_points = resolution, min_points
        self.regularization, self.pass_d2 = regularization_m**2, pass_distance**2
        self.codes = np.empty(0, dtype=np.int64)
        self.n = np.empty(0)
        self.total = np.empty((0, 3))
        self.outer = np.empty((0, 3, 3))
        self.intensity = np.empty(0)
        self.passes = np.empty(0)

    def _insert(self, codes):
        new = np.setdiff1d(codes, self.codes)
        if not len(new):
            return
        order = np.argsort(np.concatenate([self.codes, new]), kind='stable')
        grow = lambda a: np.concatenate([a, np.zeros((len(new),)+a.shape[1:])])[order]
        self.codes = np.concatenate([self.codes, new])[order]
        self.n, self.total, self.outer = grow(self.n), grow(self.total), grow(self.outer)
        self.intensity, self.passes = grow(self.intensity), grow(self.passes)

    def lookup(self, codes):
        """Index of each code and whether it is stored."""
        if not len(self.codes):
            return np.zeros(len(codes), dtype=int), np.zeros(len(codes), dtype=bool)
        index = np.clip(np.searchsorted(self.codes, codes), 0, len(self.codes)-1)
        return index, self.codes[index] == codes

    def add_points(self, points, intensity):
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        codes = encode(voxel_keys(points, self.resolution))
        self._insert(np.unique(codes))
        index, _ = self.lookup(codes)
        np.add.at(self.n, index, 1)
        np.add.at(self.total, index, points)
        np.add.at(self.outer, index, points[:, :, None]*points[:, None, :])
        np.add.at(self.intensity, index, np.asarray(intensity, dtype=np.float64))

    def add_rays(self, origin, endpoints):
        """Count, once per ray and voxel, rays passing near a stored distribution."""
        origin = np.asarray(origin, dtype=np.float64).reshape(3)
        endpoints = np.asarray(endpoints, dtype=np.float64).reshape(-1, 3)
        delta = endpoints-origin
        length = np.linalg.norm(delta, axis=1)
        keep = length > 1e-6
        delta, length, endpoints = delta[keep], length[keep], endpoints[keep]
        if not len(length):
            return 0
        direction = delta/length[:, None]
        # Quarter-voxel samples find the traversed voxels; the distance test
        # below is analytic, so sampling only affects corner-clipping cells.
        step = self.resolution/4
        counts = np.ceil(length/step).astype(np.int64)
        ray = np.repeat(np.arange(len(length)), counts)
        offset = np.arange(len(ray))-np.repeat(np.cumsum(counts)-counts, counts)
        samples = origin+direction[ray]*(offset*step)[:, None]
        codes = encode(voxel_keys(samples, self.resolution))
        codes[codes == encode(voxel_keys(endpoints, self.resolution))[ray]] = -1  # own endpoint is a hit
        index, found = self.lookup(codes)
        found &= self.n[index] >= self.min_points
        if not found.any():
            return 0
        pairs = np.unique(np.column_stack([ray[found], index[found]]), axis=0)
        ray, voxel = pairs[:, 0], pairs[:, 1]
        n = self.n[voxel]
        mean = self.total[voxel]/n[:, None]
        cov = self.outer[voxel]/n[:, None, None]-mean[:, :, None]*mean[:, None, :]
        inverse = np.linalg.inv(cov+self.regularization*np.eye(3))
        u, o = direction[ray], origin-mean
        a = np.einsum('ni,nij,nj->n', u, inverse, u)
        b = np.einsum('ni,nij,nj->n', u, inverse, o)
        # Closest approach along the measured segment only: a ray that stops
        # before reaching a distribution has not passed through it.
        t = np.clip(-b/a, 0, length[ray])
        p = o+u*t[:, None]
        d2 = np.einsum('ni,nij,nj->n', p, inverse, p)
        # A ray ending on the same surface (e.g. in a neighbouring cell of a
        # wall) terminated there; only rays ending beyond the surface pass it.
        e = endpoints[ray]-mean
        beyond = np.einsum('ni,nij,nj->n', e, inverse, e) > self.pass_d2
        passed = voxel[(d2 <= self.pass_d2) & beyond]
        np.add.at(self.passes, passed, 1)
        return len(passed)

    def features(self, centres):
        """Features for occupied voxel centres (NaN where unknown)."""
        keys = voxel_keys(centres, self.resolution)
        codes = encode(keys)
        index, found = self.lookup(codes)
        n = np.where(found, self.n[index], 0.)
        passes = np.where(found, self.passes[index], 0.)
        with np.errstate(invalid='ignore', divide='ignore'):
            result = dict(points=n, passes=passes, permeability=passes/(n+passes),
                          intensity=np.where(n > 0, self.intensity[index]/n, np.nan))
        zero3 = np.zeros((len(n), 3, 3))
        local = shape_features(n, np.where(found[:, None], self.total[index], 0.),
                               np.where(found[:, None, None], self.outer[index], zero3), self.min_points)
        result.update({f'{k}_voxel': v for k, v in local.items()})
        # 3x3x3 neighbourhood moments; code addition is exact for small offsets.
        total_n, total_sum, total_outer = np.zeros(len(n)), np.zeros((len(n), 3)), np.zeros((len(n), 3, 3))
        for offset in encode(NEIGHBOURS)-encode(np.zeros((1, 3))):
            index, found = self.lookup(codes+offset)
            total_n += np.where(found, self.n[index], 0.)
            total_sum += np.where(found[:, None], self.total[index], 0.)
            total_outer += np.where(found[:, None, None], self.outer[index], 0.)
        wide = shape_features(total_n, total_sum, total_outer, self.min_points)
        result.update({f'{k}_neighbourhood': v for k, v in wide.items()})
        return result


@dataclass(frozen=True)
class LidarClassSettings:
    """A-priori thresholds; not fitted to any labelled data."""
    min_evidence: int = 5
    permeable: float = .25
    linear: float = .6
    vertical: float = .8
    low_vegetation_max_height_m: float = 1.
    ground_clearance_m: float = .3


def lidar_classes(features, height, cell, settings=LidarClassSettings()):
    """Rule-based LiDAR-only classes from height above ground and features."""
    s = settings
    height = np.asarray(height, dtype=np.float64)
    labels = np.zeros(len(height), dtype=np.uint8)
    ground = height <= .5*cell
    labels[ground] = LIDAR_CLASSES.index('ground')
    above = ~ground  # includes columns without a ground reference (NaN)
    enough = (features['points'] >= 3) & (features['points']+features['passes'] >= s.min_evidence)
    permeable = features['permeability'] >= s.permeable
    linear = np.zeros(len(height), dtype=bool)
    for scale in ('voxel', 'neighbourhood'):
        linear |= ((features[f'linearity_{scale}'] >= s.linear)
                   & (features[f'principal_verticality_{scale}'] >= s.vertical))
    trunk = above & enough & linear & ~permeable
    vegetation = above & enough & ~trunk & permeable
    low = vegetation & (height < s.low_vegetation_max_height_m)
    labels[trunk] = LIDAR_CLASSES.index('trunk_like')
    labels[vegetation] = LIDAR_CLASSES.index('tall_vegetation')
    labels[low] = LIDAR_CLASSES.index('low_vegetation')
    solid = above & enough & ~trunk & ~vegetation & (height > s.ground_clearance_m)
    labels[solid] = LIDAR_CLASSES.index('solid_obstacle')
    return labels
