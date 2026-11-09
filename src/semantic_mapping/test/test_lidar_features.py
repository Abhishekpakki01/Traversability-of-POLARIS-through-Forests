"""Check LiDAR shape, permeability and class rules on synthetic scenes."""

import numpy as np
import pytest

from semantic_mapping.lidar_features import (
    LIDAR_CLASSES, VoxelStatistics, encode, lidar_classes, shape_features)
from semantic_mapping.semantic_fusion import voxel_keys

CELL = .35
RNG = np.random.default_rng(3)


def scan(stats, origin, points):
    stats.add_points(points, np.ones(len(points)))
    stats.add_rays(origin, points)


def centre_of(point):
    return (voxel_keys(np.asarray(point, dtype=float)[None], CELL)+.5)*CELL


def test_codes_are_ordered_and_additive():
    keys = np.array([[-3, 4, 0], [10, -2, 7]])
    offset = encode([[1, -1, 1]])-encode([[0, 0, 0]])
    np.testing.assert_array_equal(encode(keys)+offset, encode(keys+[1, -1, 1]))
    with pytest.raises(ValueError):
        encode([[1 << 21, 0, 0]])


def test_shape_features_for_line_plane_and_blob():
    def moments(points):
        return len(points), points.sum(0)[None], (points[:, :, None]*points[:, None, :]).sum(0)[None]
    t = RNG.uniform(-1, 1, 200)
    line = np.column_stack([.01*RNG.normal(size=200), .01*RNG.normal(size=200), t])
    plane = np.column_stack([RNG.uniform(-1, 1, 200), RNG.uniform(-1, 1, 200), .005*RNG.normal(size=200)])
    blob = RNG.normal(size=(200, 3))
    f = {name: shape_features(*map(np.atleast_1d, moments(p)), 4) for name, p in
         [('line', line), ('plane', plane), ('blob', blob)]}
    assert f['line']['linearity'][0] > .9 and f['line']['principal_verticality'][0] > .95
    assert f['plane']['planarity'][0] > .9 and f['plane']['normal_verticality'][0] > .95
    assert f['blob']['scattering'][0] > .5
    assert np.isnan(shape_features(np.array([2.]), np.zeros((1, 3)), np.zeros((1, 3, 3)), 4)['linearity'][0])


def test_rays_through_sparse_foliage_count_but_solid_wall_and_endpoint_do_not():
    stats = VoxelStatistics(CELL)
    origin = np.array([0., 0., 1.])
    # Sparse foliage cluster at x = 2 m; a solid wall at x = 4 m behind it.
    foliage = np.array([2.275, .175, 1.225])+RNG.normal(scale=.04, size=(30, 3))
    wall = np.column_stack([np.full(400, 4.1), RNG.uniform(-.4, .6, 400), RNG.uniform(.5, 1.5, 400)])
    for _ in range(3):
        scan(stats, origin, np.vstack([foliage[:10], wall]))
    f = stats.features(np.vstack([centre_of([2.275, .175, 1.225]), centre_of([4.1, .1, 1.1])]))
    assert f['passes'][0] > 0 and f['permeability'][0] > .25
    assert f['passes'][1] == 0 and f['permeability'][1] == 0


def test_grazing_ray_above_thin_ground_is_not_a_pass_through():
    stats = VoxelStatistics(CELL)
    ground = np.column_stack([RNG.uniform(1, 3, 2000), RNG.uniform(-1, 1, 2000), RNG.normal(scale=.005, size=2000)+.02])
    stats.add_points(ground, np.ones(len(ground)))
    # Ray ends at 6 m and crosses the ground cells 0.25 m above the plane.
    stats.add_rays([0., 0., .27], [[6., 0., .27]])
    f = stats.features(np.array([[2.1, .1, .1]]))
    assert f['passes'][0] == 0


def test_ray_stopping_before_a_distribution_does_not_pass_it():
    stats = VoxelStatistics(CELL)
    cluster = np.array([3.1, .1, .1])+RNG.normal(scale=.03, size=(20, 3))
    stats.add_points(cluster, np.ones(len(cluster)))
    stats.add_rays([0., .1, .1], [[2.6, .1, .1]])
    assert stats.features(centre_of(cluster[0]))['passes'][0] == 0


def test_class_rules():
    n = 6
    features = dict(points=np.full(n, 20.), passes=np.array([0, 0, 30, 30, 0, 0.]),
                    linearity_voxel=np.array([.1, .9, .2, .2, .2, .2]),
                    principal_verticality_voxel=np.array([.1, .95, .1, .1, .1, .1]),
                    linearity_neighbourhood=np.full(n, .1), principal_verticality_neighbourhood=np.full(n, .1))
    features['permeability'] = features['passes']/(features['points']+features['passes'])
    height = np.array([0., 1., .4, 1.6, .7, np.nan])
    names = [LIDAR_CLASSES[i] for i in lidar_classes(features, height, CELL)]
    assert names == ['ground', 'trunk_like', 'low_vegetation', 'tall_vegetation', 'solid_obstacle', 'unknown']
    # Too little evidence stays unknown rather than guessed.
    features['points'][:] = 1.
    features['passes'][:] = 0.
    assert set(lidar_classes(features, np.full(n, .7), CELL)) == {0}


def test_fill_sparse_uses_classified_neighbours_only():
    from semantic_mapping.lidar_features import fill_sparse
    cell = .2
    grid = np.array([[x, y, 0] for x in range(3) for y in range(3)], dtype=float)*cell+cell/2
    tall = LIDAR_CLASSES.index('tall_vegetation')
    unknown = LIDAR_CLASSES.index('unknown')
    classes = np.full(len(grid), tall)
    classes[4] = unknown  # the centre voxel, with few points
    points = np.full(len(grid), 10.)
    points[4] = 1
    features = dict(points=points, passes=np.zeros(len(grid)))
    filled_classes, filled = fill_sparse(grid, classes, features, cell)
    assert filled_classes[4] == tall and filled.tolist() == [i == 4 for i in range(len(grid))]
    # A well-measured unknown voxel is left alone, and so is one with too few classified neighbours.
    points[4] = 10
    assert fill_sparse(grid, classes, dict(points=points, passes=np.full(len(grid), 10.)), cell)[0][4] == unknown
    points[4] = 1
    lonely = np.full(len(grid), unknown)
    lonely[0] = tall
    assert fill_sparse(grid, lonely, features, cell)[0][4] == unknown
