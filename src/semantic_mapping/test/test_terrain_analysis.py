"""Check terrain descriptors on synthetic voxel scenes with known answers."""

import numpy as np
import pytest

from semantic_mapping.semantic_fusion import CLASS_NAMES, voxel_keys
from semantic_mapping.terrain_analysis import (
    STATES, TerrainSettings, analyze, box_sum, path_states, vehicle_consistency)

CELL = .35
LABEL = {name: i for i, name in enumerate(CLASS_NAMES)}


def centres(points):
    """Snap points to occupied voxel centres, as OHM reports them."""
    keys = np.unique(voxel_keys(points, CELL), axis=0)
    return (keys+.5)*CELL


def ground(size=8., slope_deg=0.):
    x, y = np.meshgrid(np.arange(-size, size, CELL/2), np.arange(-size, size, CELL/2))
    z = np.tan(np.radians(slope_deg))*x
    return centres(np.column_stack([x.ravel(), y.ravel(), z.ravel()]))


def state_at(terrain, xy, combined=True):
    index = terrain.footprint(np.asarray(xy, dtype=float), .01)
    state = terrain.combined_state if combined else terrain.geometric_state
    return STATES[state[index][0]]


def with_box(points, label, low=0., high=.9, centre=(0., 0.)):
    x, y, z = np.meshgrid(np.arange(-.3, .3, .1)+centre[0], np.arange(-.3, .3, .1)+centre[1],
                          np.arange(low+CELL, high, .1))
    box = centres(np.column_stack([x.ravel(), y.ravel(), z.ravel()]))
    labels = np.concatenate([np.full(len(points), LABEL['ground']), np.full(len(box), LABEL[label])])
    return np.vstack([points, box]), labels


def test_box_sum_matches_direct_neighbourhood_sum():
    values = np.random.default_rng(1).random((7, 9))
    padded = np.pad(values, 2)
    direct = np.array([[padded[i:i+5, j:j+5].sum() for j in range(9)] for i in range(7)])
    np.testing.assert_allclose(box_sum(values, 2), direct)


@pytest.mark.parametrize('slope', [0., 10., 25.])
def test_plane_fit_recovers_quantized_slope(slope):
    terrain = analyze(ground(slope_deg=slope), np.full(len(ground(slope_deg=slope)), LABEL['ground']))
    inner = terrain.slope_deg[10:-10, 10:-10]
    # 0.35 m height layers make a staircase: medians stay within ~2 degrees, but
    # individual 2.45 m windows alias by up to ~5.5 degrees (documented limit).
    assert np.nanmedian(inner) == pytest.approx(slope, abs=2.5)
    assert np.nanmax(np.abs(inner-slope)) <= 6.
    # Ground height follows the plane within one voxel layer.
    expected = np.tan(np.radians(slope))*((np.arange(terrain.observed.shape[0])+terrain.origin[0]+.5)*CELL)
    error = terrain.ground_z-expected[:, None]
    assert np.nanmax(np.abs(error[10:-10, 10:-10])) <= CELL


def test_steep_slope_exceeds_limit_and_flat_ground_is_traversable():
    flat = ground()
    assert state_at(analyze(flat, np.full(len(flat), 1)), (0., 0.)) == 'traversable'
    steep = ground(slope_deg=30.)
    assert state_at(analyze(steep, np.full(len(steep), 1)), (0., 0.)) == 'non_traversable'


def test_rigid_obstacle_vegetation_and_unknown_labels():
    for label, combined in [('unknown', 'non_traversable'), ('vegetation', 'caution'),
                            ('trunk', 'non_traversable'), ('obstacle', 'non_traversable')]:
        points, labels = with_box(ground(), label)
        terrain = analyze(points, labels)
        assert state_at(terrain, (0., 0.), combined=False) == 'non_traversable'
        assert state_at(terrain, (0., 0.)) == combined
        assert terrain.obstacle_height_m[terrain.footprint(np.zeros(2), .01)][0] == pytest.approx(2*CELL)
        # Neighbouring open ground is unaffected.
        assert state_at(terrain, (3., 3.)) == 'traversable'


def test_tall_vegetation_is_not_relaxed():
    points, labels = with_box(ground(), 'vegetation', high=1.8)
    assert state_at(analyze(points, labels), (0., 0.)) == 'non_traversable'


def test_water_surface_is_a_hazard_only_with_semantics():
    points = ground()
    labels = np.full(len(points), LABEL['ground'])
    labels[np.hypot(points[:, 0], points[:, 1]) < .5] = LABEL['water']
    terrain = analyze(points, labels)
    assert state_at(terrain, (0., 0.), combined=False) == 'traversable'
    assert state_at(terrain, (0., 0.)) == 'non_traversable'


def test_canopy_above_overhang_is_ignored_and_not_ground():
    points, labels = with_box(ground(), 'vegetation', low=2.5, high=3.5)
    terrain = analyze(points, labels)
    assert state_at(terrain, (0., 0.), combined=False) == 'traversable'
    # A canopy-only column surrounded by observed ground is not ground itself.
    floor = ground()
    floor = floor[~np.all(voxel_keys(floor[:, :2], CELL) == voxel_keys([[5., 5.]], CELL), 1)]
    canopy = centres(np.array([[5., 5., 4.]]))
    only = analyze(np.vstack([floor, canopy]), np.zeros(len(floor)+1, dtype=int))
    index = only.footprint(np.array([5., 5.]), .01)
    assert not only.ground_measured[index][0]
    assert only.ground_z[index][0] == pytest.approx(.5*CELL)


def test_unobserved_columns_remain_unobserved_and_path_checks():
    points = ground()
    keep = ~((np.abs(points[:, 0]) < 1.) & (np.abs(points[:, 1]) < 1.))
    terrain = analyze(points[keep], np.ones(keep.sum(), dtype=int))
    assert state_at(terrain, (0., 0.)) == 'unobserved'
    worst = path_states(terrain, np.array([[0., 0., 0.], [4., 4., 0.], [50., 50., 0.]]), .2)
    assert [STATES[s] for s in worst] == ['unobserved', 'traversable', 'unobserved']


def test_vehicle_consistency_on_level_and_tilted_ground():
    points = ground(slope_deg=10.)
    terrain = analyze(points, np.ones(len(points), dtype=int))
    angle = np.radians(10.)
    pose = np.eye(4)
    pose[:3, :3] = [[np.cos(angle), 0, -np.sin(angle)], [0, 1, 0], [np.sin(angle), 0, np.cos(angle)]]
    pose[:3, 3] = [0., 0., .8+.5*CELL]
    heights, angles = vehicle_consistency(terrain, pose[None], .6)
    assert heights[0] == pytest.approx(.8, abs=CELL)
    assert angles[0] == pytest.approx(0., abs=2.)


def test_settings_reject_unknown_keys_and_unresolvable_values():
    with pytest.raises(ValueError):
        TerrainSettings.from_config({'limits': {'max_slop_deg': 10}})
    with pytest.raises(ValueError):
        TerrainSettings(rigid_labels=('vegetation',))
    with pytest.raises(ValueError):
        TerrainSettings(ground_clearance_m=3.)
    settings = TerrainSettings.from_config({'vehicle': {'footprint_radius_m': .6, 'ground_clearance_m': .4},
                                            'semantics': {'hazard_labels': ['water']}})
    assert settings.ground_clearance_m == .4 and settings.hazard_labels == ('water',)


def test_carpet_places_cells_on_ground_and_omits_unreferenced_columns():
    from semantic_mapping.terrain_analysis import carpet
    points, labels = with_box(ground(), 'trunk')
    terrain = analyze(points, labels)
    xyz, state, cost, slope = carpet(terrain)
    assert len(xyz) == np.isfinite(terrain.ground_z).sum() == len(state) == len(cost) == len(slope)
    np.testing.assert_allclose(xyz[:, 2], .5*CELL)
    centre = np.argmin(np.hypot(xyz[:, 0]-.175, xyz[:, 1]-.175))
    assert STATES[state[centre]] == 'non_traversable' and cost[centre] == 1
    # Geometry-only carpet differs only through semantics.
    _, geometric, _, _ = carpet(terrain, combined=False)
    assert STATES[geometric[centre]] == 'non_traversable'
