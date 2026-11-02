"""Check camera-to-ground projection and water evidence on synthetic scenes."""

import numpy as np
import pytest

from semantic_mapping.lidar_features import encode
from semantic_mapping.semantic_fusion import voxel_keys
from semantic_mapping.water_detection import (
    WATER, GroundEvidence, camera_directions, fill_ground, ground_hits, pixel_grid)

CELL = .35


def test_fill_ground_fills_small_holes_only_within_iterations():
    ground = np.zeros((9, 9))
    ground[3:6, 3:6] = np.nan
    filled = fill_ground(ground, 2)
    assert np.isfinite(filled).all() and np.allclose(filled, 0)
    wide = np.full((9, 9), np.nan)
    wide[0, 0] = 1.
    assert np.isfinite(fill_ground(wide, 2)).sum() == 9


def test_rays_hit_flat_ground_at_expected_cells_and_occlusion_blocks():
    ground = np.zeros((60, 60))  # covers x, y in [0, 21) m
    origin = np.array([1., 10.5, 2.])
    # Looking along +x and 45 degrees down: hits the ground 2 m ahead.
    direction = np.array([[1., 0., -1.]])/np.sqrt(2)
    cells, points, hit = ground_hits(origin, direction, ground, np.zeros(2, dtype=int), CELL, np.empty(0, dtype=np.int64))
    assert hit[0] and points[0, 0] == pytest.approx(3., abs=.1)
    assert tuple(cells[0]) == tuple(voxel_keys(np.array([[3., 10.5]]), CELL)[0])
    # An above-ground voxel in front of the hit point occludes it.
    blocker = encode(voxel_keys(np.array([[2.2, 10.5, .8]]), CELL))
    _, _, hit = ground_hits(origin, direction, ground, np.zeros(2, dtype=int), CELL, np.sort(blocker))
    assert not hit[0]
    # A ray looking up never hits the ground.
    _, _, hit = ground_hits(origin, np.array([[1., 0., .2]])/np.hypot(1, .2), ground, np.zeros(2, dtype=int),
                            CELL, np.empty(0, dtype=np.int64))
    assert not hit[0]


def test_camera_directions_centre_pixel_is_optical_axis():
    intrinsic = np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1]])
    d = camera_directions(intrinsic, np.array([320.]), np.array([240.]))
    np.testing.assert_allclose(d[0], [0, 0, 1])
    u, v = pixel_grid(64, 32, 8)
    assert len(u) == 8*4 and u.min() == 4 and v.max() == 28


def test_evidence_needs_several_frames_and_water_majority():
    evidence = GroundEvidence()
    key = np.array([[3, 4]])
    for frame in range(2):
        evidence.add(np.repeat(key, 6, 0), np.array([WATER]*4+[1]*2), np.zeros(6), frame)
    assert len(evidence.water()[0]) == 0  # only two frames
    evidence.add(np.repeat(key, 6, 0), np.array([WATER]*4+[1]*2), np.zeros(6), 2)
    keys, fraction, frames, _ = evidence.water()
    assert keys.tolist() == [[3, 4]] and fraction[0] == pytest.approx(2/3) and frames[0] == 3
    # Unknown-class pixels do not dilute the water share.
    evidence.add(np.repeat(key, 10, 0), np.zeros(10, dtype=int), np.zeros(10), 3)
    assert evidence.water()[1][0] == pytest.approx(2/3)
    assert evidence.majority()[1].tolist() == [WATER]
