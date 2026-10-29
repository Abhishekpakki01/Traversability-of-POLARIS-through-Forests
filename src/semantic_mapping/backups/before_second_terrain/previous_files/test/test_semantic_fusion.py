"""Check visibility, occupied-only updates and independent-frame evidence."""

import json
from pathlib import Path

import numpy as np
import pytest

from semantic_mapping.semantic_fusion import (
    SemanticEvidence, first_occupied, surface_observations, voxel_keys)
from semantic_mapping.camera_projection import project_visible_points


def probability(label, confidence=0.95):
    values = np.full(6, (1-confidence)/5)
    values[label] = confidence
    return values


@pytest.mark.parametrize('origin,endpoint,occupied,expected', [
    ([0.1, 0.1, 0.1], [4.2, 0.1, 0.1], {(2, 0, 0), (4, 0, 0)}, (2, 0, 0)),
    ([4.1, 0.1, 0.1], [-2.2, 0.1, 0.1], {(-1, 0, 0), (-3, 0, 0)}, (-1, 0, 0)),
    ([0.1, 0.1, 0.1], [2.2, 2.2, 2.2], {(1, 1, 1)}, (1, 1, 1)),
    ([0.1, 0.1, 0.1], [2.2, 0.1, 0.1], {(0, 0, 0), (2, 0, 0)}, (0, 0, 0)),
    ([0.1, 0.1, 0.1], [0.2, 0.1, 0.1], set(), None),
])
def test_first_surface_traversal(origin, endpoint, occupied, expected):
    assert first_occupied(origin, endpoint, occupied, 1.0) == expected


def test_occluded_endpoint_does_not_label_front_surface_or_free_space():
    occupied = {(2, 0, 0), (4, 0, 0)}
    values, status = surface_observations([[4.2, 0.2, 0.2], [3.2, 0.2, 0.2]],
        [probability(4), probability(2)], [0.2, 0.2, 0.2], occupied, 1.0)
    assert not values
    assert status['occluded'] == 1
    assert status['missing_surface'] == 1
    # An actual nearer return can label its own surface.
    values, _ = surface_observations([[2.2, 0.2, 0.2]], [probability(3)],
                                     [0.2, 0.2, 0.2], occupied, 1.0)
    assert list(values) == [(2, 0, 0)]


def test_dense_points_and_duplicate_frames_cannot_inflate_evidence():
    occupied = {(2, 0, 0)}
    observations, _ = surface_observations(np.repeat([[2.2, 0.2, 0.2]], 100, axis=0),
        np.repeat([probability(2)], 100, axis=0), [0.2, 0.2, 0.2], occupied, 1.0)
    fusion = SemanticEvidence()
    assert fusion.update(observations, 10, occupied) == 1
    labels, _, counts = fusion.query([(2, 0, 0)])
    assert labels.tolist() == [0]  # Only one independent image.
    assert counts.tolist() == [1]
    assert fusion.update(observations, 10, occupied) == 0
    assert fusion.update(observations, 9, occupied) == 0
    fusion.update(observations, 11, occupied)
    labels, confidence, counts = fusion.query([(2, 0, 0)])
    assert labels.tolist() == [2]
    assert confidence[0] > 0.6
    assert counts.tolist() == [2]


def test_conflicting_evidence_stays_unknown_and_free_voxels_are_forgotten():
    occupied = {(2, 0, 0)}
    fusion = SemanticEvidence()
    fusion.update({(2, 0, 0): probability(2)}, 10, occupied)
    fusion.update({(2, 0, 0): probability(4)}, 11, occupied)
    assert fusion.query([(2, 0, 0)])[0].tolist() == [0]
    fusion.retain_occupied(set())
    assert not fusion.voxels
    assert fusion.query([(2, 0, 0)])[2].tolist() == [0]
    assert fusion.update({(2, 0, 0): probability(2)}, 12, set()) == 0


def test_negative_grid_coordinates_and_invalid_probabilities():
    assert voxel_keys([[-0.01, 0, 1.99]], 1.0).tolist() == [[-1, 0, 1]]
    with pytest.raises(ValueError, match='normalized'):
        surface_observations([[1, 0, 0]], [[1]*6], [0, 0, 0], {(1, 0, 0)}, 1.0)


def test_official_trunk_water_sky_and_ground_class_mapping():
    path = Path(__file__).resolve().parents[1]/'config/goose_classes.json'
    classes = json.loads(path.read_text())['classes']
    assert len(classes) == 64
    for index, name, group in [(28, 'tree_trunk', 'trunk'), (54, 'water', 'water'),
                                (53, 'sky', 'unknown'), (31, 'soil', 'ground'),
                                (16, 'forest', 'vegetation'), (8, 'ego_vehicle', 'unknown')]:
        assert classes[index] == dict(id=index, name=name, group=group)


def test_measured_depth_rejects_far_surface_without_solid_cube_overblocking():
    # A coarse occupied cell can protrude into the ray ahead of the actual return.
    # Current measured camera depth, rather than that cube volume, gates this mode.
    points = np.array([[0., 0., 2.2], [0., 0., 4.2], [0., 0., 2.21]])
    image = np.zeros((100, 100, 3), np.uint8)
    intrinsic = [[50, 0, 50], [0, 50, 50], [0, 0, 1]]
    projected = project_visible_points(points, image, intrinsic, [], np.eye(4), depth_tolerance=0.0)
    assert projected.source_indices.tolist() == [0]
    occupied = {(0, 0, 1), (0, 0, 2), (0, 0, 4)}
    values, _ = surface_observations(points[projected.source_indices], [probability(2)],
        [0, 0, 0], occupied, 1.0, strict_voxel_occlusion=False)
    assert list(values) == [(0, 0, 2)]
    assert (0, 0, 1) not in values and (0, 0, 4) not in values
