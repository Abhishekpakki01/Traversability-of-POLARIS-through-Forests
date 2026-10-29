"""Check frame conventions against recording evidence, not visual color count."""

from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from semantic_mapping.camera_projection import project_visible_points, transform_points
from semantic_mapping.recorded_calibration import recorded_geometry
from semantic_mapping.tartan_camera import pose_matrix


def config():
    path = Path(__file__).resolve().parents[1]/'config/tartan_recorded_calibration.yaml'
    return yaml.safe_load(path.read_text())


def test_mount_uses_recorded_pose_and_optical_chain():
    values = config()
    assert values['transform_chain'][0]['translation'] == [0., 0., -0.1274]
    assert values['transform_chain'][1]['translation'] == [0., 0.035, -0.002]
    _, actual, lidar = recorded_geometry(values)
    expected = np.eye(4)
    for link in values['transform_chain']:
        expected = expected @ pose_matrix(link)
    rectification = np.eye(4)
    rectification[:3, :3] = np.array(values['camera_R']).reshape(3, 3).T
    assert np.allclose(actual, expected @ rectification)
    assert np.allclose(lidar, np.eye(4))


def test_intrinsics_uncertainty_is_explicit():
    values = config()
    assert values['intrinsics_verified'] is False
    assert 'CameraInfo' in values['intrinsics_status']
    assert np.allclose(np.asarray(values['camera_R']).reshape(3, 3), np.eye(3))
    intrinsic, _, _ = recorded_geometry(values)
    assert intrinsic[0, 0] == pytest.approx(477.6049499511719)


def test_optical_forward_projects_to_principal_point_after_frame_roundtrip():
    intrinsic, vehicle_camera, vehicle_lidar = recorded_geometry(config())
    optical = np.array([[0., 0., 5.]])
    vehicle = transform_points(optical, vehicle_camera)
    lidar = transform_points(vehicle, np.linalg.inv(vehicle_lidar))
    projected = project_visible_points(lidar, np.zeros((544, 1024, 3), np.uint8),
        intrinsic, [], np.linalg.solve(vehicle_camera, vehicle_lidar))
    assert len(projected.pixels) == 1
    assert np.allclose(projected.pixels[0], np.rint(intrinsic[:2, 2]), atol=1)
    assert projected.depths[0] == pytest.approx(5.)


@pytest.mark.parametrize('change', ['reversed_link', 'wrong_optical_frame', 'wrong_lidar'])
def test_inconsistent_frame_chains_rejected(change):
    values = deepcopy(config())
    if change == 'reversed_link':
        values['transform_chain'][1]['parent'] = 'multisense/left_camera_frame'
    elif change == 'wrong_optical_frame':
        values['image_frame'] = 'camera_body_frame'
    else:
        values['vehicle_from_lidar']['child'] = 'velodyne_2'
    with pytest.raises(ValueError):
        recorded_geometry(values)


def test_bad_rectified_projection_rejected():
    values = deepcopy(config())
    values['camera_P'][3] = -100.
    with pytest.raises(ValueError, match='projection'):
        recorded_geometry(values)

