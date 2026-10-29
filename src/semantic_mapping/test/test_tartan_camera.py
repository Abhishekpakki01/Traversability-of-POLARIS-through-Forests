"""Verify rectified geometry uses source S21 calibration, not RELLIS values."""

from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
import yaml

from semantic_mapping.tartan_camera import (
    pose_matrix, quaternion_from_matrix, rectified_geometry)
from semantic_mapping.camera_projection import transform_points


def calibration():
    path = Path(__file__).resolve().parents[1]/'config/tartan_meadows_camera.yaml'
    return yaml.safe_load(path.read_text())


def test_rectified_intrinsics_use_projection_and_own_dimensions():
    config = calibration()
    intrinsic, camera, lidar = rectified_geometry(config)
    assert (config['width'], config['height']) == (1024, 544)
    assert intrinsic[0, 0] == pytest.approx(477.6049499511719)
    assert intrinsic[0, 0] != config['camera_K_raw'][0]
    assert np.allclose(lidar, np.eye(4))
    assert np.isfinite(camera).all()


def test_rectification_frame_rotation_preserves_projection_geometry():
    config = calibration()
    _, vehicle_rectified, _ = rectified_geometry(config)
    vehicle_optical = pose_matrix(config['vehicle_from_head']) @ pose_matrix(config['head_from_camera'])
    raw_point = np.array([[0.2, 0.1, 4.]])
    world = transform_points(raw_point, vehicle_optical)
    rectified = transform_points(world, np.linalg.inv(vehicle_rectified))
    expected = raw_point @ np.array(config['camera_R']).reshape(3, 3).T
    assert np.allclose(rectified, expected, atol=1e-5)


@pytest.mark.parametrize('quaternion', [[0, 0, 0, 1], [-.5, .5, -.5, .5],
                                       [np.sqrt(.5), -np.sqrt(.5), 0, 0]])
def test_rotation_to_quaternion_roundtrip(quaternion):
    pose = pose_matrix(dict(translation=[0, 0, 0], quaternion=quaternion))
    result = quaternion_from_matrix(pose[:3, :3])
    reconstructed = pose_matrix(dict(translation=[0, 0, 0], quaternion=result))
    assert np.allclose(pose, reconstructed, atol=1e-7)


def test_invalid_calibration_rejected():
    bad = deepcopy(calibration())
    bad['camera_R'] = [0]*9
    with pytest.raises(ValueError, match='rotation'):
        rectified_geometry(bad)
    bad = deepcopy(calibration())
    bad['camera_P'][3] = 1.
    with pytest.raises(ValueError, match='left-camera'):
        rectified_geometry(bad)
