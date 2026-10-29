"""Check geometry, real RGB sampling and measured-depth visibility rejection."""

import cv2
import numpy as np
import pytest
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from semantic_mapping.camera_projection import (
    project_visible_points, transform_matrix, transform_points, xyz_from_cloud)


def calibration():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    image[50, 50] = [10, 20, 30]
    return image, np.array([[50., 0, 50], [0, 50., 50], [0, 0, 1]])


def test_rgb_and_occlusion_filter():
    image, intrinsic = calibration()
    result = project_visible_points(
        [[0, 0, 2], [0, 0, 4], [0, 0, -2], [100, 0, 2], [np.nan, 0, 1]],
        image, intrinsic, [], np.eye(4))
    assert result.source_indices.tolist() == [0]
    assert result.pixels.tolist() == [[50, 50]]
    assert result.rgb.tolist() == [[30, 20, 10]]


def test_translation_rotation_and_image_time_motion():
    image, intrinsic = calibration()
    # LiDAR at scan time is at x=1; camera at image time is at x=2.
    world_lidar, world_camera = np.eye(4), np.eye(4)
    world_lidar[0, 3], world_camera[0, 3] = 1, 2
    camera_lidar = np.linalg.solve(world_camera, world_lidar)
    result = project_visible_points([[1, 0, 2]], image, intrinsic, [], camera_lidar)
    assert result.pixels.tolist() == [[50, 50]]
    assert transform_points([[1, 0, 2]], world_lidar).tolist() == [[2, 0, 2]]
    transform = Transform()
    transform.translation.x = 1.0
    transform.rotation.z = transform.rotation.w = np.sqrt(0.5)
    assert transform_points([[1, 0, 0]], transform_matrix(transform))[0] == pytest.approx([1, 1, 0])


def test_raw_image_distortion_is_applied():
    image, intrinsic = calibration()
    points = np.array([[1., 0, 2.]])
    distortion = np.array([0.4, 0., 0., 0., 0.])
    result = project_visible_points(points, image, intrinsic, distortion, np.eye(4))
    expected, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), intrinsic, distortion)
    assert result.pixels.tolist() == np.rint(expected.reshape(-1, 2)).astype(int).tolist()
    assert result.pixels[0, 0] != 75  # Undistorted projection would land here.


def test_empty_cloud_and_uncalibrated_camera():
    image, intrinsic = calibration()
    assert len(project_visible_points(np.empty((0, 3)), image, intrinsic, [], np.eye(4)).rgb) == 0
    with pytest.raises(ValueError, match='calibration'):
        project_visible_points([[0, 0, 2]], image, np.zeros((3, 3)), [], np.eye(4))


def test_mixed_type_lidar_fields_and_nonstandard_field_order():
    # Like Ouster: integer metadata coexists with floating-point coordinates.
    fields = [PointField(name='ring', offset=0, datatype=PointField.UINT16, count=1)]
    fields += [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
               for name, offset in [('z', 4), ('x', 8), ('y', 12)]]
    cloud = point_cloud2.create_cloud(Header(), fields, [(7, 3.0, 1.0, 2.0)])
    assert xyz_from_cloud(cloud).tolist() == [[1.0, 2.0, 3.0]]
