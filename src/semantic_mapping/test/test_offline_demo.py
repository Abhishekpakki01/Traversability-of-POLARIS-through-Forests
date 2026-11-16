"""Independent geometry, matching and serialization checks for offline output."""
import os
from types import SimpleNamespace

import numpy as np
import pytest

from semantic_mapping.offline_demo import (NearestImage, OfflineOhm, cloud_message, colored_points,
                                          headlight_colors, in_camera_view, local_cloud)
from semantic_mapping.semantic_fusion import voxel_keys


def image(stamp):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
        sec=stamp//10**9, nanosec=stamp % 10**9)))


def test_matching_nearest_tolerance_and_no_reuse():
    matcher = NearestImage([image(100_000_000), image(200_000_000), image(300_000_000)])
    assert matcher.match(80_000_000).header.stamp.nanosec == 100_000_000
    assert matcher.match(110_000_000) is None  # same image cannot count twice
    assert matcher.match(249_000_000).header.stamp.nanosec == 200_000_000
    assert matcher.match(351_000_000) is None  # beyond 50 ms


def test_matching_rejects_time_reversal():
    matcher = NearestImage([image(100_000_000)])
    matcher.match(100_000_000)
    with pytest.raises(ValueError):
        matcher.match(90_000_000)
    # After selecting a future exposure, do not fall back to an older unused
    # exposure when LiDAR arrives more frequently than the camera.
    matcher = NearestImage([image(150_000_000), image(200_000_000)])
    assert matcher.match(190_000_000).header.stamp.nanosec == 200_000_000
    assert matcher.match(195_000_000) is None


def test_cloud_preserves_attributes_and_elevation_reference():
    from std_msgs.msg import Header
    from sensor_msgs_py.point_cloud2 import read_points
    points = np.array([[1., 2., 3.], [1., 2., 5.]])
    args = (Header(frame_id='sensor_init'), points, np.array([0, 2]),
            np.array([.3, .9]), np.array([1, 4]), np.array([-1., 1.]))
    semantic = read_points(cloud_message(*args))
    elevation = read_points(cloud_message(*args, elevation_colors=True))
    for key in ['x', 'y', 'z', 'label', 'confidence', 'observations', 'relative_z']:
        np.testing.assert_array_equal(semantic[key], elevation[key])
    assert list(semantic['label']) == [0, 2]
    assert list(semantic['relative_z']) == [-1, 1]
    assert not np.array_equal(semantic['rgb'], elevation['rgb'])


def test_offline_ohm_uses_world_origin_and_only_occupied_endpoints():
    executable = os.environ.get('OHM_OFFLINE_EXECUTABLE')
    if not executable:
        pytest.skip('Build the C++ bridge and provide OHM_OFFLINE_EXECUTABLE')
    mapper = OfflineOhm(executable, .35)
    try:
        first = mapper.integrate([10., 0., 0.], [[11., 0., 0.]])
        keys = set(map(tuple, voxel_keys(first, .35)))
        assert tuple(voxel_keys([[11., 0., 0.]], .35)[0]) in keys
        assert tuple(voxel_keys([[10.4, 0., 0.]], .35)[0]) not in keys
        assert len(keys) == 1  # unknown/free cells must not be exported
        # A second translated sensor observation accumulates in the same world map.
        second = mapper.integrate([10., 1., 0.], [[11., 1., 0.]])
        keys = set(map(tuple, voxel_keys(second, .35)))
        assert tuple(voxel_keys([[11., 0., 0.]], .35)[0]) in keys
        assert tuple(voxel_keys([[11., 1., 0.]], .35)[0]) in keys
        assert len(keys) == 2
        with pytest.raises(ValueError):
            mapper.integrate([np.nan, 0., 0.], [[1., 0., 0.]])
    finally:
        mapper.close()


def test_local_view_preserves_unknown_and_semantic_fields_and_requires_exact_pose():
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Header
    from sensor_msgs_py.point_cloud2 import read_points
    header = Header(frame_id='sensor_init'); header.stamp.sec = 3
    cloud = cloud_message(header, np.array([[1, 0, 0], [2, 0, 0], [50, 0, 0]]),
        np.array([0, 2, 1]), np.array([.3, .9, .8]), np.array([0, 3, 5]), np.zeros(3))
    pose = PoseStamped(header=header)
    cropped = local_cloud(cloud, pose, 15.)
    assert cropped.width == 2 and cloud.width == 3
    assert cropped.fields == cloud.fields
    assert bytes(cropped.data) == bytes(cloud.data[:2*cloud.point_step])
    assert list(read_points(cropped)['label']) == [0, 2]
    different_pose = PoseStamped(header=Header(frame_id='sensor_init'))
    different_pose.header.stamp.sec = 4
    with pytest.raises(ValueError):
        local_cloud(cloud, different_pose, 15.)


def test_colored_points_pack_xyz_and_rgb_per_point():
    from std_msgs.msg import Header
    xyz = np.array([[1., 2., 3.], [-4., 5.5, 0.25]])
    rgb = np.array([[255, 0, 0], [10, 20, 30]], dtype=np.uint8)
    message = colored_points(Header(frame_id='sensor_init'), xyz, rgb)
    assert (message.width, message.point_step) == (2, 16)
    assert [f.name for f in message.fields] == ['x', 'y', 'z', 'rgb']
    data = np.frombuffer(bytes(message.data), dtype=np.dtype(dict(
        names=['x', 'y', 'z', 'rgb'], formats=['<f4']*3+['<u4'], offsets=[0, 4, 8, 12], itemsize=16)))
    assert np.allclose(np.column_stack([data['x'], data['y'], data['z']]), xyz)
    assert list(data['rgb']) == [0xFF0000, (10 << 16) | (20 << 8) | 30]
    with pytest.raises(ValueError):
        colored_points(Header(), xyz, rgb[:1])



def test_in_camera_view_keeps_points_inside_the_image_and_reach():
    intrinsic = np.array([[100., 0, 50], [0, 100., 25], [0, 0, 1]])
    camera = np.eye(4); camera[:3, 3] = [1., 2., 3.]  # optical frame: z forward
    points = np.array([[1, 2, 8], [1, 2, -2], [1, 2, 20], [9, 2, 8], [1.2, 2.1, 5]], dtype=float)
    assert in_camera_view(points, camera, intrinsic, 100, 50, 10.).tolist() == [
        True, False, False, False, True]  # ahead / behind / too far / outside the image / ahead
    hood = np.zeros((50, 100), dtype=bool); hood[28:, 55:] = True  # hides only (1.2, 2.1, 5): u=60, v=30
    assert in_camera_view(points, camera, intrinsic, 100, 50, 10., hood).tolist() == [
        True, False, False, False, False]


def test_headlight_colors_light_the_beam_and_dim_the_rest():
    colours = headlight_colors(np.array([1, 0, 1, 0]), np.array([True, True, False, False]))
    assert colours[0].tolist() == [139, 90, 43]                 # ground, inside the beam
    assert colours[1].tolist() == [210, 210, 210]               # no class yet, inside the beam
    assert colours[2].sum() < .8*colours[0].sum() and colours[3].tolist() == [70, 70, 70]  # outside: dimmed
