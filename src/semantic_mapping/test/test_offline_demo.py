"""Independent geometry, matching and serialization checks for offline output."""
import os
from types import SimpleNamespace

import numpy as np
import pytest

from semantic_mapping.offline_demo import NearestImage, OfflineOhm, cloud_message, local_cloud
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
