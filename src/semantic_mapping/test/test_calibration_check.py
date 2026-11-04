import numpy as np
import pytest

from semantic_mapping.calibration_check import (
    ClickSession, depth_at, interpolate_pose, lidar_view, project, summarise_pairs)

K = np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1]])


def test_project_keeps_points_in_view_and_reports_depth():
    points = np.array([[0., 0, 10], [1., 0, 10], [0., 0, -5], [100., 0, 10]])
    pixels, depths, kept = project(points, np.eye(4), K, 640, 480)
    assert kept.tolist() == [0, 1]
    assert np.allclose(pixels, [[320, 240], [370, 240]]) and np.allclose(depths, 10)


def test_near_point_wins_in_lidar_view():
    pixels = np.array([[50., 50], [50., 50]])
    _, depth = lidar_view(pixels, np.array([20., 5.]), 100, 100, radius=0)
    assert depth[50, 50] == 5.
    assert depth_at(depth, (52, 50)) == 5.
    assert depth_at(depth, (90, 90)) is None


def test_pose_interpolation_is_halfway():
    first, second = np.eye(4), np.eye(4)
    second[:3, 3] = [2, 0, 0]
    second[:2, :2] = [[0, -1], [1, 0]]  # 90 degrees about z
    pose = interpolate_pose([0, 10], [first, second], 5)
    assert np.allclose(pose[:3, 3], [1, 0, 0])
    assert np.isclose(np.degrees(np.arctan2(pose[1, 0], pose[0, 0])), 45)


def test_summary_separates_systematic_offset_from_noise():
    pairs = [dict(camera=[100, 100], lidar=[103+s, 100], depth_m=10.) for s in (-1, 1, -1, 1)]
    result = summarise_pairs(pairs, 500.)
    assert np.allclose(result['mean_offset_px'], [3, 0])
    assert result['median_error_px'] == pytest.approx(3)
    assert result['median_error_m_at_feature'] == pytest.approx(3*10/500)


def test_summary_rejects_no_pairs():
    with pytest.raises(ValueError):
        summarise_pairs([], 500.)


def test_click_session_pairs_camera_then_lidar_and_undoes():
    session = ClickSession()
    session.click('lidar', '00001', (5, 5))
    assert not session.pairs and session.pending is None
    session.click('camera', '00001', (1, 2))
    session.click('camera', '00001', (3, 4))  # a second camera click replaces the first
    session.click('lidar', '00001', (6, 4), 12.)
    assert session.pairs == [dict(frame='00001', camera=[3., 4.], lidar=[6., 4.], depth_m=12.)]
    session.click('camera', '00001', (1, 1))
    session.click('lidar', '00002', (1, 1))  # the frame changed: never pair across frames
    assert len(session.pairs) == 1
    session.undo()
    assert session.pairs == []
