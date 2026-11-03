"""Check rigid trajectory alignment and lever-arm recovery on synthetic data."""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from check_trajectory_gps import fit_with_lever_arm, rigid_fit  # noqa: E402


def yaw(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def test_rigid_fit_recovers_rotation_and_translation():
    rng = np.random.default_rng(0)
    source = rng.normal(size=(50, 3))*10
    rotation, translation = yaw(.7), np.array([3., -2., 1.])
    r, t = rigid_fit(source, source @ rotation.T+translation)
    np.testing.assert_allclose(r, rotation, atol=1e-9)
    np.testing.assert_allclose(t, translation, atol=1e-9)


def test_lever_arm_is_recovered_on_a_turning_path():
    # Straight then turning, like the recorded drive (a pure circle cannot
    # separate a forward offset from a frame rotation).
    straight = np.column_stack([np.linspace(-30, 0, 40), np.full(40, -10.), np.zeros(40)])
    angles = np.linspace(-np.pi/2, np.pi/2, 40)
    turn = np.column_stack([10*np.cos(angles), 10*np.sin(angles), np.zeros(40)])
    positions = np.vstack([straight, turn])
    positions[:, 2] = -.1*np.arange(80)
    rotations = np.array([yaw(0.)]*40+[yaw(a+np.pi/2) for a in angles])
    arm = np.array([.3, -.4, .2])
    frame, offset = yaw(-.4), np.array([5., 1., 2.])
    target = (positions+rotations @ arm) @ frame.T+offset
    r, t, estimated = fit_with_lever_arm(positions, rotations, target)
    np.testing.assert_allclose(estimated[:2], arm[:2], atol=1e-3)
    aligned = (positions+rotations @ estimated) @ r.T+t
    assert np.abs(aligned-target).max() < 1e-3
