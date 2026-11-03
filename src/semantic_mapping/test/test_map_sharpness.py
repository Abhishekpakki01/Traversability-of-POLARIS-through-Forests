"""Check the flat-patch thickness measure used for map sharpness."""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from check_map_sharpness import Moments  # noqa: E402


def test_thickness_measures_noise_of_flat_patches_and_ignores_walls():
    rng = np.random.default_rng(2)
    ground = np.column_stack([rng.uniform(0, 4, 20000), rng.uniform(0, 4, 20000), rng.normal(scale=.01, size=20000)+.2])
    wall = np.column_stack([np.full(5000, 6.2)+rng.normal(scale=.01, size=5000), rng.uniform(0, 4, 5000), rng.uniform(0, 4, 5000)])
    m = Moments(.4)
    m.add(ground); m.add(wall)
    thickness = m.thickness()
    assert len(thickness) == 100  # 10 x 10 ground patches; wall patches are not horizontal
    assert .008 < np.median(thickness) < .012
    # A second, offset copy of the ground (a pose error) makes patches thicker.
    m.add(ground+[0, 0, .05])
    assert np.median(m.thickness()) > .02
