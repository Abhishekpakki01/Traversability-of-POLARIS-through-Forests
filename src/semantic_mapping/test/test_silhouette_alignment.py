import numpy as np

from semantic_mapping.silhouette_alignment import best_shift, camera_outline, first_false, lidar_outline, solid_columns


def bumpy(width=600):
    x = np.arange(width)
    return 120+40*np.sin(x/23.)+25*np.sin(x/7.)


def test_best_shift_recovers_a_known_offset():
    camera = bumpy()
    lidar = np.roll(camera, -18)+5  # LiDAR outline 18 px left and 5 px lower
    dx, dy, spread, _ = best_shift(camera, lidar)
    assert (dx, dy) == (-18, 5) and spread < 1


def test_best_shift_needs_enough_columns():
    camera = bumpy()
    usable = np.zeros(len(camera), bool)
    usable[:20] = True
    assert best_shift(camera, camera, usable=usable) is None


def test_first_false_handles_all_true_columns():
    mask = np.array([[1, 1], [0, 1]], bool)
    assert first_false(mask).tolist() == [1, 2]


def test_outlines_find_the_top_of_a_dark_mass_below_bright_sky():
    image = np.full((200, 300, 3), 250, np.uint8)
    image[80:] = 40
    rows = camera_outline(image)
    assert np.allclose(rows[20:-20], 80, atol=4)
    assert solid_columns(image, rows)[20:-20].all()
    depth = np.zeros((200, 300), np.float32)
    depth[90:] = 10
    assert np.allclose(lidar_outline(depth)[20:-20], 90, atol=6)
