"""Estimate camera-LiDAR image offset from the outline of the terrain against the sky.

Each column gives the top row of the scene in the camera (first non-sky pixel) and in the LiDAR
view (first dense returns). Sliding one outline over the other finds the shift where their shape
agrees best. The shape match fixes the horizontal shift well. The vertical shift is less certain,
because a camera sees thin branch tips that a LiDAR rarely hits.
"""
import cv2
import numpy as np


def camera_outline(bgr, sky_level=200):
    """Top row of the non-sky scene per column; sky is bright and connected to the image top."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    bright = (cv2.GaussianBlur(gray, (5, 5), 0) >= sky_level).astype(np.uint8)
    bright = cv2.morphologyEx(bright, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    _, labels = cv2.connectedComponents(bright)
    top_labels = np.unique(labels[0][labels[0] > 0])
    sky = np.isin(labels, top_labels)
    return first_false(sky).astype(float)


def first_false(mask):
    """First row in each column that is False, or the height if every row is True."""
    height = mask.shape[0]
    rows = np.argmax(~mask, axis=0)
    return np.where(mask.all(0), height, rows)


def solid_columns(bgr, rows, depth_rows=35, dark_level=115, margin=3, limit=430):
    """Columns whose camera outline is a dense dark mass: not thin branches, not sun glare."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    usable = np.zeros(len(rows), bool)
    for x, row in enumerate(rows):
        row = int(row)
        if margin < row < limit:
            usable[x] = gray[row+2:row+2+depth_rows, x].mean() < dark_level
    return cv2.morphologyEx(usable.astype(np.uint8)[None], cv2.MORPH_OPEN, np.ones((1, 25), np.uint8))[0].astype(bool)


def lidar_outline(depth, close=9, density=.5):
    """Top row of dense LiDAR returns per column; NaN where a column has none."""
    filled = (depth > 0).astype(np.uint8)
    closed = cv2.morphologyEx(filled, cv2.MORPH_CLOSE, np.ones((close, close), np.uint8))
    solid = cv2.blur(closed.astype(np.float32), (close, close)) >= density
    rows = np.argmax(solid, axis=0).astype(float)
    rows[~solid.any(0)] = np.nan
    return rows


def best_shift(camera_rows, lidar_rows, search=40, min_columns=150, usable=None):
    """Return (dx, dy, spread, columns): LiDAR outline minus camera outline at the best match.

    usable marks camera columns that may be compared. Spread is the median absolute difference
    left after removing the vertical offset, so a low value means the shapes truly agree.
    """
    width = len(camera_rows)
    usable = np.ones(width, bool) if usable is None else usable
    costs = {}
    for dx in range(-search, search+1):
        x = np.arange(max(0, -dx), min(width, width-dx))
        difference = lidar_rows[x+dx]-camera_rows[x]
        difference = difference[usable[x] & np.isfinite(difference)]
        if len(difference) >= min_columns:
            dy = float(np.median(difference))
            costs[dx] = (float(np.median(np.abs(difference-dy))), dy, len(difference))
    if not costs:
        return None
    dx = min(costs, key=lambda k: costs[k][0])
    spread, dy, columns = costs[dx]
    return dx, dy, spread, columns
