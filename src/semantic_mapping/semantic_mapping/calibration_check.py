"""Measure camera-LiDAR alignment from clicked feature pairs.

The LiDAR view is drawn with the same camera model and pose as the image, so a
feature clicked in both views differs only by the alignment error. That error
includes calibration, timing and pose error, and the person's click noise.
"""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def interpolate_pose(stamps, poses, stamp):
    """World-from-vehicle at a time between recorded poses, clamped to their range."""
    stamps = np.asarray(stamps, dtype=np.float64)
    stamp = min(max(float(stamp), stamps[0]), stamps[-1])
    upper = min(max(int(np.searchsorted(stamps, stamp)), 1), len(stamps)-1)
    lower = upper-1
    share = (stamp-stamps[lower])/(stamps[upper]-stamps[lower])
    both = Rotation.from_matrix(np.stack([poses[lower][:3, :3], poses[upper][:3, :3]]))
    result = np.eye(4)
    result[:3, :3] = Slerp([0., 1.], both)([share]).as_matrix()[0]
    result[:3, 3] = (1-share)*poses[lower][:3, 3]+share*poses[upper][:3, 3]
    return result


def project(points, camera_from_world, intrinsic, width, height, near=.5, far=45.):
    """Return pixels, depths and the indices of world points inside the image."""
    camera = points @ camera_from_world[:3, :3].T + camera_from_world[:3, 3]
    ahead = np.flatnonzero((camera[:, 2] > near) & (camera[:, 2] < far))
    pixels = camera[ahead] @ np.asarray(intrinsic).T
    pixels = pixels[:, :2]/pixels[:, 2:3]
    inside = (pixels[:, 0] >= 0) & (pixels[:, 0] < width) & (pixels[:, 1] >= 0) & (pixels[:, 1] < height)
    return pixels[inside], camera[ahead[inside], 2], ahead[inside]


def lidar_view(pixels, depths, width, height, radius=1):
    """Draw nearest-first points; returns a colour image and a depth image in metres (0 = empty)."""
    depth_image = np.zeros((height, width), np.float32)
    order = np.argsort(-depths)  # far first, so near points overwrite them
    columns, rows = np.round(pixels[order]).astype(int).T
    near = depths[order].astype(np.float32)
    for dy in range(-radius, radius+1):
        for dx in range(-radius, radius+1):
            r, c = rows+dy, columns+dx
            keep = (r >= 0) & (r < height) & (c >= 0) & (c < width)
            depth_image[r[keep], c[keep]] = near[keep]
    filled = depth_image > 0
    scaled = np.clip(np.log(np.maximum(depth_image, 1.))/np.log(45.), 0, 1)
    colour = cv2.applyColorMap((255*(1-scaled)).astype(np.uint8), cv2.COLORMAP_TURBO)
    colour[~filled] = 18
    return colour, depth_image


def depth_at(depth_image, pixel, radius=4):
    """Nearest measured depth around a pixel, or None where the view is empty."""
    x, y = int(round(pixel[0])), int(round(pixel[1]))
    patch = depth_image[max(y-radius, 0):y+radius+1, max(x-radius, 0):x+radius+1]
    found = patch[patch > 0]
    return float(found.min()) if found.size else None


def summarise_pairs(pairs, focal_px):
    """Summarise pairs of {'camera': [x, y], 'lidar': [x, y], 'depth_m': d}.

    The mean offset is the systematic part. The spread around it is mostly click
    noise and feature ambiguity, which is why both are reported.
    """
    if not pairs:
        raise ValueError('No clicked pairs')
    offsets = np.array([np.subtract(p['lidar'], p['camera']) for p in pairs], dtype=float)
    size = np.linalg.norm(offsets, axis=1)
    mean = offsets.mean(0)
    spread = offsets.std(0, ddof=1) if len(offsets) > 1 else np.zeros(2)
    depths = np.array([np.nan if p.get('depth_m') is None else p['depth_m'] for p in pairs], dtype=float)
    metres = size*depths/focal_px
    return dict(pairs=len(pairs), median_error_px=float(np.median(size)), rms_error_px=float(np.sqrt((size**2).mean())),
                p90_error_px=float(np.percentile(size, 90)), max_error_px=float(size.max()),
                mean_offset_px=mean.tolist(), offset_spread_px=spread.tolist(),
                mean_offset_standard_error_px=(spread/np.sqrt(len(offsets))).tolist(),
                mean_offset_deg=np.degrees(np.arctan(mean/focal_px)).tolist(),
                median_error_m_at_feature=None if np.isnan(metres).all() else float(np.nanmedian(metres)),
                median_feature_depth_m=None if np.isnan(depths).all() else float(np.nanmedian(depths)))


class ClickSession:
    """Pair a camera click with the next LiDAR click; the GUI only draws this state."""

    def __init__(self, pairs=None):
        self.pairs = list(pairs or [])
        self.pending = None

    def click(self, view, frame, pixel, depth_m=None):
        """Return a short message for the person clicking."""
        pixel = [float(pixel[0]), float(pixel[1])]
        if view == 'camera':
            self.pending = dict(frame=frame, camera=pixel)
            return 'Now find the same point in the LiDAR view'
        if self.pending is None or self.pending['frame'] != frame:
            self.pending = None
            return 'Click the point in the camera view first'
        self.pairs.append(dict(self.pending, lidar=pixel, depth_m=depth_m))
        self.pending = None
        return 'Saved'

    def undo(self):
        if self.pending is not None:
            self.pending = None
        elif self.pairs:
            self.pairs.pop()
