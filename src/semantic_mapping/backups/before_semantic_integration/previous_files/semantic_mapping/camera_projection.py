"""Geometry for camera/LiDAR alignment; RGB appearance is not a semantic label."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Projection:
    source_indices: np.ndarray
    pixels: np.ndarray
    depths: np.ndarray
    rgb: np.ndarray


def transform_matrix(transform):
    """Return target-from-source from a geometry_msgs Transform."""
    q = transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid transform quaternion')
    x, y, z, w = quaternion / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]
    t = transform.translation
    matrix[:3, 3] = [t.x, t.y, t.z]
    if not np.isfinite(matrix).all():
        raise ValueError('Nonfinite transform')
    return matrix


def transform_points(points, matrix):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def xyz_from_cloud(cloud):
    """Read XYZ even when other PointCloud2 fields use integer datatypes."""
    from sensor_msgs_py import point_cloud2
    values = point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.column_stack([values[name] for name in ('x', 'y', 'z')]).reshape(-1, 3)


def project_visible_points(points, bgr_image, intrinsic, distortion,
                           camera_from_lidar, max_range=10.0,
                           visibility_cell=2, depth_tolerance=0.10):
    """Project into a raw distorted image and retain nearest measured depths.

    camera_from_lidar must relate the LiDAR acquisition time to the camera
    acquisition time. The depth filter only resolves measured returns in the
    same small pixel cell; it cannot prove visibility in unsampled space.
    """
    points = np.asarray(points, dtype=np.float64)
    intrinsic = np.asarray(intrinsic, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(distortion, dtype=np.float64)
    camera_from_lidar = np.asarray(camera_from_lidar, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('LiDAR points must have shape (N, 3)')
    if bgr_image.ndim != 3 or bgr_image.shape[2] != 3 or bgr_image.dtype != np.uint8:
        raise ValueError('Expected a uint8 BGR image')
    if (not np.isfinite(intrinsic).all() or intrinsic[0, 0] <= 0
            or intrinsic[1, 1] <= 0 or not np.isfinite(distortion).all()):
        raise ValueError('Invalid camera calibration')
    if camera_from_lidar.shape != (4, 4) or not np.isfinite(camera_from_lidar).all():
        raise ValueError('Invalid camera-from-LiDAR transform')
    if max_range <= 0 or visibility_cell < 1 or depth_tolerance < 0:
        raise ValueError('Invalid range or visibility settings')

    finite = np.isfinite(points).all(axis=1)
    distances = np.linalg.norm(points, axis=1)
    indices = np.flatnonzero(finite & (distances > 0.1) & (distances <= max_range))
    camera_points = transform_points(points[indices], camera_from_lidar)
    front = camera_points[:, 2] > 0.01
    indices, camera_points = indices[front], camera_points[front]
    if not len(indices):
        return Projection(indices, np.empty((0, 2), dtype=np.int64),
                          np.empty(0), np.empty((0, 3), dtype=np.uint8))

    projected, _ = cv2.projectPoints(
        camera_points, np.zeros(3), np.zeros(3), intrinsic,
        distortion if distortion.size else None)
    projected = projected.reshape(-1, 2)
    height, width = bgr_image.shape[:2]
    # Bound floating-point coordinates before rounding to avoid overflow.
    inside = (np.isfinite(projected).all(axis=1)
              & (projected[:, 0] >= 0) & (projected[:, 0] <= width-1)
              & (projected[:, 1] >= 0) & (projected[:, 1] <= height-1))
    indices = indices[inside]
    pixels = np.rint(projected[inside]).astype(np.int64)
    depths = camera_points[inside, 2]
    cells_wide = (width + visibility_cell - 1) // visibility_cell
    cells = (pixels[:, 1] // visibility_cell) * cells_wide + pixels[:, 0] // visibility_cell
    nearest = np.full(cells_wide * ((height+visibility_cell-1)//visibility_cell), np.inf)
    np.minimum.at(nearest, cells, depths)
    visible = depths <= nearest[cells] + depth_tolerance
    indices, pixels, depths = indices[visible], pixels[visible], depths[visible]
    rgb = bgr_image[pixels[:, 1], pixels[:, 0], ::-1].copy()
    return Projection(indices, pixels, depths, rgb)


def draw_projection(bgr_image, projection, max_range=10.0):
    """Draw depth-colored LiDAR samples for visual calibration inspection."""
    overlay = bgr_image.copy()
    if len(projection.pixels):
        depth = np.clip(projection.depths / max_range * 255, 0, 255).astype(np.uint8)
        colors = cv2.applyColorMap(depth[:, None], cv2.COLORMAP_TURBO)[:, 0, :]
        for pixel, color in zip(projection.pixels, colors):
            cv2.circle(overlay, tuple(pixel), 2, tuple(int(c) for c in color), -1)
    return overlay
