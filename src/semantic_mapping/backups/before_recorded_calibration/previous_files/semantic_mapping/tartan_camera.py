"""Rectified-camera geometry for the publisher's TartanDrive S21 calibration."""

from types import SimpleNamespace
import numpy as np

from semantic_mapping.camera_projection import transform_matrix


def pose_matrix(values):
    xyz, xyzw = values['translation'], values['quaternion']
    return transform_matrix(SimpleNamespace(
        translation=SimpleNamespace(**dict(zip('xyz', xyz))),
        rotation=SimpleNamespace(**dict(zip('xyzw', xyzw)))))


def quaternion_from_matrix(rotation):
    rotation = np.asarray(rotation, dtype=float)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5)):
        raise ValueError('Invalid camera rotation')
    r = rotation
    if np.trace(r) > 0:
        scale = 2*np.sqrt(1+np.trace(r))
        q = np.array([(r[2, 1]-r[1, 2])/scale, (r[0, 2]-r[2, 0])/scale,
                      (r[1, 0]-r[0, 1])/scale, scale/4])
    else:
        axis = int(np.diag(r).argmax())
        j, k = (axis+1)%3, (axis+2)%3
        scale = 2*np.sqrt(1+r[axis, axis]-r[j, j]-r[k, k])
        q = np.zeros(4)
        q[axis] = scale/4
        q[j], q[k] = (r[j, axis]+r[axis, j])/scale, (r[k, axis]+r[axis, k])/scale
        q[3] = (r[k, j]-r[j, k])/scale
    return q/np.linalg.norm(q)


def rectified_geometry(config):
    """Return K_rect, vehicle-from-rectified-camera and vehicle-from-LiDAR.

    The source image is already rectified: use P's 3x3 intrinsic block and
    zero lens distortion. R maps raw camera coordinates to rectified ones,
    so the child rectified frame has R.T relative to the raw optical frame.
    """
    projection = np.asarray(config['camera_P'], dtype=float).reshape(3, 4)
    rotation = np.asarray(config['camera_R'], dtype=float).reshape(3, 3)
    quaternion_from_matrix(rotation)
    if (not config['input_is_rectified'] or config['width'] <= 0 or config['height'] <= 0
            or not np.isfinite(projection).all() or projection[0, 0] <= 0
            or projection[1, 1] <= 0 or not np.allclose(projection[:, 3], 0)):
        raise ValueError('Expected valid rectified left-camera calibration')
    optical_from_rectified = np.eye(4)
    optical_from_rectified[:3, :3] = rotation.T
    vehicle_from_rectified = (pose_matrix(config['vehicle_from_head'])
        @ pose_matrix(config['head_from_camera']) @ optical_from_rectified)
    return projection[:, :3], vehicle_from_rectified, pose_matrix(config['vehicle_from_lidar'])
