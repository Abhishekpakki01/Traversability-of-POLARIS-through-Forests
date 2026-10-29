"""Compose the Tartan recording's explicit parent-from-child transform chain."""

import numpy as np

from semantic_mapping.tartan_camera import pose_matrix, quaternion_from_matrix


def recorded_geometry(config):
    """Return rectified K, vehicle-from-rectified-camera and vehicle-from-LiDAR.

    This checks geometry consistency, not calibration accuracy. The current
    recording lacks CameraInfo, so its P and R remain explicitly provisional.
    """
    if config['width'] <= 0 or config['height'] <= 0:
        raise ValueError('Invalid image dimensions')
    projection = np.asarray(config['camera_P'], dtype=float).reshape(3, 4)
    rotation = np.asarray(config['camera_R'], dtype=float).reshape(3, 3)
    quaternion_from_matrix(rotation)
    if (not np.isfinite(projection).all() or projection[0, 0] <= 0
            or projection[1, 1] <= 0 or not np.allclose(projection[:, 3], 0)
            or not np.allclose(projection[2, :3], [0, 0, 1])):
        raise ValueError('Expected a valid rectified left-camera projection')
    transform = np.eye(4)
    frame = 'vehicle'
    if not config['transform_chain']:
        raise ValueError('Missing recorded camera transform chain')
    for link in config['transform_chain']:
        if link['parent'] != frame:
            raise ValueError('Disconnected or reversed recorded transform chain')
        transform = transform @ pose_matrix(link)
        frame = link['child']
    if frame != config['image_frame']:
        raise ValueError('Recorded chain does not end in the image optical frame')
    optical_from_rectified = np.eye(4)
    optical_from_rectified[:3, :3] = rotation.T
    lidar = config['vehicle_from_lidar']
    if lidar['parent'] != 'vehicle' or lidar['child'] != 'velodyne_1':
        raise ValueError('Unexpected recorded LiDAR frames')
    return projection[:, :3], transform @ optical_from_rectified, pose_matrix(lidar)

