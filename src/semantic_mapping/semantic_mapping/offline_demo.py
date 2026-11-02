"""Small, testable helpers for sequential mapping and honest demo outputs."""

import struct
import subprocess

import numpy as np

from semantic_mapping.semantic_fusion import PALETTE

# Fixed across the whole down_meadows excerpt; chosen to include its measured
# -15.53 .. +0.58 m voxel-Z range without changing colors between frames.
ELEVATION_RANGE = (-20., 5.)


def local_cloud(cloud, pose, radius):
    """Display crop only: preserve all fields and unknown cells inside an XY radius."""
    from copy import copy
    from sensor_msgs_py.point_cloud2 import read_points
    if radius <= 0 or not np.isfinite(radius):
        raise ValueError('Local display radius must be positive')
    if cloud.header.frame_id != pose.header.frame_id or stamp_ns(cloud) != stamp_ns(pose):
        raise ValueError('Map and pose must have identical frame and timestamp')
    if cloud.height != 1 or cloud.row_step != cloud.width*cloud.point_step:
        raise ValueError('Expected the unorganized offline map format')
    points = read_points(cloud)
    p = pose.pose.position
    inside = (points['x']-p.x)**2+(points['y']-p.y)**2 <= radius**2
    result = copy(cloud)
    result.width = int(inside.sum())
    result.row_step = result.width*result.point_step
    raw = np.frombuffer(cloud.data, dtype=np.uint8).reshape(cloud.width, cloud.point_step)
    result.data = raw[inside].tobytes()
    return result


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


class NearestImage:
    """Bounded-memory nearest exposure matching; never reuse an exposure.

    Inputs must be ordered by acquisition timestamp. The next exposure is kept
    as lookahead; its future pose must be available rather than extrapolated.
    """
    def __init__(self, images, tolerance_ns=50_000_000):
        self.images = iter(images)
        self.previous = None
        self.next = next(self.images, None)
        self.last_image = -1
        self.tolerance_ns = tolerance_ns
        self.last_cloud = -1

    def match(self, stamp):
        if stamp <= self.last_cloud:
            raise ValueError('LiDAR timestamps must strictly increase')
        self.last_cloud = stamp
        while self.next is not None and stamp_ns(self.next) <= stamp:
            self.previous, old_stamp = self.next, stamp_ns(self.next)
            self.next = next(self.images, None)
            if self.next is not None and stamp_ns(self.next) <= old_stamp:
                raise ValueError('Camera timestamps must strictly increase')
        choices = [m for m in (self.previous, self.next) if m is not None
                   and stamp_ns(m) > self.last_image
                   and abs(stamp_ns(m)-stamp) <= self.tolerance_ns]
        if not choices:
            return None
        image = min(choices, key=lambda m: (abs(stamp_ns(m)-stamp), stamp_ns(m)))
        self.last_image = stamp_ns(image)
        return image


class OfflineOhm:
    """One request completes before the next scan can be integrated."""
    def __init__(self, executable, resolution):
        self.process = subprocess.Popen([str(executable), str(resolution)],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        if self._read(8) != b'OHMDEM01':
            self.close()
            raise RuntimeError('Unexpected offline OHM protocol')

    def _read(self, count):
        result = self.process.stdout.read(count)
        if len(result) != count:
            raise RuntimeError('Offline OHM exited or returned truncated output; inspect stderr')
        return result

    def integrate(self, origin, world_points):
        origin = np.asarray(origin, dtype='<f8').reshape(3)
        points = np.asarray(world_points, dtype='<f8').reshape(-1, 3)
        if not np.isfinite(origin).all() or not np.isfinite(points).all():
            raise ValueError('Nonfinite occupancy input')
        self.process.stdin.write(struct.pack('<Q', len(points)))
        self.process.stdin.write(origin.tobytes())
        self.process.stdin.write(points.tobytes())
        self.process.stdin.flush()
        count, = struct.unpack('<Q', self._read(8))
        if count > 10_000_000:
            raise RuntimeError('Implausible occupied voxel count')
        return np.frombuffer(self._read(count*24), dtype='<f8').reshape(-1, 3).copy()

    def close(self):
        if self.process.stdin and not self.process.stdin.closed:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.process.stdout.close()


def cloud_message(header, centres, labels, confidence, observations, relative_z,
                  elevation_colors=False):
    """Preserve semantic attributes even in the optional elevation view."""
    from sensor_msgs.msg import PointCloud2, PointField
    names = ['x', 'y', 'z', 'rgb', 'label', 'confidence', 'observations', 'relative_z']
    offsets = [0, 4, 8, 12, 16, 20, 24, 28]
    dtype = np.dtype(dict(names=names, formats=['<f4']*3+['<u4', 'u1', '<f4', '<u4', '<f4'],
                         offsets=offsets, itemsize=32))
    data = np.zeros(len(centres), dtype=dtype)
    for i, name in enumerate(('x', 'y', 'z')):
        data[name] = centres[:, i]
    colors = PALETTE[labels].astype(np.uint32)
    if elevation_colors:
        import cv2
        low, high = ELEVATION_RANGE
        values = np.rint(np.clip((relative_z-low)/(high-low), 0, 1)*255).astype(np.uint8)
        colors = cv2.applyColorMap(values[:, None], cv2.COLORMAP_TURBO)[:, 0, ::-1].astype(np.uint32)
    data['rgb'] = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
    data['label'], data['confidence'], data['observations'] = labels, confidence, observations
    data['relative_z'] = relative_z
    kinds = [PointField.FLOAT32]*4+[PointField.UINT8, PointField.FLOAT32,
                                  PointField.UINT32, PointField.FLOAT32]
    return PointCloud2(header=header, height=1, width=len(centres), is_dense=True,
        point_step=32, row_step=len(centres)*32, data=data.tobytes(),
        fields=[PointField(name=n, offset=o, datatype=k, count=1)
                for n, o, k in zip(names, offsets, kinds)])
