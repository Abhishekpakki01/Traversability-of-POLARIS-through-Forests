#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import cv2

# Message Type Imports
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2

# ROS 2 Time Synchronization and CV Utilities
import message_filters
from cv_bridge import CvBridge

# TF2 Transform Imports
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class VoxelMapNode(Node):
    def __init__(self):
        super().__init__('voxel_map_node')
        
        # 1. Declare Node Parameters
        self.declare_parameter('voxel_resolution', 0.25)
        self.declare_parameter('local_frame', 'base_link')
        
        # 2. Extract Parameter Values
        self.resolution = self.get_parameter('voxel_resolution').value
        self.local_frame = self.get_parameter('local_frame').value
        
        # 3. Core Database & Utilities
        self.voxel_map = {}
        self.bridge = CvBridge()
        
        # 4. Setup TF2 Coordinate Buffer and Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 5. Global State Storage for Filtered Odometry Data
        self.latest_odom = None
        
        # 6. Configure Explicit QoS Profile for Vehicle State Telemetry
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            odom_qos
        )
        
        # 7. Configure QoS Profiles for High-Frequency Sensor Feeds
        # High-volume streaming data is optimized via BEST_EFFORT reliability
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 8. Define message_filters Subscribers for Synchronization
        self.lidar_sub = message_filters.Subscriber(
            self, PointCloud2, '/os1_cloud_node/points', qos_profile=sensor_qos
        )
        self.camera_sub = message_filters.Subscriber(
            self, Image, '/pylon_camera_node/image_raw', qos_profile=sensor_qos
        )
        
        # 9. Instantiate the Time Synchronizer
        # queue_size: Cache history depth for processing slow pipelines
        # slop: Max allowed timestamp divergence (0.05 seconds = 50ms)
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.camera_sub],
            queue_size=15,
            slop=0.05
        )
        self.time_synchronizer.registerCallback(self.synchronized_perception_callback)
        
        self.get_logger().info("=======================================================")
        self.get_logger().info("Multi-Modal Synchronized Voxel Mapper Node Active.")
        self.get_logger().info(f"Voxel Resolution Set To: {self.resolution} meters")
        self.get_logger().info("Temporal Slop Window Configured to: 50 ms")
        self.get_logger().info("=======================================================")

    def odom_callback(self, msg: Odometry):
        """Constantly caches the latest available telemetry."""
        self.latest_odom = msg

    def quaternion_to_rotation_matrix(self, q):
        """Converts a geometry_msgs/Quaternion into an unstructured 3x3 rotation matrix."""
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2*(y**2 + z**2),   2*(x*y - z*w),       2*(x*z + y*w)],
            [2*(x*y - z*w),       1 - 2*(x**2 + z**2),   2*(y*z - x*w)],
            [2*(x*z - y*w),       2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
        ])

    def synchronized_perception_callback(self, lidar_msg: PointCloud2, img_msg: Image):
        """Executes execution logic when a synchronized pair of sensors is found."""
        if self.latest_odom is None:
            self.get_logger().warn("Waiting for initial vehicle odometry state...", throttle_duration_sec=2.0)
            return

        try:
            # --- STAGE 1: IMAGE CONVERSION VIA CV_BRIDGE ---
            # Unpack the raw ROS byte-array matrix directly into a native BGR OpenCV array
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            img_height, img_width, _ = cv_image.shape

            # --- STAGE 2: COORDINATE TRANSFORMS (Sensor -> base_link) ---
            # Look up sensor orientation relative to the robot chassis base frame
            transform = self.tf_buffer.lookup_transform(
                self.local_frame,
                lidar_msg.header.frame_id,
                rclpy.time.Time()
            )
            
            # Extract point elements to bypass core library array assertion limitations
            data_generator = pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)
            structured_pts = np.array(list(data_generator))
            
            if structured_pts.size == 0:
                return

            pts = np.zeros((len(structured_pts), 3))
            pts[:, 0] = structured_pts['x']
            pts[:, 1] = structured_pts['y']
            pts[:, 2] = structured_pts['z']
            
            t_local = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            R_local = self.quaternion_to_rotation_matrix(transform.transform.rotation)
            pts_base = (R_local @ pts.T).T + t_local

            # --- STAGE 3: GLOBAL MAP PROJECTION (base_link -> World) ---
            pos = self.latest_odom.pose.pose.position
            ori = self.latest_odom.pose.pose.orientation
            
            t_global = np.array([pos.x, pos.y, pos.z])
            R_global = self.quaternion_to_rotation_matrix(ori)
            pts_world = (R_global @ pts_base.T).T + t_global

            # --- STAGE 4: SPATIAL QUANTIZATION ---
            voxel_indices = np.floor(pts_world / self.resolution).astype(int)
            for idx in voxel_indices:
                self.voxel_map[tuple(idx)] = True
            
            # --- STAGE 5: SYSTEM METRICS ENGINE OUTPUT ---
            # Compute time variance metrics between sensor timestamps to observe hardware delay
            lidar_time = lidar_msg.header.stamp.sec + (lidar_msg.header.stamp.nanosec * 1e-9)
            image_time = img_msg.header.stamp.sec + (img_msg.header.stamp.nanosec * 1e-9)
            time_delta_ms = abs(lidar_time - image_time) * 1000.0

            self.get_logger().info(
                f"[SYNC SUCCESS] Slop Delta: {time_delta_ms:.1f}ms | "
                f"Img Res: {img_width}x{img_height} | "
                f"Voxels: {len(self.voxel_map)}"
            )

        except TransformException as ex:
            self.get_logger().error(f"Could not transform spatial frames: {ex}")
        except Exception as e:
            self.get_logger().error(f"CV_BRIDGE Image extraction failure: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoxelMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()