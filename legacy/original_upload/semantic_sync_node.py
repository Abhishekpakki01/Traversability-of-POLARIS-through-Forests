#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
import message_filters
from cv_bridge import CvBridge
import cv2

class SemanticSyncNode(Node):
    def __init__(self):
        super().__init__('semantic_sync_node')
        self.bridge = CvBridge()

        # NOTE: Verify these topic names against your 'ros2 bag info <bag_name>' output
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/rgb/image_raw')
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/lidar/pointcloud')
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/odometry/filtered')

        # Synchronize topics running at different rates (Camera 20Hz, LiDAR 10Hz)
        # slop=0.05 allows a maximum 50ms time discrepancy between matching headers
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub, self.odom_sub],
            queue_size=10,
            slop=0.05
        )
        self.ts.registerCallback(self.synchronized_callback)
        self.get_logger().info("Multi-modal sensor synchronizer initialized successfully.")

    def synchronized_callback(self, image_msg, lidar_msg, odom_msg):
        self.get_logger().info("--- Synchronized Sensor Frame Captured ---")
        
        # Extrapolate timestamp alignment for logging verification
        img_time = image_msg.header.stamp.sec + (image_msg.header.stamp.nanosec * 1e-9)
        lidar_time = lidar_msg.header.stamp.sec + (lidar_msg.header.stamp.nanosec * 1e-9)
        self.get_logger().info(f"Sync Delta T: {abs(img_time - lidar_time):.4f} seconds")

        try:
            # Task 6: Convert ROS Image message to OpenCV array for pipeline ingestion
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            
            # Placeholder for Task 7: Run ML model inference here on CPU
            # segmented_mask = self.run_inference(cv_image)
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert image matrix: {str(e)}")

        # Task 4: Ingest and print vehicle odometry coordinates
        pos = odom_msg.pose.pose.position
        self.get_logger().info(f"Odometry Frame Pos: X={pos.x:.2f}, Y={pos.y:.2f}, Z={pos.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SemanticSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()