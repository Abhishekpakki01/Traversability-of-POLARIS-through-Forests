#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class StandaloneVoxelNode(Node):
    def __init__(self):
        super().__init__('standalone_voxel_node')
        
        # 🛠️ CONFIGURATION & RANGE KNOBS
        self.res = 0.35                      # Voxel resolution (35 cm)
        self.max_sensor_range = 10.0         # Limit perception distance to 10m
        self.horizon_radius = 10.0          # Operational rolling map horizon
        self.process_every_n_frames = 2      # Throttle frame rate
        self.point_step_stride = 4           # Subsample point cloud (take 1 out of 4 points for speed)
        self.target_frame = 'odom'           # Change to 'map' or 'base_link' if 'odom' is missing
        
        # 📊 PROBABILISTIC OCCUPANCY PARAMETERS (Log-Odds)
        self.l_occ = 0.85                    # Hit increment
        self.l_free = -0.40                  # Free space decay
        self.l_min = -2.00                   # Deletion floor
        self.l_max = 3.50                    # Confidence ceiling
        self.l_thresh = 0.00                 # Render threshold
        
        self.frame_counter = 0
        self.voxel_grid = {}          
        self.robot_pos = [0.0, 0.0, 0.0]
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/os1_cloud_node/points',  
            self.point_cloud_callback,
            qos_profile_sensor_data
        )
        
        self.marker_pub = self.create_publisher(Marker, '/semantic_voxel_map', 10)
        self.get_logger().info(
            f"🔄 Probabilistic Occupancy Node Active | Max Range: {self.max_sensor_range}m | Res: {self.res}m"
        )

    def point_cloud_callback(self, msg):
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        # FIX 1: Synchronize transform with exact PointCloud2 header timestamp
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # Fallback to latest transform if exact time lookup fails slightly
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time()
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                return

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        self.robot_pos = [tx, ty, tz]
        
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # 3D Rotation Matrix Construction
        r11 = 1.0 - 2.0 * (qy**2 + qz**2)
        r12 = 2.0 * (qx*qy - qz*qw)
        r13 = 2.0 * (qx*qz + qy*qw)
        r21 = 2.0 * (qx*qy + qz*qw)
        r22 = 1.0 - 2.0 * (qx**2 + qz**2)
        r23 = 2.0 * (qy*qz - qx*qw)
        r31 = 2.0 * (qx*qz - qy*qw)
        r32 = 2.0 * (qy*qz + qx*qw)
        r33 = 1.0 - 2.0 * (qx**2 + qy**2)

        # FIX 2: Subsample points to prevent Python thread stalling
        raw_points = list(pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True))
        points = raw_points[::self.point_step_stride]

        for p in points:
            lx, ly, lz = p[0], p[1], p[2]
            
            # 1. Perception Range Filtering
            local_dist = math.sqrt(lx**2 + ly**2 + lz**2)
            if local_dist > self.max_sensor_range or local_dist < 0.5:
                continue

            # 2. Coordinate Transformation
            gx = r11 * lx + r12 * ly + r13 * lz + tx
            gy = r21 * lx + r22 * ly + r23 * lz + ty
            gz = r31 * lx + r32 * ly + r33 * lz + tz
            
            # 3. Ray-Marching (Free Space Decay)
            steps = int(local_dist / self.res)
            if steps > 1:
                dx = (gx - tx) / steps
                dy = (gy - ty) / steps
                dz = (gz - tz) / steps

                # FIX 3: Track visited voxels along the ray to avoid duplicate decay
                visited_ray_voxels = set()
                for i in range(1, steps):
                    rx = tx + i * dx
                    ry = ty + i * dy
                    rz = tz + i * dz
                    
                    rv_idx = (
                        int(math.floor(rx / self.res)),
                        int(math.floor(ry / self.res)),
                        int(math.floor(rz / self.res))
                    )
                    
                    if rv_idx not in visited_ray_voxels:
                        visited_ray_voxels.add(rv_idx)
                        if rv_idx in self.voxel_grid:
                            self.voxel_grid[rv_idx]['log_odds'] += self.l_free

            # 4. Color Assignment
            rel_z = gz - tz
            val = max(0.0, min(1.0, (rel_z - (-2.0)) / 6.0))
            if val < 0.25:
                rgb_color = [0.0, val * 4.0, 1.0]                  
            elif val < 0.5:
                rgb_color = [0.0, 1.0, 1.0 - (val - 0.25) * 4.0]    
            elif val < 0.75:
                rgb_color = [(val - 0.5) * 4.0, 1.0, 0.0]          
            else:
                rgb_color = [1.0, 1.0 - (val - 0.75) * 4.0, 0.0]    

            # 5. Hit Voxel Log-Odds Update
            v_idx = (
                int(math.floor(gx / self.res)),
                int(math.floor(gy / self.res)),
                int(math.floor(gz / self.res))
            )
            
            if v_idx not in self.voxel_grid:
                self.voxel_grid[v_idx] = {
                    'position': (
                        v_idx[0] * self.res + self.res / 2.0, 
                        v_idx[1] * self.res + self.res / 2.0, 
                        v_idx[2] * self.res + self.res / 2.0
                    ),
                    'log_odds': 0.0,
                    'color': rgb_color
                }
            
            self.voxel_grid[v_idx]['log_odds'] = min(
                self.l_max, self.voxel_grid[v_idx]['log_odds'] + self.l_occ
            )
            self.voxel_grid[v_idx]['color'] = rgb_color

        # 🧹 Memory Cleanup & Rolling Horizon Pruning
        keys_to_prune = []
        for v_idx, voxel_data in list(self.voxel_grid.items()):
            vx, vy, vz = voxel_data['position']
            dist = math.sqrt((vx - tx)**2 + (vy - ty)**2 + (vz - tz)**2)
            
            if voxel_data['log_odds'] <= self.l_min or dist > self.horizon_radius:
                keys_to_prune.append(v_idx)
        
        for k in keys_to_prune:
            del self.voxel_grid[k]

        self.publish_map()

    def publish_map(self):
        if not self.voxel_grid:
            return
            
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        marker.scale.x = self.res
        marker.scale.y = self.res
        marker.scale.z = self.res
        
        marker.points = []
        marker.colors = []
        
        for v_idx, voxel_data in self.voxel_grid.items():
            if voxel_data['log_odds'] >= self.l_thresh:
                p = Point()
                p.x, p.y, p.z = voxel_data['position']
                marker.points.append(p)
                
                c = ColorRGBA()
                c.r, c.g, c.b = voxel_data['color']
                c.a = 1.0  
                marker.colors.append(c)
            
        self.marker_pub.publish(marker)
        self.get_logger().info(
            f"📊 Confirmed Occupied Voxels: {len(marker.points)} / Total Tracked: {len(self.voxel_grid)}", 
            throttle_duration_sec=2.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = StandaloneVoxelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()