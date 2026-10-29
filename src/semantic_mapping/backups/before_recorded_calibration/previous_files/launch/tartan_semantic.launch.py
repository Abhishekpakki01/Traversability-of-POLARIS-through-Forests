"""Run the preserved mapper/model on the independently calibrated meadow data."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    shared = {'use_sim_time': True}
    config = Path(get_package_share_directory('semantic_mapping'))/'config/tartan_semantic.rviz'
    cloud = ('/os1_cloud_node/points', '/velodyne_1/velodyne_points')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('strict_voxel_occlusion', default_value='false'),
        Node(package='semantic_mapping', executable='ohm_occupancy_pose_node',
             parameters=[dict(shared, fixed_frame='sensor_init', base_frame='vehicle')],
             remappings=[cloud], output='screen'),
        Node(package='semantic_mapping', executable='tartan_camera_adapter.py',
             parameters=[shared], output='screen'),
        Node(package='semantic_mapping', executable='camera_semantic_voxel_node.py',
             parameters=[dict(shared, fixed_frame='sensor_init',
                 strict_voxel_occlusion=ParameterValue(
                     LaunchConfiguration('strict_voxel_occlusion'), value_type=bool))],
             remappings=[cloud, ('/pylon_camera_node/image_raw', '/tartan/camera/image'),
                         ('/pylon_camera_node/camera_info', '/tartan/camera/camera_info')],
             output='screen'),
        Node(package='rviz2', executable='rviz2', parameters=[shared],
             arguments=['-d', str(config)], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
