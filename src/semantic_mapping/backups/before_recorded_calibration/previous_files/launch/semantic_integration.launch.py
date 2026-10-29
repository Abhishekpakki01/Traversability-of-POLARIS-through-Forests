"""RELLIS camera semantics and OHM occupied voxels; original launches are preserved."""

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
    config = str(Path(get_package_share_directory('semantic_mapping'))/'config/semantic_integration.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('model_path', default_value=str(Path.home()/
            'semantic_map_ws/models/goose/ppliteseg_class_512.pth')),
        DeclareLaunchArgument('min_confidence', default_value='0.6'),
        DeclareLaunchArgument('min_observations', default_value='2'),
        DeclareLaunchArgument('strict_voxel_occlusion', default_value='false'),
        Node(package='semantic_mapping', executable='ohm_occupancy_pose_node',
             parameters=[shared], output='screen'),
        Node(package='semantic_mapping', executable='camera_color_preview.py',
             parameters=[shared], output='screen'),
        Node(package='semantic_mapping', executable='camera_semantic_voxel_node.py',
             parameters=[dict(shared,
                model_path=LaunchConfiguration('model_path'),
                min_confidence=ParameterValue(LaunchConfiguration('min_confidence'), value_type=float),
                min_observations=ParameterValue(LaunchConfiguration('min_observations'), value_type=int),
                strict_voxel_occlusion=ParameterValue(
                    LaunchConfiguration('strict_voxel_occlusion'), value_type=bool))],
             output='screen'),
        Node(package='rviz2', executable='rviz2', parameters=[shared], arguments=['-d', config],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
