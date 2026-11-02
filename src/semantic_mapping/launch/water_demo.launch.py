"""Replay the saved map with LiDAR-only classes and the camera-projected water layer.

No inference, terrain or feature computation runs during playback. Vehicle limits behind
the colours are placeholders, not POLARIS specifications.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    shared = Path(get_package_share_directory('semantic_mapping'))
    default = Path.home()/'Traversability-of-POLARIS-through-Forests/artifacts/water_demo_2026_11_02/replay'
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value=str(default)),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='semantic_mapping', executable='offline_demo_view.py',
             parameters=[{'radius_m': 15., 'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}],
             arguments=['-d', str(shared/'config/water_demo.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        ExecuteProcess(cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
            '--clock', '--delay', '4', '--rate', LaunchConfiguration('rate'),
            '--read-ahead-queue-size', '100', '--disable-keyboard-controls'], output='screen')
    ])
