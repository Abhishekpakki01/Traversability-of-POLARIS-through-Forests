import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. C++ OHM GPU/CPU Occupancy Engine Node
        Node(
            package='semantic_mapping',
            executable='ohm_occupancy_node',
            name='ohm_occupancy_node',
            output='screen',
            parameters=[{'max_range': 10.0}]
        ),

        # 2. Python Semantic / Voxel Node
        Node(
            package='semantic_mapping',
            executable='standalone_voxel_node.py',
            name='python_semantic_controller',
            output='screen'
        ),

        # 3. RViz2 Visualization Window
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])