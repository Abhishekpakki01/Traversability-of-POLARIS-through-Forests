import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define the launch configuration variable
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2. Declare the launch argument for command line overriding
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,

        # 1. C++ OHM GPU/CPU Occupancy Engine Node
        Node(
            package='semantic_mapping',
            executable='ohm_occupancy_node',
            name='ohm_occupancy_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'max_range': 10.0
            }]
        ),

        # 2. Python Semantic / Voxel Node
        Node(
            package='semantic_mapping',
            executable='standalone_voxel_node.py',
            name='python_semantic_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),

        # 3. RViz2 Visualization Window
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        )
    ])