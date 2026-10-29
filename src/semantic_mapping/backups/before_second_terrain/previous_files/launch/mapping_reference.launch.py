"""Separate reference views; the original semantic_mapping launch is unchanged."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_nodes(context):
    dataset = LaunchConfiguration('dataset').perform(context)
    if dataset not in ('rellis', 'tartan'):
        raise ValueError('dataset must be rellis or tartan')
    tartan = dataset == 'tartan'
    shared = {'use_sim_time': True}
    nodes = [Node(
        package='semantic_mapping', executable='ohm_occupancy_pose_node', output='screen',
        parameters=[dict(shared, fixed_frame='sensor_init' if tartan else 'odom',
                         base_frame='vehicle' if tartan else 'base_link')],
        remappings=[('/os1_cloud_node/points', '/velodyne_1/velodyne_points')] if tartan else [])]
    if not tartan:
        nodes += [Node(package='semantic_mapping', executable=executable,
                       output='screen', parameters=[shared])
                  for executable in ('camera_color_preview.py', 'camera_lidar_projection_node.py')]
    config = Path(get_package_share_directory('semantic_mapping'))/'config'/(dataset+'_reference.rviz')
    nodes.append(Node(package='rviz2', executable='rviz2', parameters=[shared],
                      arguments=['-d', str(config)],
                      condition=IfCondition(LaunchConfiguration('rviz'))))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dataset', default_value='rellis'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=start_nodes),
    ])
