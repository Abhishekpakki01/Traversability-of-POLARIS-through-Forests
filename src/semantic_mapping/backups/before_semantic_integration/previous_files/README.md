# Pose-aware OHM occupancy mapping

The updated source is `src/ohm_occupancy_pose_node.cpp`, built as
`ohm_occupancy_pose_node`. The original `src/ohm_occupancy_node.cpp` and launch
file are preserved unchanged. The original executable remains available as
`ohm_occupancy_node`.

`ohm_occupancy_pose_node` integrates scans into `odom` by default. For every cloud,
it looks up `fixed_frame <- cloud.header.frame_id` at `cloud.header.stamp`.
It transforms both LiDAR endpoints and the sensor origin before passing the
origin/endpoint pairs to OHM. Range filtering stays in the sensor frame.
Scans without a timestamped transform are skipped with a warning; the node
does not substitute the latest pose.

The provided RELLIS bag contains dynamic `odom -> base_link` TF and a static
chain from `base_link` to `ouster1/os1_lidar`. `/odometry/filtered` also contains
the `base_link` pose in `odom`; this node uses the recorded TF chain, including
the LiDAR mounting offset, instead of creating a second pose source.

Outputs:

- `/semantic_voxel_map_cpp`: one `visualization_msgs/msg/Marker` (`CUBE_LIST`)
  containing only voxels classified occupied by OHM. Unknown cells use
  positive infinity in OHM and are explicitly excluded by its `isOccupied()`
  helper. Free cells still receive ray updates internally, but are not rendered.
- `/vehicle_pose`: `geometry_msgs/msg/PoseStamped` for `base_link` in `odom`,
  at the integrated cloud's timestamp. `fixed_frame` and `base_frame` are ROS
  parameters; their defaults are `odom` and `base_link`.

## Run the C++ mapper

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run semantic_mapping ohm_occupancy_pose_node --ros-args -p use_sim_time:=true
```

In a second sourced terminal, replay the recorded TF and LiDAR without looping:

```bash
ros2 bag play ~/RELLIS-3D/example_filtered_ros2 --clock \
  --qos-profile-overrides-path ~/semantic_map_ws/qos_override.yaml \
  --topics /tf /tf_static /os1_cloud_node/points
```

In RViz2, enable simulation time, set **Fixed Frame** to `odom`, and add:

- **Marker** on `/semantic_voxel_map_cpp` for occupied voxels.
- **Pose** on `/vehicle_pose` for the vehicle position and heading.
- **TF** if you want to inspect the vehicle-to-sensor chain.

The existing `semantic_mapping.launch.py` still runs the original C++ executable
and the separate Python mapper. Use the standalone command above for this variant.
Run only one C++ mapper at a time because both use `/semantic_voxel_map_cpp`.
If the bag is restarted or seeked backwards, restart the mapper for a fresh
map. Automatic replay reset handling is outside this change.

## Return to the original version

Stop the updated mapper with Ctrl+C, restart the bag from the beginning, then run:

```bash
ros2 run semantic_mapping ohm_occupancy_node --ros-args -p use_sim_time:=true
```

This runs the original source, with its original behavior. For a complete build-file
rollback, the original `CMakeLists.txt` and `package.xml` are also saved under
`backups/before_pose_mapping/` with `.original` suffixes, alongside a source and
launch snapshot. Those snapshots were taken before this change.

## Build and test

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select semantic_mapping
colcon test --packages-select semantic_mapping --return-code-on-test-failure
colcon test-result --verbose
```

The tests run the C++ executable with synthetic timestamped TF and clouds.
They check rotation, translation, sensor mounting offset, pose interpolation,
multi-scan alignment, missing-pose rejection, occupied-only output, free-space
clearing, and ray origins following the sensor.
