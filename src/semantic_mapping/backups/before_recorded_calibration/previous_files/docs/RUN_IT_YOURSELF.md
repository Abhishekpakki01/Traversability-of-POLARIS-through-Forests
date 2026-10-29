# Run the semantic mapping yourself

Run these commands in Ubuntu/WSL terminals. The model and ROS package are already
on this PC. ChatGPT is not required to run the application.

## What the program does

1. Reads LiDAR and recorded vehicle/sensor transforms from the RELLIS bag.
2. Accumulates occupied voxels in the fixed `odom` frame using the existing OHM mapper.
3. Uses the pretrained GOOSE model to predict terrain classes in camera images.
4. Uses timestamps, camera calibration and LiDAR depths to associate predictions
   with measured occupied voxels. Complete occlusion accuracy is not yet established.
5. Accumulates class evidence over images and publishes the colored voxel map.
6. Displays the camera, predictions, vehicle pose and occupied map in RViz.

Gray boxes are occupied voxels with an unknown semantic class. They are not free
space. Green means vegetation, red obstacle, brown ground, dark brown trunk,
and blue water. A color is a model prediction, not a guarantee of safe driving.

## Terminal 1 — mapper, camera processing and RViz

Stop an existing mapper before starting another. Copy this whole block:

```bash
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
export ROS_DOMAIN_ID=101
export ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping semantic_integration.launch.py
```

Leave this terminal open. Wait for the semantic node's `Ready` message.
An empty RViz view before bag playback is expected.

## Terminal 2 — replay the recording

Copy this whole block into a second Ubuntu terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
export ROS_DOMAIN_ID=101
export ROS_LOCALHOST_ONLY=1
ros2 bag play ~/RELLIS-3D/example_filtered_ros2 \
  --clock --delay 3 --rate 0.25 --read-ahead-queue-size 10 \
  --qos-profile-overrides-path ~/semantic_map_ws/artifacts/semantic_integration_2026_10_26/playback_qos.yaml \
  --topics /tf /tf_static /os1_cloud_node/points \
    /pylon_camera_node/image_raw /pylon_camera_node/camera_info
```

This plays the one-minute recording at quarter speed, taking about four minutes.
The arrow follows the recorded vehicle pose once the vehicle starts moving.
When playback finishes, the last images and map remain visible.

`--clock` supplies recorded simulation time. The topic list supplies geometry,
poses, camera data and calibration. The QoS file lets ROS subscribers receive
the recorded messages. The small read-ahead queue limits playback memory use.

## Terminal 3 — verify that it is working

Copy this setup block into a third Ubuntu terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
export ROS_DOMAIN_ID=101
export ROS_LOCALHOST_ONLY=1
ros2 node list
```

While the launch and bag run, look for `ohm_occupancy_pose_node`,
`camera_color_preview`, `camera_semantic_voxel`, `rviz2`, and `rosbag2_player`.

Check the semantic status:

```bash
ros2 topic echo /camera_semantic_voxel/status --once --qos-durability transient_local
```

Run it again after a few seconds. `fused_frames` should increase while data is
being processed. `class_counts` lists the map's classified and unknown voxels.
Most voxels can remain unknown because they are outside the camera view,
insufficiently observed, or uncertain. `dropped` can increase under CPU load.

Check the colored map's coordinate frame:

```bash
ros2 topic echo /semantic_voxel_map --field header --once --qos-durability transient_local
```

It should say `frame_id: odom`. RViz should also use `odom` as Fixed Frame and
show Global Status OK. Camera reference and Predicted terrain classes should
both receive images. The colored cloud contains only the occupied OHM geometry.

## Stop and restart

1. Press Ctrl+C in Terminal 2 to stop playback if it is still running.
2. Press Ctrl+C in Terminal 1 to stop mapping, image processing and RViz.
3. To replay from the beginning, run Terminal 1 again, wait for Ready, then
   run Terminal 2 again.

Do not add `--loop`. Restarting the mapper gives the replay a fresh map and
avoids combining evidence across a backwards clock jump.

## Build and run the regression tests

The package is already built. Use these commands after editing code, or to
repeat the checks. Run them after stopping your preview.

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select semantic_mapping --symlink-install
source install/setup.bash
colcon test --packages-select semantic_mapping --return-code-on-test-failure
colcon test-result --test-result-base build/semantic_mapping/test_results --verbose
```

The verified result was 20 tests, 0 errors, 0 failures, 0 skipped. These tests
check mapping, projection and fusion logic; they do not measure recognition
accuracy on ground-truth RELLIS annotations.

## Repeat the full bag check without RViz

This is the automated integration command used during development. It starts
and stops its own mapper and player, runs for about 130 seconds, and saves its
own report and images. Stop your interactive preview first.

```bash
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 python3 \
  ~/semantic_map_ws/src/semantic_mapping/tools/check_semantic_integration.py \
  --seconds 130 --rate 0.5 \
  --output ~/semantic_map_ws/artifacts/my_semantic_check
```

Read its result:

```bash
cat ~/semantic_map_ws/artifacts/my_semantic_check/report.json
```

Look for `passed: true` and `occupied_geometry_matched: true`. The script also
checks that classified voxels have at least two observations and confidence
at least 0.6. Exact counts vary with CPU load and message delivery.

The developer's final test covered 55.29 m of recorded motion. It ended with
33,511 occupied voxels, of which 315 met the semantic thresholds. Coverage is
still limited; the integration is a working baseline, not a completed
traversability or accuracy evaluation.

## Files added or changed for this integration

| File | Change and purpose |
| --- | --- |
| `CMakeLists.txt` | Only existing package file changed: two added lines install the new node and register its tests. |
| `scripts/camera_semantic_voxel_node.py` | New node connecting camera predictions, timestamped poses and occupied voxels. |
| `semantic_mapping/goose_model.py` | New loader for the local pretrained GOOSE model. |
| `semantic_mapping/semantic_fusion.py` | New occupied-voxel class evidence and optional strict ray traversal. |
| `launch/semantic_integration.launch.py` | New launch starting the three processing nodes and RViz. |
| `config/semantic_integration.rviz` | New RViz layout. |
| `config/goose_classes.json` | New explicit mapping of 64 model classes to six displayed groups. |
| `test/test_semantic_fusion.py` | Eleven new test cases. |
| `tools/check_semantic_integration.py` | New automatic bag verification and saved outputs. |

Additional new files document dependencies, licensing, model provenance and run
instructions. The model is in `~/semantic_map_ws/models/goose/`. PyTorch was
already installed. A PDF reader used to extract the official class table was
installed only in `models/goose/tools/`; running ROS does not need that reader.

The existing OHM C++ files, Python controllers, prior camera projection script,
prior launch files and prior RViz profiles were not edited during this stage.

## Backup and previous working view

There is a verified snapshot of all 25 previous package files:

```text
~/semantic_map_ws/src/semantic_mapping/backups/before_semantic_integration/previous_files/
```

The same snapshot is in:

```text
~/semantic_map_ws/src/semantic_mapping/backups/before_semantic_integration/semantic_mapping_before_integration.zip
```

The copies were checked against the SHA-256 hashes recorded before integration.
The previous CMakeLists.txt and package.xml also have separate `.original` copies
in that backup directory. This is a package-source backup, not a duplicate of
the dataset or entire workspace.

To use the previous green voxel view, stop Terminals 1 and 2, then run this in
Terminal 1 (the same ROS setup and exports still apply):

```bash
ros2 launch semantic_mapping mapping_reference.launch.py dataset:=rellis
```

Run Terminal 2's playback command again. No file copying or restoring is needed
to use that previous view.

Water/trunk recognition, complete occlusion accuracy, heights above local ground,
slope, clearance and route safety still need evaluation or further development.
The current colors are predictions; gray boxes correctly show occupied geometry
whose terrain class has not been established.
