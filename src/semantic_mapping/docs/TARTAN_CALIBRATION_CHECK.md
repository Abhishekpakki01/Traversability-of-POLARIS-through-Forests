# Tartan camera–LiDAR correction and check

This is a separate version. The previous `tartan_semantic.launch.py`, its camera
adapter and calibration file are retained. The original working workspace at
`~/semantic_map_ws` has not been edited.
This configuration is for the named Tartan recording, not the physical POLARIS
sensor installation or another recording.

## What changed

The previous adapter used a later publisher example for both the camera mount
and optical-frame offset. The source recording contains different transforms:

| Transform | Previous values | Values from this recording |
|---|---|---|
| Vehicle → camera head translation | `[-0.0949, 0.0641, -0.136]` m | `[0, 0, -0.1274]` m |
| Vehicle → camera head rotation | Publisher quaternion | Original `/multisense_to_base` quaternion |
| Head → left camera origin | `[0.005, 0.105, 0]` m | `[0, 0.035, -0.002]` m, then optical-axis rotation |

The vehicle-to-head transforms differ by 0.11484 m and 1.62183°. The new version
composes the original `vehicle → head → left_camera_frame → optical_frame`
chain. It keeps the recorded identity relationship between vehicle and Velodyne 1.

The images are already rectified. The new version uses rectified P, zero
additional distortion and the recorded rectified optical-frame convention. It
does not add the later example's rectification rotation. The generic raw K/D
and later R are kept in the old configuration for comparison.

There is no CameraInfo connection in the original source bag. The parameter
file saved with this exact drive contains the resolution and robot_description,
but no camera matrix. We therefore also checked the original stereo output
against its matching camera exposure. This supports the rectified projection
matrix; it does **not** establish factory camera calibration or physical
LiDAR-to-image reprojection accuracy. `intrinsics_verified` remains false for
that reason. No parameters were fitted by eye to produce more colors.

## Run the new version

Use Ubuntu/WSL terminals. Stop another mapper in domain 111 before starting.

### Terminal 1 — build and open the projection check

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
colcon build --packages-select semantic_mapping --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=111
export ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping tartan_recorded_calibration.launch.py
```

The package is already built on this PC, so later starts can omit the build
line. Wait for `Camera projection reference` before starting Terminal 2.

### Terminal 2 — replay the same moving excerpt

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=111
export ROS_LOCALHOST_ONLY=1
ros2 bag play ~/thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2 \
  --clock --delay 3 --rate 0.5 --read-ahead-queue-size 10 \
  --qos-profile-overrides-path src/semantic_mapping/config/tartan_playback_qos.yaml \
  --topics /tf /tf_static /velodyne_1/velodyne_points \
  /multisense/left/image_rect_color /lidar_odometry
```

The excerpt takes about 64 seconds at half speed. Do not add `--loop`. Stop both
terminals with Ctrl+C and restart the launch before replaying from the start.

### Terminal 3 — inspect the projection status

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=111
export ROS_LOCALHOST_ONLY=1
ros2 topic echo /camera_lidar_projection/status --once
```

RViz uses Fixed Frame `sensor_init` and follows `vehicle`. It shows the original
camera, depth-colored LiDAR projection, occupied geometry and red vehicle pose.
The projection diagnostic includes returns up to 30 m to make distant structure
visible. OHM's map range remains 10 m and its voxel size remains 0.35 m.

In this check, the uniform green occupied boxes are **geometry display colors**,
not vegetation labels. Inspect whether projected points follow visible ground,
trunks and object boundaries at several timestamps. One plausible overlay does
not establish calibration accuracy; quantitative landmark checks are still needed.

## Optional semantic view

The default launch checks projection without loading the segmentation model.
To inspect the same recorded-transform variant with semantic fusion, stop the
launch/player, then use this in Terminal 1 and repeat Terminal 2:

```bash
ros2 launch semantic_mapping tartan_recorded_calibration.launch.py semantic:=true
```

Enable **Optional semantic voxels** in RViz and disable **Occupied OHM geometry**
to see class colors. This mode uses the semantic node's 10 m projection overlay
and stops the separate 30 m projection worker to avoid duplicate processing.
Its counters are on `/camera_semantic_voxel/status`. These predictions remain
unvalidated; frame drops and low
class coverage are separate unfinished problems. This switch does not certify
the calibration.

## Checks that can be repeated

After the setup/source commands above, stop the preview first:

```bash
ROS_DOMAIN_ID=110 ROS_LOCALHOST_ONLY=1 python3 \
  src/semantic_mapping/tools/check_recorded_projection.py \
  --output artifacts/calibration_my_check
```

This runs about 80 seconds, starts/stops its own nodes/player, and saves a report,
matching camera/overlay images and received poses. Its pass conditions are data
flow, matching image timestamps and movement, not physical calibration accuracy.

Offline comparisons do not start ROS nodes or replay:

```bash
python3 src/semantic_mapping/tools/compare_tartan_calibration.py \
  --output artifacts/calibration_my_comparison
python3 src/semantic_mapping/tools/check_tartan_stereo_intrinsics.py \
  --reference artifacts/calibration_review_2026_10_29/stereo_reference \
  --output artifacts/calibration_my_stereo_check.json
```

For a separate semantic integration check using the recorded-transform variant:

```bash
ROS_DOMAIN_ID=110 ROS_LOCALHOST_ONLY=1 python3 \
  src/semantic_mapping/tools/check_recorded_semantics.py \
  --seconds 80 --rate 0.5 --output artifacts/calibration_my_semantic_check
```

The regression suite now has **33 passing tests**: the previous 26 plus seven
recorded-chain/projection checks. It does not measure physical calibration error.

The moving projection run produced 91 overlays from 92 matched pairs, with
31.56 s of received poses and 80.75 m endpoint displacement. Only 114 of the
320 source camera images reached the projection node's counter, so the single
post-pairing drop does not mean lossless input delivery.

Two semantic runs of the new variant fused 27 and 24 frames and produced 522
and 460 classified voxels respectively. The final check saved camera,
segmentation and projection at the same exposure (`1698955333916900000` ns),
and verified that colored geometry matched OHM occupied geometry. These are
functional results with substantial frame loss, not an accuracy comparison
against the previous calibration.

## Saved evidence and interpretation

`artifacts/calibration_review_2026_10_29/` contains:

- `frame_comparison/`: five identical camera/scan pairs, each with old/new overlays
  and acquisition-time pose compensation. Median changes in projected position
  are about 13.6–18.1 pixels across frames. These are **model differences**, not
  measured errors or claimed error reductions.
- `stereo_reference/` and `stereo_consistency.json`: original stereo cloud and its
  exact exposure at `1698955259317091000` ns. Across 149,901 stereo points, median
  channel-averaged color differences were 5.33/255 for rectified P, 19.0/255 for
  raw K, and 13.67/255 when adding the later example's R. This checks consistency
  with the original stereo pipeline, not independent LiDAR calibration accuracy.
- `ros_replay/`: the live ROS projection test report and outputs.

`data_provenance/tartan_calibration_review/` contains the recorded TF evidence.
The original parameter file's SHA-256 is recorded in the new YAML. Source files
were fetched from the publisher's public recording using bounded HTTP reads.

## Files and return to the previous version

New files: `recorded_calibration.py`, `tartan_recorded_camera_adapter.py`,
`tartan_recorded_calibration.yaml`, `tartan_recorded_calibration.launch.py`,
`tartan_recorded_calibration.rviz`, seven tests, four check/comparison tools and
this guide. Only two lines were added to the existing CMakeLists.txt, to install
the new adapter and register its tests. Old numerical mapper/model/fusion and
camera-adapter code was not changed.

The verified backup of 49 pre-change package files is under
`src/semantic_mapping/backups/before_recorded_calibration/`, as `previous_files/`,
a ZIP and SHA-256 manifest. Its original-workspace hash manifest can also verify
that the original workspace stayed untouched.

To return to the previous behavior, stop the new launch/player and run
`tartan_semantic.launch.py` with its earlier playback commands. No source files
need to be restored.
