# Second-terrain test: TartanDrive 2.0 meadow and wooded track

This is the separate test requested by Jere: run the existing mapping and image
fusion code on a different recording, with that recording's own poses and sensor
calibration. It does not replace the working RELLIS setup.

## What was run

- Dataset: [TartanDrive 2.0](https://github.com/castacks/tartan_drive_2.0).
- Recording: `2023-11-02-16-00-43_down_meadows/down_meadows_0.bag`.
- Requested interval: 45–75 seconds after the source bag starts, plus one second
  on each side for synchronization. This is a moving excerpt, not the whole
  dataset or the full two-minute source recording.
- Local ROS 2 bag: `~/thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2`.
- Contains 320 Velodyne scans, 320 rectified color images, 308 recorded LiDAR
  odometry messages, 308 transforms generated from those same poses, and two
  static-transform messages.
- Approximately 940 MB was transferred using selective HTTP ranges. The source
  bag is 32.50 GB; the dataset's GUI downloader was not used.
- The C++ OHM mapper, semantic model, projection code, semantic fusion code,
  occupancy resolution (0.35 m), and maximum range (10 m) were kept unchanged.

## Run it yourself

Use Ubuntu/WSL terminals. These commands use ROS domain 104 to keep this test
separate from another ROS session. Use the same domain in every terminal below.
Close the RViz window left from the demonstration before starting a fresh launch.

### Build once if needed

The build has already succeeded on this PC. After future source changes:

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select semantic_mapping --symlink-install
source install/setup.bash
```

### Terminal 1 — mapper, camera adapter, semantic inference, and RViz

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=104
export ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping tartan_semantic.launch.py
```

Wait for `Ready: camera classes -> measured visible occupied surface` in this
terminal before starting the bag. RViz may be empty until playback starts.

### Terminal 2 — play the new recording

The slower verified run used half speed and produced more confirmed labels.
Playback takes about 64 seconds:

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=104
export ROS_LOCALHOST_ONLY=1
ros2 bag play ~/thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2 \
  --clock --delay 3 --rate 0.5 --read-ahead-queue-size 10 \
  --qos-profile-overrides-path src/semantic_mapping/config/tartan_playback_qos.yaml \
  --topics /tf /tf_static /velodyne_1/velodyne_points \
  /multisense/left/image_rect_color /lidar_odometry
```

Do not add `--loop`. To replay from the beginning, stop Terminal 1 with Ctrl+C,
start it again, wait for Ready, and run Terminal 2 again. This creates a fresh map
and avoids carrying evidence across a backward clock jump.

### Terminal 3 — optional checks while it runs

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=104
export ROS_LOCALHOST_ONLY=1
ros2 topic echo /camera_semantic_voxel/status --once --qos-durability transient_local
```

Look for `fused_frames`, `class_counts`, `processing_seconds`, and `dropped`.
Gray voxels have insufficient semantic evidence. They are still occupied voxels,
not free space. Confirm a pose during playback with:

```bash
ros2 topic echo /vehicle_pose --once
```

RViz should already have Fixed Frame `sensor_init`, Target Frame `vehicle`, a red
pose arrow, the occupied semantic voxel cloud, the original camera reference,
and a predicted-terrain image. Enable **Projection alignment reference** to see
the LiDAR-to-image overlay. The colored map is a `PointCloud2` display with
`RGB8`; the original occupancy topic remains available as a Marker.

## Actual results

Saved outputs:
`~/semantic_map_ws/artifacts/second_terrain_tartan_meadows/half_speed_replay/`.
The initial run is also retained in `native_replay/` beside it.

| Measurement | Native speed | Half speed |
|---|---:|---:|
| Span of received vehicle poses | 31.66 s | 31.66 s |
| First-to-last pose displacement | 81.03 m | 81.03 m |
| Occupied map messages received | 115 | 174 |
| Camera/LiDAR pairs handled by semantic node | 144 | 115 |
| Fused semantic frames | 19 | 29 |
| Dropped pairs | 125 | 86 |
| Final occupied voxels | 23,662 | 24,888 |
| Voxels with confirmed class | 324 (1.37%) | 569 (2.29%) |
| Vegetation / ground | 299 / 25 | 530 / 39 |
| Trunk / water / obstacle | 0 / 0 / 0 | 0 / 0 / 0 |
| Unknown occupied voxels | 23,338 | 24,319 |
| Colored geometry equals OHM occupied geometry | Yes | Yes |

The last worker durations were 1.29 s (native) and 6.04 s (half speed); these are
individual measurements, not average latencies or end-to-end frequencies.
Performance varies substantially with scheduling/load. Inference uses the CPU
on this PC. The 101.29 m sampled path length in the first run includes a turn;
81.03 m is the endpoint displacement, not total distance travelled.
Only voxels with at least two observations and confidence at least 0.6 receive a
class color. These thresholds do not make confidence statistically calibrated.

`report.json` contains measured results; `semantic_voxels.npy` contains actual
ROS output fields; `vehicle_poses.npy` contains the observed trajectory;
`semantic_voxel_map.png` is a headless plot of those outputs. Camera,
segmentation, and projection images are also saved. The initial run's image
files were independently the latest messages, so they can differ slightly in
timestamp. The updated check script saves the matching camera exposure for the
last segmentation and reports the image timestamps explicitly. In the half-speed
run all three saved images have timestamp `1698955331917020000` ns.

## How this differs from RELLIS

Tartan uses `/velodyne_1/velodyne_points` rather than the Ouster topic, and
`sensor_init` / `vehicle` rather than `odom` / `base_link`. The separate launch
remaps topic names and selects these frames without editing the mapper.

The source bag has no CameraInfo stream. We used the dataset publisher's
[intrinsics](https://raw.githubusercontent.com/castacks/tartan_drive_2.0/main/calibration/multisense_intrinsics.txt)
and [extrinsics](https://raw.githubusercontent.com/castacks/tartan_drive_2.0/main/calibration/extrinsics.yaml).
The camera images are already rectified: the adapter takes the intrinsic 3×3
block of P, uses zero distortion for these rectified pixels, and includes R's
inverse in a new virtual optical-frame transform. It preserves the image pixels
and exposure timestamps.

The publisher's S21 camera extrinsic differs from the bag's default camera URDF
offset (0.105 m versus 0.035 m laterally). The adapter explicitly uses the
publisher calibration in `tartan_left_rectified_optical`; it does not combine
that intrinsic calibration with the conflicting recorded optical transform.
The saved projection overlay permits visual inspection. Replay and geometry
unit checks do not establish the calibration's numerical accuracy for this
particular recording.

`sensor_init -> vehicle` transforms are generated from the source
`/lidar_odometry` values and original pose timestamps. For this offline bag they
are dispatched 1 ms before their pose timestamps, rather than at the source
odometry computation's later arrival time. Pose values and acquisition times
are not changed. Thus this is an offline mapping/fusion test, not a benchmark of
online pose-estimation latency. TF interpolates available poses at each sensor
timestamp; scans without a usable scan-time pose are skipped.

## New files and their purpose

| New file under `src/semantic_mapping/` | Purpose |
|---|---|
| `tools/extract_remote_tartan.py` | Reads the public ROS 1 bag by byte ranges, converts selected streams to ROS 2, and writes recorded pose values as TF. Uses pinned rosbags 0.11.3 in `.dataset_venv`. |
| `config/tartan_meadows_camera.yaml` | Stores the publisher calibration, image dimensions, source URLs, and calibration caveat. |
| `semantic_mapping/tartan_camera.py` | Computes rectified camera geometry and valid rotation quaternions. |
| `scripts/tartan_camera_adapter.py` | Publishes rectified-image CameraInfo and explicit static extrinsics with the original image timestamps. Checks source dimensions/frame. |
| `launch/tartan_semantic.launch.py` | Starts the unchanged mapper and semantic node with dataset-specific frames and topic remaps. |
| `config/tartan_semantic.rviz` | Opens the second terrain with the correct fixed frame, pose, images, and semantic voxels. |
| `config/tartan_playback_qos.yaml` | Sets LiDAR best-effort and static-TF transient-local QoS for replay. |
| `tools/check_second_terrain.py` | Runs a moving replay without RViz, verifies occupied geometry and label thresholds, requires >25 s and >10 m, and saves measured outputs. |
| `test/test_tartan_camera.py` | Six checks for rectified intrinsics, transform composition, quaternion conversion, and invalid calibration. |
| `docs/SECOND_TERRAIN_TARTAN.md` | This explanation and the terminal commands. |

Only two lines were added to the existing `CMakeLists.txt`: install the new
adapter and register the calibration test. Existing C++/Python mapping and
semantic files, RELLIS launch files, package.xml, and model weights are unchanged
in this step. During the session, the existing RELLIS RViz profile was also saved
with a different camera view, panel layout, and projection display enabled. Its
mapping frame and data topics remain the RELLIS ones. That view change is kept;
both the earlier profile and the saved layout have backups.

## Verify the saved result yourself

Stop any launch using domain 103 first, then run this single-terminal check:

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=103
export ROS_LOCALHOST_ONLY=1
python3 src/semantic_mapping/tools/check_second_terrain.py \
  --seconds 80 --rate 0.5 --output artifacts/tartan_my_check
```

It starts and stops its own mapper and player. Read its `report.json`; a `passed`
result checks the stated functional conditions, not semantic accuracy. Counts
can vary with CPU load and scheduling. The calibration and existing mapping
tests also passed: 26 tests, zero failures, errors, or skipped tests.

## Backup and returning to RELLIS

Verified copies of all 39 pre-existing project files are in:

`~/semantic_map_ws/src/semantic_mapping/backups/before_second_terrain/previous_files/`

The same files are in `semantic_mapping_before_second_terrain.zip`, beside the
SHA-256 manifest `preserved_files.json` and separate `.original` build/package
files. The intentional source edit is the two CMakeLists.txt additions. The final
comparison also detects the saved RELLIS RViz view described above, so it does
not claim that every configuration file stayed byte-identical. The saved view
is copied separately as `rellis_saved_layout_2026_10_27.rviz` in the backup
folder; the verified pre-step profile stays in `previous_files/config/` and in
the ZIP.

To return to RELLIS, stop the Tartan launch and player. Follow
`docs/RUN_IT_YOURSELF.md` with `semantic_integration.launch.py` and the RELLIS bag.
No restoration or file replacement is needed to run the previous setup.

## What the colors mean for navigation

Green is predicted vegetation, brown ground, dark brown trunk, blue water,
red obstacle, and gray unknown. `/semantic_voxel_map` also carries class label,
confidence, observation count, and metric XYZ coordinates. A future planner
can read those fields; it should not infer safety just from display RGB.

This run demonstrates cross-dataset integration and multi-frame semantic color
updates. Most geometry remains semantically unknown. Image predictions include
questionable regions under glare; an earlier preview even predicted a blue
patch on the vehicle hood. There are no semantic ground-truth labels in this
extracted recording, and this test establishes no detection accuracy for
water or trunks. Neither class obtained confirmed voxel labels in the initial
run or the half-speed run. Do not claim reliable water avoidance or forest
navigation from it.

Before navigation: improve processing coverage and TF availability; check
projection against measured landmarks; evaluate class errors against annotated
examples; derive ground-relative height, slope, roughness, and vehicle clearance;
then convert semantic/geometry evidence into a conservative traversability cost
map and test a planner. Metric Z already exists, but it is height in sensor_init,
not yet height above a local ground estimate. No planner or robot motion command
was added in this step.

Source selection and conversion provenance is saved under
`~/thesis_data/tartan_drive_2_sample/source_metadata/`, particularly
`down_meadows_moving.json`, `extrinsics.yaml`, and `multisense_intrinsics.txt`.

## Optional: reproduce the data extraction

The required data is already on this PC. This is only for reproducing the
download/conversion into a new folder; normal visualization uses Terminals 1–2.
The isolated `.dataset_venv` already contains access to rosbags 0.11.3. The
extractor uses that version's reader internals and is specific to these ROS 1
bag records. It enforces a 1.5 GB transfer limit and requires exact HTTP range
support from the server. It does not download the source bag in full.

```bash
cd ~/semantic_map_ws
TARTAN_MEADOWS_SOURCE='https://airlab-cloud.andrew.cmu.edu:8080/swift/v1/AUTH_ac8533a83cff4d48bc8c608ad222d330/tartandrive2/bags/2023-11-02-16-00-43_down_meadows/down_meadows_0.bag'
.dataset_venv/bin/python src/semantic_mapping/tools/extract_remote_tartan.py \
  --url "$TARTAN_MEADOWS_SOURCE" --offset 45 --seconds 30 \
  --metadata ~/thesis_data/tartan_drive_2_sample/source_metadata/down_meadows_repeat.json \
  --output ~/thesis_data/tartan_drive_2_sample/down_meadows_repeat_ros2
```

The output folder must not already exist. Do not replace the verified bag;
choose a different new folder if repeating this command.
