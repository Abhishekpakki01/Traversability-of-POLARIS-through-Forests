# Thesis project handoff — 29 October 2026

**Project:** Pose-aware 3D semantic voxel mapping for off-road terrain, with forest-navigation applications for the POLARIS vehicle.

**Owner:** Abhishek. **Supervisors:** Jere and Arto. **User's target completion date:** 23 November 2026.

This is a self-contained handoff for the owner or another coding agent. It describes the actual local files and saved results, including the unfinished parts. It was prepared from read-only source inspection and saved test reports on 29 October. No mapping/model/launch code was changed to prepare this handoff.


## Later update on 29 October: separate recorded-transform correction

The initial handoff below is retained as historical context. A separate variant
was subsequently implemented in the GitHub checkout at
`~/Traversability-of-POLARIS-through-Forests`; the original `~/semantic_map_ws`
source is unchanged. Read
`src/semantic_mapping/docs/TARTAN_CALIBRATION_CHECK.md` for the new commands.

- New launch: `tartan_recorded_calibration.launch.py`; projection-only by default,
  with optional `semantic:=true`. Previous launches and calibration are retained.
- Restores the original vehicle/head/camera optical transform chain. The earlier
  head mount differed by 11.5 cm and 1.62 degrees.
- Uses rectified P and the recorded rectified-frame convention, without the
  later example's extra R. Original stereo-cloud/matching-image color checks
  support this choice; they do not prove physical LiDAR calibration accuracy.
- The original bag has no CameraInfo. Physical calibration remains unverified;
  the configuration explicitly records that limitation.
- Current regression total: 33 passing tests. The moving projection check
  produced 91 overlays, with matching camera stamps, 31.56 s pose span and
  80.75 m endpoint displacement. This was not lossless processing of all inputs.
- Backup: 49 pre-change package files under
  `src/semantic_mapping/backups/before_recorded_calibration/`.
- New evidence: `artifacts/calibration_review_2026_10_29/` and
  `data_provenance/tartan_calibration_review/`. No geometric ground-truth
  reprojection error was measured. Existing frame-loss and semantic-accuracy
  problems remain separate work.

## 1. Read this first

- The pipeline runs locally in Ubuntu/WSL. ROS, OHM, the local PyTorch model, and RViz perform the work; this chat is not part of the running pipeline.
- The system is an **initial working integration**, not a completed or validated navigation system. Semantic coverage is sparse and results vary substantially between runs.
- The user requires **small, explained changes, separate experimental files where practical, and verified backups before editing working files**. Do not rewrite everything or remove earlier working versions.
- Explain what was changed, why, exactly which terminal commands to run, what the output means, and how to return to the previous view.
- Jere requested a test in another environment, then camera-to-voxel fusion. A moving TartanDrive excerpt has now been tested, and initial fusion is implemented. Longer/repeatable evaluation remains unfinished.
- The user had **not previously told Jere about the GOOSE model choice**. Introduce it transparently as the source of pretrained image-segmentation weights. Do not imply that GOOSE recordings were used for the demonstrated mapping test or that the student trained this model.
- The mapping recordings are **RELLIS-3D and TartanDrive 2.0**. The pretrained image model comes from the **GOOSE project**. These are different roles.
- Do not send Teams/email messages. Draft messages for the user to review and send.
- This handoff request authorizes documentation and an export bundle, not additional mapping refactors. The next implementation priority is diagnosing dropped observations before improving coverage.

### Suggested first message to a new agent

> Read this handoff and inspect the referenced local source and reports before changing code. Preserve the working RELLIS and Tartan launches and all originals. Our next task is to measure why camera/LiDAR observations are dropped and make semantic updates repeatable. Start with a separate diagnostic version, explain the proposed limited change, preserve a verified backup, and provide exact commands. Do not treat sparse colors or passing smoke tests as validated forest navigation. Keep me informed of findings and limitations.

If the next agent has this same workspace, the Markdown file is enough to locate everything. If it works on a different machine, give it the source ZIP too; datasets, model weights, and installed ROS dependencies must be supplied or recovered separately.

## 2. Current machine and important paths

| Item | Verified location/state |
|---|---|
| Ubuntu | 22.04.5 LTS under WSL2 |
| Kernel | 6.6.87.2-microsoft-standard-WSL2 |
| ROS | Humble, `/opt/ros/humble` |
| Workspace | `/home/abhishek/semantic_map_ws` |
| Our ROS package | `~/semantic_map_ws/src/semantic_mapping` |
| OHM dependency source | `~/semantic_map_ws/src/ohm`; Git HEAD `4e2e76945e550eb8a96cef9c4434e7c1811dd030` |
| Built workspace setup | `~/semantic_map_ws/install/setup.bash` |
| Python | 3.10.12; normal ROS commands use system Python |
| PyTorch / acceleration | 2.9.0+cu128; `torch.cuda.is_available()` was **false** |
| NumPy / OpenCV | 1.26.4 / 4.11.0 |
| Dataset conversion | rosbags 0.11.3; isolated launcher `~/semantic_map_ws/.dataset_venv/bin/python` |
| Model weights | `~/semantic_map_ws/models/goose/ppliteseg_class_512.pth`, about 94 MiB |
| RELLIS ROS 2 bag | `~/RELLIS-3D/example_filtered_ros2` |
| Moving Tartan ROS 2 bag | `~/thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2` |
| Dataset provenance | `~/thesis_data/tartan_drive_2_sample/source_metadata/` |
| User's demonstration video | `/home/abhishek/Different environment testing.mp4` |

These versions describe the current PC, not a fully specified clean-machine installer. The ZIP does not include the `build/`, `install/`, datasets, model weights, virtual environment, or recording video. Keep the existing machine/workspace intact if possible.

## 3. What the application actually does

```text
Recorded LiDAR + recorded poses/TF
        -> transform rays at scan acquisition time
        -> OHM occupancy evidence in a fixed world frame
        -> occupied voxel centers + vehicle pose

Camera image + calibration
        -> pretrained PP-LiteSeg-B image class probabilities
LiDAR + camera/scan-time poses
        -> project measured LiDAR surfaces into the image
        -> nearest measured-depth visibility filtering
        -> class observations only at corresponding occupied endpoints
        -> accumulate voxel class evidence over images
        -> semantic PointCloud2 -> RViz boxes and class colors
```

LiDAR supplies geometry and measured depth. Camera segmentation supplies appearance-based class predictions. The pose makes measurements from different vehicle locations refer to the same fixed voxels.

For a LiDAR point:

```text
p_world  = T_world_lidar(t_lidar) * p_lidar
p_camera = inverse(T_world_camera(t_image)) * p_world
(u,v)    = camera_projection(p_camera, intrinsics, distortion)
```

The code samples the predicted pixel distribution at `(u,v)` and updates the measured occupied endpoint voxel. It does not paint all empty cells or fill unobserved space behind a measured surface.

### How the camera classes are predicted

1. `GoosePredictor` loads official pretrained **GOOSE PP-LiteSeg-B 64-class weights**. We did not train a new model.
2. `predict()` converts BGR to RGB, scales pixels to `[0,1]`, and resizes to 512×512 using antialiased bilinear interpolation. It does not apply ImageNet normalization.
3. A neural network produces 64 class logits per pixel; softmax converts them to probabilities.
4. An explicit class table groups the 64 probabilities into six display groups. These group IDs are **not the original RELLIS label IDs**.
5. Camera/LiDAR projection, visibility checks, and occupied-cell checks select which voxel observations are usable.
6. `SemanticEvidence` accumulates probability pseudo-counts over different image timestamps. A voxel displays its most probable class after at least **2 observations** and **0.6 posterior confidence**. Otherwise its displayed class is unknown.

| Display ID | Group | RGB |
|---:|---|---|
| 0 | Unknown | 150, 150, 150 |
| 1 | Ground | 139, 90, 43 |
| 2 | Vegetation | 45, 180, 45 |
| 3 | Trunk | 100, 55, 20 |
| 4 | Water | 30, 120, 255 |
| 5 | Obstacle | 230, 50, 50 |

The three image panels are **views of the same camera stream**, not three lenses: original image, predicted-class overlay, and LiDAR projection overlay. Gray boxes are occupied geometry with unknown semantics; they are not free cells. The red arrow is vehicle position and orientation.

OHM integrates occupancy evidence. Semantic fusion currently uses **bounded Dirichlet-style probability pseudo-counts**, not the original proposal's per-class log-odds implementation. Confidence is uncalibrated, and successive frames are correlated. A blue prediction does not establish actual water or navigation safety.

### Model provenance

- Weights: <https://goose-dataset.de/models/ppliteseg_class_512.pth>
- SHA-256: `6dd412c0c99115e359896c4cab43a8e6bce9e09b843e7fa885fe597b0a6121cd`
- Model/preprocessing source: <https://github.com/FraunhoferIOSB/goose_dataset>
- Architecture adapted from SuperGradients, pinned source tree `63de22c404d5740f34f7706c302b37fce3c8fe5d`.
- `models/goose/manifest.json` records URLs/hashes; `parity_check.json` records strict loading of 421 tensors and maximum logit error 0.0 against the upstream forward implementation in that check.
- This implementation check does **not** establish segmentation accuracy on RELLIS or TartanDrive. Preserve `LICENSE.super-gradients` and `THIRD_PARTY_NOTICES.md` when sharing source.

## 4. Code written and the purpose of its main functions

All package paths below are relative to `~/semantic_map_ws/src/semantic_mapping/`.

| File | Role and important functions |
|---|---|
| `src/ohm_occupancy_pose_node.cpp` | Separate C++ pose-aware mapper. `cloudCallback()` rejects zero/missing timestamps, looks up exact scan-time TF, filters finite/range-limited XYZ, transforms both endpoints and sensor ray origins, integrates OHM rays, publishes vehicle pose and occupied-only `CUBE_LIST`. Uses `ohm::isOccupied()` to exclude free/unobserved cells and `voxelCentreGlobal()` for positions. |
| `scripts/camera_color_preview.py` | `on_image()` converts the recorded camera image to BGR for RViz while preserving its acquisition header. RELLIS input may be Bayer encoded. |
| `semantic_mapping/camera_projection.py` | `xyz_from_cloud()` handles structured/mixed-field PointCloud2 data; `transform_matrix()` / `transform_points()` perform geometry; `project_visible_points()` checks range, positive camera depth, image bounds and nearest measured depth; `draw_projection()` draws an inspection overlay. |
| `scripts/camera_lidar_projection_node.py` | Separate appearance/projection reference. `on_camera()` pairs image and CameraInfo; `on_pair()` queues camera/LiDAR pairs; `process_pending()` waits for acquisition-time TF; `project_pair()` publishes overlay and camera-RGB surface samples. These RGB samples are appearance colors, not semantic classes. |
| `semantic_mapping/goose_model.py` | Inference-only PP-LiteSeg-B architecture and `GoosePredictor`. `__init__()` strictly loads checkpoint weights; `predict()` performs the preprocessing, softmax and 64-to-6 probability grouping. No training or remote model API calls are used. |
| `config/goose_classes.json` | Explicit official 64-class ID/name/group table. Do not replace it with the six example RELLIS IDs from the initial project notes. |
| `semantic_mapping/semantic_fusion.py` | `voxel_keys()` computes fixed grid indices. `first_occupied()` implements optional Amanatides–Woo voxel traversal. `surface_observations()` groups usable measured endpoints into one mean class distribution per voxel/image. `SemanticEvidence.update()` accumulates unique-timestamp evidence; `retain_occupied()` removes evidence for cells OHM clears; `query()` applies display thresholds. |
| `scripts/camera_semantic_voxel_node.py` | Main semantic controller. `on_camera()` validates matching calibration; `on_pair()` queues synchronized observations; `on_map()` caches exact-timestamp occupied snapshots; `tick()` schedules work only with matching map and camera/scan-time TF; `process()` predicts, projects and prepares endpoint evidence; `finish()` fuses results; `semantic_cloud()` / `publish_map()` publish inspectable semantic attributes without changing occupied geometry; `publish_status()` publishes counters. |
| `semantic_mapping/tartan_camera.py` | `rectified_geometry()` composes publisher-supplied rectified camera geometry; `pose_matrix()` and `quaternion_from_matrix()` validate/convert transformations. |
| `scripts/tartan_camera_adapter.py` | Separate Tartan adapter. `on_image()` checks dimensions/source frame, preserves pixels/timestamps, supplies matching CameraInfo and an explicit virtual optical frame. `make_transform()` prepares camera and LiDAR mounting transforms. |
| `config/tartan_meadows_camera.yaml` | Publisher calibration values, source URLs and caveat about the conflicting recorded default optical-frame offset. |
| `launch/mapping_reference.launch.py` | Preserved geometry/projection reference launch; `dataset:=rellis` or the early `dataset:=tartan` compatibility view. Use RELLIS here for the previous green voxel view. |
| `launch/semantic_integration.launch.py` | RELLIS mapper + preview + semantic node + RViz. Arguments include `rviz`, `model_path`, `min_confidence`, `min_observations`, `strict_voxel_occlusion`. |
| `launch/tartan_semantic.launch.py` | Same mapper/model/fusion with Tartan topic remaps, `sensor_init` / `vehicle` frames and separate camera adapter. Arguments: `rviz`, `strict_voxel_occlusion`. |
| `config/rellis_reference.rviz`, `config/tartan_reference.rviz` | Geometry/reference profiles; Tartan reference is from the earlier short compatibility check. |
| `config/semantic_integration.rviz`, `config/tartan_semantic.rviz` | Dataset-specific semantic point cloud, pose and image displays. |
| `config/tartan_playback_qos.yaml` | LiDAR best-effort and static-TF transient-local QoS for Tartan playback. |
| `tools/check_recording.py` | Headless geometry/camera smoke check with saved poses, images and map evidence; optional `--projection`. |
| `tools/check_semantic_integration.py` | Headless RELLIS integration check. Verifies colored geometry matches an OHM map at the same timestamp and class thresholds are respected; saves actual output/report. |
| `tools/check_second_terrain.py` | Separate moving Tartan check; also requires >25 s pose span, >10 m endpoint displacement, and matching saved camera/segmentation timestamps. |
| `tools/extract_remote_tartan.py` | `HTTPRangeFile` fetches bounded exact HTTP byte ranges; `IntervalReader` reads indexes and uncompressed message records for selected streams/intervals; `main()` converts ROS 1 messages to ROS 2 and writes TF from recorded odometry. Pinned to rosbags 0.11.3 internals; not a general compressed-bag extractor. |
| `test/test_ohm_pose_mapping.py` | 4 pose/occupancy mapping tests. |
| `test/test_camera_projection.py` | 5 geometry/projection/PointCloud2 tests. |
| `test/test_semantic_fusion.py` | 11 evidence/occupied-endpoint/ray traversal tests. |
| `test/test_tartan_camera.py` | 6 rectified geometry/quaternion/calibration checks, including parameterized cases. |
| `CMakeLists.txt` | Builds both original and pose-aware C++ executables, installs Python nodes/launch/config, and registers tests. |
| `package.xml` | Earlier stages declared ROS, TF, image, synchronization and test dependencies. It was unchanged in the semantic-integration and second-terrain stages. |
| `requirements-semantic.txt` | Records torch requirement separately from ROS-only dependencies; it is not a complete reproducible environment lockfile. |

### Originals retained

`src/ohm_occupancy_node.cpp`, `scripts/standalone_voxel_node.py`, `semantic_mapping/standalone_voxel_node.py`, and `launch/semantic_mapping.launch.py` are retained. The legacy launch starts the original mapper/Python controller; do **not** run it concurrently with the new semantic launches or assume it has the pose/occupied-only corrections.

The current mapping variants use one CUBE_LIST for occupied geometry and one semantic PointCloud2. We did not replace the original occupancy topic's message type. `/semantic_voxel_map_cpp` is still a **Marker**; `/semantic_voxel_map` is the semantic **PointCloud2**.

## 5. Topics, frames and defaults

| Role | RELLIS | Moving Tartan |
|---|---|---|
| Input LiDAR | `/os1_cloud_node/points` | `/velodyne_1/velodyne_points` |
| Input camera | `/pylon_camera_node/image_raw` | `/multisense/left/image_rect_color` |
| Input calibration | `/pylon_camera_node/camera_info` | Generated `/tartan/camera/camera_info` |
| Adapted image used for segmentation | Original raw image via cv_bridge | `/tartan/camera/image` |
| Fixed world frame / RViz Fixed Frame | `odom` | `sensor_init` |
| Vehicle frame / RViz Target Frame | `base_link` | `vehicle` |
| Camera reference display | `/pylon_camera_node/image_color` | `/multisense/left/image_rect_color` |

Common outputs:

| Topic | Type / meaning |
|---|---|
| `/semantic_voxel_map_cpp` | `visualization_msgs/msg/Marker`; occupied OHM CUBE_LIST |
| `/vehicle_pose` | `geometry_msgs/msg/PoseStamped`; vehicle pose at an integrated scan's timestamp |
| `/semantic_voxel_map` | `sensor_msgs/msg/PointCloud2`; occupied XYZ, packed RGB, `label`, `confidence`, `observations` |
| `/camera_semantic_voxel/segmentation_image` | Class overlay image |
| `/camera_semantic_voxel/labels` | `mono8` group IDs |
| `/camera_semantic_voxel/projection_image` | Measured LiDAR projection on the corresponding camera image |
| `/camera_semantic_voxel/status` | JSON in `std_msgs/msg/String`; transient-local status |
| `/camera_lidar_projection/points_rgb` | Appearance-RGB measured surface points, only in the reference projection launch |

Defaults: **0.35 m** voxels, **10 m** LiDAR range, camera/LiDAR synchronization slop **0.05 s**, semantic minimum **2 observations**, confidence **0.6**. Current semantic runtime uses `strict_voxel_occlusion:=false`, with nearest measured LiDAR depth in 4×4 image bins. Optional solid occupied-voxel ray blocking exists but is not the current default.

## 6. Terminal commands — normal operation on this PC

Run these in **Ubuntu/WSL terminals**. Choose one dataset at a time. Source ROS in every new terminal. Every communicating terminal must use the same `ROS_DOMAIN_ID`. Domains below are isolation choices, not coordinate frames. Stop another mapper/player in the chosen domain before starting.

### Build after future code changes — already built on this PC

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select semantic_mapping --symlink-install
source install/setup.bash
```

The existing OHM dependency must already be built. On a different machine, do not assume this package-only build installs Ubuntu/ROS/OHM or Python dependencies.

### RELLIS — Terminal 1: processing and RViz

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=101
export ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping semantic_integration.launch.py
```

Wait for `Ready: camera classes -> measured visible occupied surface` before starting Terminal 2. Empty RViz before data arrives is expected.

### RELLIS — Terminal 2: recorded data

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=101
export ROS_LOCALHOST_ONLY=1
ros2 bag play ~/RELLIS-3D/example_filtered_ros2 \
  --clock --delay 3 --rate 0.25 --read-ahead-queue-size 10 \
  --qos-profile-overrides-path artifacts/semantic_integration_2026_10_26/playback_qos.yaml \
  --topics /tf /tf_static /os1_cloud_node/points \
  /pylon_camera_node/image_raw /pylon_camera_node/camera_info
```

The 59.996-second bag contains 599 Ouster clouds and 598 pylon images/calibration messages; quarter-speed playback takes about four minutes. The saved full-duration automated check below used half speed. Neither rate guarantees all frames are processed.

### Tartan — Terminal 1: separate processing and RViz

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=104
export ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping tartan_semantic.launch.py
```

Wait for the same Ready message. Fixed Frame must be `sensor_init`; Target Frame `vehicle`.

### Tartan — Terminal 2: second-terrain data

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

The bag is about 32.202 seconds long, so half-speed playback takes about 64 seconds. It contains 320 LiDAR scans, 320 images, 308 recorded odometry messages, 308 generated TF messages and 2 static-TF messages. It is a moving excerpt of a longer recording, not the complete TartanDrive dataset.

### Either dataset — Terminal 3: check operation

Use `101` for RELLIS or replace it with `104` for Tartan:

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=101
export ROS_LOCALHOST_ONLY=1
ros2 node list
ros2 topic list -t
ros2 topic echo /camera_semantic_voxel/status --once --qos-durability transient_local
```

Then run individual checks as needed:

```bash
ros2 topic echo /semantic_voxel_map --field header --once --qos-durability transient_local
ros2 topic echo /vehicle_pose --once
ros2 topic info /semantic_voxel_map --verbose
ros2 topic hz /semantic_voxel_map
```

`/vehicle_pose` needs ongoing playback; it is not a saved history/latched trajectory. Stop `topic hz` with Ctrl+C. Point-cloud publish frequency is not the segmentation inference rate: geometry updates can republish existing semantics. RViz's displayed FPS is the render rate, not the mapping rate.

Status meanings: `paired` counts matched camera/LiDAR callbacks; `fused_frames` counts jobs that updated evidence; `dropped` aggregates several causes; `class_counts` counts current displayed voxel groups. `processing_seconds` is the last semantic job duration, not average latency. Increasing `fused_frames` demonstrates updates, not recognition accuracy.

### Stop, replay, and return to the earlier green view

Press Ctrl+C in the player terminal, then in the launch terminal. Restart the launch, wait for Ready, then restart playback. **Do not use `--loop`**: replay resets recorded time and can mix map/evidence across a backward time jump. The TF buffer resetting on the initial transition to bag time is different from repeated old-data errors.

For the previous pose-aware green RELLIS geometry/projection view, use this instead of the semantic launch in Terminal 1; keep Terminal 2's RELLIS playback command:

```bash
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
export ROS_DOMAIN_ID=101
export ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping mapping_reference.launch.py dataset:=rellis
```

No file restoration is needed to use this fallback. Do not launch both mapper versions together.

### What the playback options do

`--clock` supplies recorded ROS time; all processing launch nodes use simulation time. `--delay` gives subscribers time to connect. `--rate` changes replay speed. The small read-ahead queue limits buffering. QoS overrides make LiDAR/static TF compatible with subscribers. The topic list avoids unrelated recorded traffic, including recorded vehicle command topics. The current mapping launch does not command robot motion.

## 7. Automated checks and how to read them

Stop an interactive preview in the same domain before these commands. Test harnesses start and stop their own mapper/player; do not start another launch alongside them.

### Stored unit/integration test results, and rerunning after code edits

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select semantic_mapping --return-code-on-test-failure
colcon test-result --test-result-base build/semantic_mapping/test_results --verbose
```

The stored result inspected on 29 October was **26 tests, 0 errors, 0 failures, 0 skipped**. The full suite was run in the earlier implementation session; preparing this handoff only read its results. Tests cover synthetic/functional mapping, projection, evidence and calibration behavior. They do not measure class accuracy or safe traversability.

### Full-duration RELLIS example bag check

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 python3 \
  src/semantic_mapping/tools/check_semantic_integration.py \
  --seconds 130 --rate 0.5 --output artifacts/rellis_handoff_check
cat artifacts/rellis_handoff_check/report.json
```

This attempts the duration of the local one-minute example bag. It does not ensure that every scan/image was processed. Use a new output directory if preserving an earlier report. The harness defaults are shorter; pass the explicit duration above for the intended full-duration check.

### Moving Tartan excerpt check

```bash
cd ~/semantic_map_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=103 ROS_LOCALHOST_ONLY=1 python3 \
  src/semantic_mapping/tools/check_second_terrain.py \
  --seconds 80 --rate 0.5 --output artifacts/tartan_handoff_check
cat artifacts/tartan_handoff_check/report.json
```

The Tartan harness checks occupied-geometry equality, threshold compliance, multiple semantic frames, >25 s pose span, >10 m endpoint displacement, and matching saved camera/segmentation timestamps. `passed: true` means these limited conditions passed. It does not establish quantitative map accuracy, water/trunk detection accuracy, complete ray occlusion handling, all-frame processing, or thesis completion.

### Saved measured results

| Measurement | RELLIS reference run | Tartan half-speed run | Another saved RELLIS run |
|---|---:|---:|---:|
| Pose time span | 59.00 s | 31.66 s | 58.40 s |
| First-to-last displacement | 55.29 m | 81.03 m | 54.52 m |
| Matched camera/LiDAR pairs | 372 | 115 | 124 |
| Fused frames | 107 | 29 | 16 |
| Dropped pairs | 265 | 86 | 108 |
| Final occupied voxels | 33,511 | 24,888 | 30,769 |
| Confirmed classified voxels | 315 (0.94%) | 569 (2.29%) | 18 (0.06%) |
| Ground / vegetation / obstacle | 1 / 308 / 6 | 39 / 530 / 0 | 1 / 15 / 2 |
| Trunk / water | 0 / 0 | 0 / 0 | 0 / 0 |
| Last semantic job duration | 0.49 s | 6.04 s | 2.72 s |
| Colored geometry matched OHM | Yes | Yes | Yes |

These are separate saved runs, not promised reproducible counts or average speeds. Endpoint displacement is not distance travelled; the first Tartan run's sampled path was about 101.29 m because the vehicle turned. Global classified percentage also includes voxels outside the camera's view; a visibility-aware coverage metric is still needed.

Paths to those reports, relative to the workspace:

- `artifacts/semantic_integration_2026_10_26/depth_constrained_replay/report.json`
- `artifacts/second_terrain_tartan_meadows/half_speed_replay/report.json`
- `artifacts/my_semantic_check/report.json`

Corresponding folders contain actual `semantic_voxels.npy`, `vehicle_poses.npy`, output images and headless plots. The Tartan half-speed saved camera, segmentation and projection share timestamp `1698955331917020000` ns. The earlier RELLIS harness saves independently latest images, so do not assume their exposures match exactly.

Other evidence: Tartan `native_replay/` had 324 classified voxels and 19 fused frames. `projection_probe/` is a single-frame calibration preview. `verification_2026_10_26/tartan_figure8/` is an older 1.517-second, almost stationary compatibility test and must not be presented as the moving-terrain validation. `moving_replay/` under the RELLIS semantic artifacts is the stricter solid-cube ray experiment, with much sparser results.

The user's video confirms RViz displays geometry, pose, camera, predictions and projection in the new terrain. Sparse colors in the video are a real limitation. Images freezing after the finite bag ends is normal; a video by itself does not prove timestamp alignment or terrain accuracy.

## 8. Backups and exact change boundaries

Backups are under `~/semantic_map_ws/src/semantic_mapping/backups/`:

| Folder | Contents |
|---|---|
| `before_pose_mapping/` | Original C++ node, legacy launch, CMakeLists and package.xml as `.original` files. |
| `before_camera_projection/` | Build/package originals and a six-file preservation hash manifest. Current files in that manifest still match. |
| `before_semantic_integration/` | Verified copies of all 25 previous package files in `previous_files/`, matching ZIP and hash manifest; separate build/package originals. Only CMakeLists differs from that stage's baseline. |
| `before_second_terrain/` | Verified 39-file snapshot/ZIP/manifest of the project immediately before moving Tartan support, plus `.original` build/package files. |

Both the 25-file and 39-file snapshots were rechecked against their manifests while writing this handoff; every saved snapshot file matched. These are package-source backups, not full dataset/workspace backups.

For the Tartan stage, the planned edit to an existing source file was **two CMakeLists lines**, to install the separate adapter and register its tests. The new Tartan launch, adapter, calibration, profile and verification tool are separate files. Original mapping/model/fusion files stayed unchanged.

The final comparison also detects `config/semantic_integration.rviz` saved with different RELLIS view/panel settings during the session. It still uses RELLIS frames/topics. Its current saved view was retained and copied to `before_second_terrain/rellis_saved_layout_2026_10_27.rviz`; the earlier profile remains in the verified snapshot. Do not claim every configuration file stayed byte-identical.

Use the reference launch to compare behavior before restoring files. If an agent needs to restore a source snapshot, do so in a separate checkout/copy first, inspect the difference, and rebuild there. Do not blindly overwrite current user work.

## 9. Important Tartan-specific provenance and caveats

Source recording: `2023-11-02-16-00-43_down_meadows/down_meadows_0.bag`, about 32.50 GB and 119.95 seconds. The extracted requested interval is source offset 45–75 seconds plus one second on each side for synchronization. About 940 MB of network ranges was fetched, rather than the entire source recording. The dataset GUI downloader was avoided.

Official repository: <https://github.com/castacks/tartan_drive_2.0>

Camera calibration sources:

- <https://raw.githubusercontent.com/castacks/tartan_drive_2.0/main/calibration/multisense_intrinsics.txt>
- <https://raw.githubusercontent.com/castacks/tartan_drive_2.0/main/calibration/extrinsics.yaml>

The bag lacks CameraInfo. The adapter uses publisher-supplied S21 calibration and a separate `tartan_left_rectified_optical` frame. For already rectified pixels it uses P's intrinsic block, zero distortion, and R transpose in the camera-frame transform. It preserves image pixels and exposure times.

The publisher optical extrinsic's lateral offset is 0.105 m, while the bag's default camera URDF contains 0.035 m. Do not silently mix them. The current adapter explicitly selects the publisher calibration. Unit checks confirm the mathematical composition, not numerical calibration accuracy for this recording; measured landmark/reprojection verification is still needed.

`sensor_init -> vehicle` TF is generated from original `/lidar_odometry` poses and original pose timestamps. Offline TF messages are dispatched 1 ms before their pose timestamps, rather than at the later source odometry-processing arrival time. No pose values/acquisition timestamps are invented. This is **not an online odometry-latency benchmark**. Retain this disclosure in thesis methods.

`source_metadata/down_meadows_moving.json` records the public source URL, ETag, selection interval, original connection metadata, message counts and generated-TF source. The old `figure_8_ros2` sample is too short to replace this moving excerpt.

### Optional recovery of model weights or Tartan data

The files already exist on this PC; do not redownload for normal use. On a new machine the model source URL and SHA above allow recovery. Do not overwrite an existing checkpoint before comparing its hash:

```bash
sha256sum ~/semantic_map_ws/models/goose/ppliteseg_class_512.pth
```

The extractor needs rosbags 0.11.3 plus requests in a suitable Python environment. On this PC the existing isolated launcher is ready. To reproduce the Tartan extraction into a **new** directory:

```bash
cd ~/semantic_map_ws
TARTAN_MEADOWS_SOURCE='https://airlab-cloud.andrew.cmu.edu:8080/swift/v1/AUTH_ac8533a83cff4d48bc8c608ad222d330/tartandrive2/bags/2023-11-02-16-00-43_down_meadows/down_meadows_0.bag'
.dataset_venv/bin/python src/semantic_mapping/tools/extract_remote_tartan.py \
  --url "$TARTAN_MEADOWS_SOURCE" --offset 45 --seconds 30 \
  --metadata ~/thesis_data/tartan_drive_2_sample/source_metadata/down_meadows_recovery.json \
  --output ~/thesis_data/tartan_drive_2_sample/down_meadows_recovery_ros2
```

The output folder must not exist. This may transfer about 940 MB; do not run it just to inspect results. The tool requires an exact-range-capable server, uncompressed source chunks, and the pinned reader format; it enforces a 1.5 GB transfer cap. The RELLIS local bag must be copied separately or recovered through its original dataset/conversion workflow; this handoff does not invent a replacement download URL.

## 10. Known problems and what to do next

### Priority 1 — diagnose losses before tuning appearance

The semantic node has one inference worker, an eight-entry pending deque, a 32-map snapshot cache, a two-second wall-clock wait, and selects newer eligible pairs. Under load, queued observations can expire, be overwritten, or be discarded as older than the last processed image. A matching exact LiDAR-timestamp occupied map is required; a scan that OHM skips cannot supply it.

The C++ mapper waits only **0.1 seconds** for scan-time TF. Slower bag playback can make the next bracketing pose arrive more than 0.1 wall seconds later. Tartan poses are about 10 Hz. Slowing playback therefore does not automatically solve everything. End-of-excerpt scans may also lack a following pose. Current reports combine drop causes, so their contributions are unmeasured.

Recommended next limited implementation: create backed-up/separate diagnostic variants that count raw messages, synchronization matches, map availability, TF wait failures, pending overflow, stale pairs and inference failures separately. Profile time in inference, projection, evidence, map conversion/publishing and queue waits. Save timestamped counters and latency distributions in the report.

Then make a repeatable paced/offline replay path which retains the needed snapshots and acquisition-time poses until processing is ready. Consider the source's denser recorded pose/TF streams only after checking their frame semantics and provenance. Do not replace exact sensor-time poses with latest TF, invent poses, widen sync arbitrarily, or lower class thresholds merely to get more color.

Exit evidence: explain the fate of every input/matched observation; process the intended excerpt reliably across repeated runs; verify motion/world alignment; report failures rather than only a pass flag.

### Priority 2 — calibration and semantic quality

Check camera projection against visible landmarks and measured reprojection errors. Separate voxels outside camera coverage from visible voxels that lack accepted evidence. Report coverage on eligible visible surfaces as well as on the whole map. Validate the selected 64-to-6 class grouping against the thesis requirements.

Evaluate class predictions using available annotations or a clearly labeled manually annotated subset. Report per-class precision/recall/IoU and confusion/failure cases as supported by the labels. There are no semantic ground-truth annotations in the extracted Tartan bag. Trunk and water have zero confirmed voxel labels in the saved moving runs; this alone does not prove either presence or detector failure. Do not claim validated detection of those classes. Glare and vehicle-hood predictions need inspection.

### Priority 3 — visibility and multi-frame fusion

Measured-depth filtering limits updates to observed endpoints. Optional solid-cube ray traversal exists, but 0.35 m ground cubes can overblock grazing camera rays and reduce coverage. The current default's `occluded: 0` counter is **not proof of zero occlusion mistakes**; that counter is populated by the optional strict traversal. Validate first-surface behavior on known occlusion cases and account for sparse LiDAR depth.

Evaluate evidence stability, occupancy-clearing interactions, moving objects, correlated observations, confidence calibration, and whether repeated image counts give trustworthy labels. Preserve unknown space and ambiguous classes rather than filling them for appearance.

### Priority 4 — longer, repeatable evaluation

After diagnosing timing/visibility, run a longer meaningful recording and additional environments with the same configuration. Record input counts, accepted counts, covered distance/time, memory/CPU, timing distributions, geometric consistency, class coverage, class accuracy and representative failures. An end-to-end run should mean the intended duration is covered with accounted-for losses; do not equate one local excerpt with the entire dataset.

### Priority 5 — height and navigation interpretation

Metric XYZ already exists. Z in `odom` or `sensor_init` is not height above local ground. A local ground estimate, height differences, slope, roughness, obstacle height and vehicle clearance still need implementation/evaluation. POLARIS dimensions and acceptable terrain limits must be established before translating them into traversability costs. Semantic class labels and geometric constraints can then feed a cost map/planner. No planner or robot-control integration exists in the current work.

### Priority 6 — thesis and final demonstration

Document source data selection, recorded-pose use, calibration assumptions, pretrained-model origin, semantic fusion equations, visibility limits, CPU timing, reproducibility and failure cases. Preserve reference comparisons and avoid describing original-model training or completed autonomous navigation. Reserve time before 23 November for the thesis, reproducible scripts, figures and supervisor feedback, not only code changes.

## 11. Status against the original 14 tasks

| Task | Actual state |
|---|---|
| 1. Ubuntu and ROS foundations | Environment working; normal commands provided. |
| 2. Workspace/pub-sub | Workspace/package working; original learning nodes retained. |
| 3. LiDAR read/filter/transform/publish | Implemented with range/finite filtering and scan-time world transforms. |
| 4. Pose source | Recorded TF/odometry used; no new localization algorithm built. |
| 5. Multi-scan voxel map | Implemented in fixed voxels for accepted scans; full-input and accuracy evaluation unfinished. |
| 6. Camera/LiDAR/pose synchronization | Basic image/info exact and camera/cloud approximate synchronization plus acquisition-time TF; losses and reliability unresolved. |
| 7. Pretrained segmentation inference | Implemented with GOOSE PP-LiteSeg-B; CPU performance and dataset suitability need evaluation. |
| 8. Pixel class probabilities | Implemented, grouped into six classes; recognition accuracy unverified. |
| 9. Calibrated 2D/3D association | Implemented; numerical alignment/calibration validation still required, especially Tartan. |
| 10. Semantic ray tracing | Partial: measured-surface updates and optional occupied-voxel traversal; complete intended camera-ray formulation/evaluation unfinished. |
| 11. LiDAR depth constraint | Implemented measured-depth endpoint constraint; sparse depth failure cases need testing. |
| 12. Visibility/occlusion | Partial; complete occlusion correctness not established. |
| 13. Multi-frame class fusion | Implemented pseudo-count evidence; stability/accuracy/coverage evaluation unfinished. |
| 14. Semantic visualization | Implemented camera/pose/occupied-voxel displays; sparse coverage remains. |

Height/traversability and safe POLARIS navigation are additional desired outcomes, not completed consequences of the colored display. Do not repeat the earliest handoff's blanket claim that Tasks 13/14 are fully finished.

## 12. Troubleshooting without changing code

| Symptom | Check |
|---|---|
| `ros2` or package not found | Source `/opt/ros/humble/setup.bash`, then the workspace setup in that same terminal. |
| Missing newly added Python helper after edits | Rebuild with `--symlink-install`, then source setup again; ament's generated Python file links can need regeneration. |
| Empty RViz | Start the correct bag, same ROS domain, correct Fixed Frame, correct output topic/type, compatible QoS. Inspect status/header before editing code. |
| Pose arrow missing | Set Pose topic `/vehicle_pose`; check playback is still running and Fixed Frame matches. Pose is published on accepted scan callbacks. |
| Only gray voxels | Read class/fusion/drop counters; gray is occupied unknown. Check actual camera projection and evidence thresholds; do not recolor all voxels artificially. |
| Images/map stop changing | Check whether the finite bag finished. That is expected at its end. |
| Repeated `TF_OLD_DATA` / backwards time | Stop and restart mapper/player together. Avoid bag looping and unrelated TF publishers in the same domain. |
| Future extrapolation | Check bracketing pose availability and wall-time TF wait. Never substitute latest pose to silence it. |
| Model load error | Check file path/SHA and explicit class mapping. Strict weight loading should fail rather than use partially random weights. |
| `passed: true` but poor colors | The harness checks functional conditions, not class accuracy or adequate coverage. Compare report numbers and raw images. |
| ROS/socket/GUI tool blocked for another agent | Use the environment's legitimate permission request for local DDS/GUI or public downloads. Do not bypass sandbox access checks. |

## 13. Other documents and evidence

Under `~/semantic_map_ws/src/semantic_mapping/docs/`:

- `RUN_IT_YOURSELF.md`: original semantic RELLIS quick start. Its historical 20-test count predates the Tartan checks; current stored total is 26.
- `COMMANDS_AND_CODE_WALKTHROUGH.md`: detailed function-by-function RELLIS explanation from 27 October. Its early Tartan description predates the moving meadow test.
- `SECOND_TERRAIN_TARTAN.md`: full Tartan commands, calibration provenance, results and backups.
- `semantic_integration.md`: semantic implementation/evaluation notes.
- `verification_and_fusion.md`: earlier geometry/projection compatibility checks.

This dated handoff supersedes those documents only where it explicitly records newer state. Do not delete the historical evidence.

Useful bundles: `artifacts/second_terrain_tartan_meadows/supervisor_results.zip`, `artifacts/semantic_integration_2026_10_26/integrated_results.zip`, and camera references under `artifacts/camera_reference/`.

### Plain-language supervisor explanation

> I tested the pose-aware voxel mapping pipeline on RELLIS and a moving TartanDrive excerpt. The map accumulates accepted LiDAR scans in a fixed frame and displays the vehicle pose. For initial image labeling, I used public pretrained PP-LiteSeg-B weights from the GOOSE project. Camera calibration and recorded poses relate pixel predictions to measured LiDAR surfaces, and class evidence is accumulated in occupied voxels. The semantic coverage is still sparse, and I am investigating dropped observations and checking alignment and accuracy. Height-relative-to-ground and navigation evaluation remain further work.

### Files to give another agent

1. This Markdown file, `THESIS_HANDOFF_2026-10-29.md`.
2. `THESIS_HANDOFF_2026-10-29.zip` for source, backups, provenance, reports and selected images.
3. Access to the existing workspace/data/model paths, or separate copies of those large dependencies if moving computers.

The ZIP includes an integrity manifest and an explicit exclusion list. It is a source/evidence handoff, not an installed Ubuntu/ROS environment or a full dataset backup. Extract it to a separate review folder first; do not unpack over current work without comparing files.
