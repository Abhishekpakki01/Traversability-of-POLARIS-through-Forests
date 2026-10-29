# Traversability of POLARIS through Forests

ROS 2 / OHM research code for accumulating pose-aligned LiDAR occupancy and
associating camera terrain predictions with occupied voxels. The application
targets off-road terrain understanding for the POLARIS vehicle.

**Current state: initial semantic-mapping integration.** It is not a completed
or validated navigation system. Semantic coverage is sparse, frames are dropped,
and water/trunk accuracy, ground-relative height, slope, clearance and planning
still need work. The implementation uses recorded poses, not a newly developed
localization method.

## Start here

- [Complete handoff, code explanations, results, remaining tasks and terminal commands](THESIS_HANDOFF_2026-10-29.md)
- [Terminal commands reference](THESIS_TERMINAL_COMMANDS_2026-10-29.txt)
- [RELLIS quick start](src/semantic_mapping/docs/RUN_IT_YOURSELF.md)
- [TartanDrive moving-terrain test](src/semantic_mapping/docs/SECOND_TERRAIN_TARTAN.md)
- [How to edit and upload future changes](docs/GITHUB_WORKFLOW.md)
- [Sensor-data locations and provenance](data_provenance/README.md)
- [Pretrained model origin and recovery](models/goose/README.md)

The original ten-file GitHub upload is preserved in
[`legacy/original_upload/`](legacy/original_upload/), and the original Git history
is retained. `legacy/COLCON_IGNORE` prevents colcon from discovering that older
flattened package alongside the current one.

## What supplies the data and predictions

**RELLIS-3D and TartanDrive 2.0** supply the demonstrated recordings. **GOOSE**
supplies official pretrained PP-LiteSeg-B image-segmentation weights. We did not
train a new image model or use a GOOSE recording for the demonstrated mapping
tests. The model's 64 pixel class distributions are grouped into unknown,
ground, vegetation, trunk, water and obstacle.

Recorded poses transform LiDAR ray origins and endpoints into a fixed frame.
OHM accumulates occupancy. Camera calibration, acquisition-time poses and
measured LiDAR depth relate camera predictions to occupied surface voxels.
Class evidence is accumulated across images and displayed in RViz.

```text
Recorded LiDAR + poses -> fixed-frame OHM occupied voxels ----+
Camera + calibration -> PP-LiteSeg-B pixel probabilities -----+-> semantic voxel evidence -> RViz
LiDAR + camera-time/scan-time poses -> visible surface links -+
```

Gray means occupied geometry with unknown semantics, rather than free space.
Colors are predictions. The three image panels show the same camera stream:
original pixels, class overlay, and LiDAR projection overlay.

## Repository layout

```text
src/semantic_mapping/   Current package, tests, launch/config, docs and original backups
src/ohm/                OHM source snapshot with its original license
models/goose/           Model provenance and reference code; checkpoint downloaded separately
artifacts/              Saved measured reports, logs, point/pose arrays and output images
data_provenance/        Source/calibration records and bag metadata; no raw sensor payloads
docs/                   GitHub workflow and project figures
legacy/                 Previous GitHub files, preserved and ignored by colcon
THESIS_HANDOFF_*.md      Full context for another researcher or coding agent
```

Build/install/log trees, Python environments, credentials, raw bags, videos and
learned weight binaries are intentionally outside normal Git tracking. They
remain on the original PC. See the provenance documents for locations and
recovery. Selected saved results and the small verified source-backup ZIPs are
included. This repository is source and evidence, not an installed Ubuntu image
or a full dataset backup.

## Run on the existing PC

The working application remains in `~/semantic_map_ws`. Creating this GitHub
checkout did not modify its mapping code or datasets. The new editable checkout
is `~/Traversability-of-POLARIS-through-Forests`.

To run the existing baseline, use the terminal blocks in the handoff. To build
and run edits made in this checkout on the same PC, overlay it on the existing
workspace, which supplies the already-built OHM dependency:

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
colcon build --packages-select semantic_mapping --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=104
export ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping tartan_semantic.launch.py
```

Wait for the semantic node's `Ready` message. In another Ubuntu terminal:

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=104
export ROS_LOCALHOST_ONLY=1
ros2 bag play ~/thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2 \
  --clock --delay 3 --rate 0.5 --read-ahead-queue-size 10 \
  --qos-profile-overrides-path src/semantic_mapping/config/tartan_playback_qos.yaml \
  --topics /tf /tf_static /velodyne_1/velodyne_points \
  /multisense/left/image_rect_color /lidar_odometry
```

Stop the player and launch with Ctrl+C before a fresh replay. Avoid `--loop` and
duplicate mapper instances. Existing default model/data paths refer to the
original home workspace; see the handoff before moving machines. A new machine
also needs Ubuntu/ROS, OHM dependencies, the model, and sensor data; these commands
are not a clean-machine installer.

## Saved measurements and limitations

| Measurement | RELLIS reference | Tartan moving excerpt |
|---|---:|---:|
| Received pose span | 59.00 s | 31.66 s |
| First-to-last pose displacement | 55.29 m | 81.03 m |
| Fused semantic frames / matched pairs | 107 / 372 | 29 / 115 |
| Final classified / occupied voxels | 315 / 33,511 | 569 / 24,888 |
| Classified percentage of whole map | 0.94% | 2.29% |
| Geometry matched occupied OHM output | Yes | Yes |

The reports are in [RELLIS results](artifacts/semantic_integration_2026_10_26/depth_constrained_replay/report.json)
and [Tartan results](artifacts/second_terrain_tartan_meadows/half_speed_replay/report.json).
Another saved RELLIS run produced only 18 classified voxels, so the above counts
are not guaranteed repeatable results. Global percentages include areas outside
camera coverage; visibility-aware coverage is not yet measured. The extracted
Tartan recording is about 32 seconds, not the entire dataset.

The stored suite contains **26 passing tests** covering mapping, projection,
semantic evidence and rectified geometry. These are functional/synthetic checks,
not semantic ground-truth accuracy or safe-navigation certification. The next
priority is to measure drop reasons and timing, make replay reliable, validate
projection and class quality, then evaluate longer sequences and height-based
traversability.

## Attribution

- [OHM](https://github.com/csiro-robotics/ohm), included with its Modified MIT license.
- [RELLIS-3D](https://github.com/unmannedlab/RELLIS-3D), recording data used locally.
- [TartanDrive 2.0](https://github.com/castacks/tartan_drive_2.0), recording/calibration source.
- [GOOSE](https://github.com/FraunhoferIOSB/goose_dataset), pretrained segmentation weights and class definitions.
- [SuperGradients](https://github.com/Deci-AI/super-gradients), architecture reference.

Preserve [third-party notices](src/semantic_mapping/THIRD_PARTY_NOTICES.md),
[the adapter's upstream Apache license](src/semantic_mapping/LICENSE.super-gradients),
and the licenses distributed with OHM/reference sources. Dataset and pretrained
model terms are separate from the student's project source.

