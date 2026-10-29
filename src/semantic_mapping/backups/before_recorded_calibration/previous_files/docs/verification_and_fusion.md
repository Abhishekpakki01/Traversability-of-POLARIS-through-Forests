# Dataset verification and camera projection — 26 October 2026

## Results and scope

The pose-aware OHM mapper ran on a second dataset, TartanDrive 2.0. A separate
RELLIS node now pairs camera images with LiDAR and projects measured surfaces
into the image. Both checks ran without a GUI. Their saved plots show actual
published ROS data; they are not RViz screenshots.

| Check | Observed result | Limit |
| --- | --- | --- |
| TartanDrive `figure_8_3.bag` | 1,506 occupied voxels, 5 map messages, 5 vehicle poses, 13 camera images | The entire source segment is only 1.517 seconds; accepted vehicle poses moved about 1 cm. This is a compatibility check, not a moving-drive accuracy evaluation. |
| RELLIS partial replay | 2,258 occupied voxels, 8 projection images, 8 RGB clouds; last cloud has 3,513 samples in `odom` | Last paired image and cloud differ by 24.33 ms. This checks data flow and geometry code, not quantitative calibration accuracy or real-time performance. |
| Automated checks | 4 ROS mapping tests and 5 projection tests passed | Synthetic cases complement the bag checks; they do not establish terrain recognition accuracy. |

Saved evidence is in `~/semantic_map_ws/artifacts/verification_2026_10_26/`:

- `tartan_figure8/camera_reference.png`, `occupied_voxels.png`, `report.json`
- `rellis_projection/camera_reference.png`, `camera_lidar_projection.png`,
  `occupied_voxels.png`, `report.json`
- Each directory also contains playback QoS settings and process logs.

The Tartan segment starts before its recorded dynamic TF becomes available.
The mapper rejected the early scans and accepted five later scans. It did not
invent poses or fall back to the latest TF. A longer segment with appreciable
vehicle motion is still needed to evaluate accumulated-map consistency.

## What changed

The existing OHM sources, Python controllers, and original launch file retain
their previous contents. `backups/before_camera_projection/preserved_files.json`
records their SHA-256 hashes. Copies of the build files before these additions
are in that same directory with `.original` suffixes.

New files provide:

- `scripts/camera_lidar_projection_node.py`: image/CameraInfo/LiDAR pairing,
  timestamp-specific TF, reference overlay, and an RGB surface cloud.
- `semantic_mapping/camera_projection.py`: geometry and measured-depth filtering.
- `launch/mapping_reference.launch.py` and `config/*_reference.rviz`: separate
  RELLIS and Tartan reference views.
- `tools/check_recording.py`: reproducible headless verification and saved evidence.
- `test/test_camera_projection.py`: projection and mixed-field PointCloud2 checks.

`CMakeLists.txt` installs the new scripts/configuration and registers the new
tests. `package.xml` declares the camera, Python, and launch dependencies already
available on this machine. The existing camera preview script is now installed
as an executable; its contents are unchanged. No dataset GUI was installed.

The voxel topic remains a single occupied-only `Marker` CUBE_LIST. The RGB cloud
is a separate reference topic. Its colors are camera appearance, not predicted
vegetation, trunk, ground, or water classes. This change does not update voxel
semantics or change occupancy integration.

## View the second dataset

Run only one mapper at a time. Stop existing mapping and playback terminals
before using these commands. All terminals must use the same ROS_DOMAIN_ID.

Terminal 1 starts the mapper and RViz with the Tartan frames and camera topic:

```bash
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
ros2 launch semantic_mapping mapping_reference.launch.py dataset:=tartan
```

Terminal 2 starts the downloaded, converted sample:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play ~/thesis_data/tartan_drive_2_sample/figure_8_ros2 \
  --clock --delay 3 --rate 0.2 \
  --qos-profile-overrides-path ~/semantic_map_ws/artifacts/verification_2026_10_26/tartan_figure8/playback_qos.yaml \
  --topics /tf /tf_static /velodyne_1/velodyne_points /multisense/left/image_rect_color
```

Expected fixed frame: `sensor_init`. Vehicle frame: `vehicle`.
The slow playback gives about 7.6 seconds of recorded data display time.
The accumulated map remains after playback ends. Start RViz before playing
because map and pose publishers use volatile durability.

The YAML configurations and launch arguments were checked programmatically.
The new RViz layouts have not been checked in a GUI on this machine.

## View the RELLIS camera projection

Stop both Tartan terminals before starting RELLIS.

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
ros2 launch semantic_mapping mapping_reference.launch.py dataset:=rellis
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play ~/RELLIS-3D/example_filtered_ros2 \
  --clock --delay 3 --rate 0.5 \
  --qos-profile-overrides-path ~/semantic_map_ws/artifacts/verification_2026_10_26/rellis_projection/playback_qos.yaml \
  --topics /tf /tf_static /os1_cloud_node/points \
    /pylon_camera_node/image_raw /pylon_camera_node/camera_info
```

The reference view contains the occupied map, vehicle arrow, color camera
image, and image with depth-colored LiDAR samples. For an isolated RGB point
view, disable the occupied map display and enable
`RGB surface reference (appearance only)`.

To inspect synchronization in a third sourced terminal:

```bash
ros2 topic echo /camera_lidar_projection/status --once
```

The projection requires matching image/CameraInfo timestamps, then pairs that
camera frame with a LiDAR scan within 50 ms. It obtains world-from-camera at
the image timestamp and world-from-LiDAR at the scan timestamp. Their composition
accounts for vehicle movement between acquisitions. This is not per-point scan
deskewing and assumes a static scene between the two measurements.

Projection uses the bag's raw camera K/D calibration and TF, positive camera
depth, a 10 m LiDAR range, image bounds, and the nearest measured depths within
2-pixel cells with 0.10 m tolerance. Sparse returns cannot prove complete
occlusion handling. Inspect hard object boundaries in the overlay before using
this geometry to assign semantic labels. No numerical reprojection error has
been measured yet.

Restart the mapper and projection nodes before restarting or seeking the bag
backwards. Do not use `--loop` for these checks.

## Return to the existing mapper

Stop the new launch and playback with Ctrl+C. The previously working pose-aware
mapper remains available directly:

```bash
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
ros2 run semantic_mapping ohm_occupancy_pose_node --ros-args -p use_sim_time:=true
```

The original executable `ohm_occupancy_node` is also still available. The original
`semantic_mapping.launch.py` retains its previous behavior and starts the old
mapper/controller combination; use the explicit command above for the
pose-aware occupied-only mapper.

## Reproduce headless checks

These commands create their own mapper/playback processes, so run them separately
from the interactive launch. The isolated domain numbers avoid existing sessions.

```bash
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 python3 \
  ~/semantic_map_ws/src/semantic_mapping/tools/check_recording.py \
  --bag ~/thesis_data/tartan_drive_2_sample/figure_8_ros2 \
  --cloud-topic /velodyne_1/velodyne_points \
  --image-topic /multisense/left/image_rect_color \
  --fixed-frame sensor_init --base-frame vehicle --rate 0.2 --seconds 18 \
  --output ~/semantic_map_ws/artifacts/verification_2026_10_26/tartan_figure8
```

```bash
ROS_DOMAIN_ID=98 ROS_LOCALHOST_ONLY=1 python3 \
  ~/semantic_map_ws/src/semantic_mapping/tools/check_recording.py \
  --bag ~/RELLIS-3D/example_filtered_ros2 --rate 0.5 --seconds 25 --projection \
  --output ~/semantic_map_ws/artifacts/verification_2026_10_26/rellis_projection
```

Results can differ slightly with scheduling and DDS discovery. Bag sizes,
source URLs, and the conversion command are recorded in
`~/thesis_data/tartan_drive_2_sample/verification_provenance.json`.

## The two alternative datasets

The official sources were inspected on 26 October 2026. Neither ORAD nor GOOSE
has been downloaded or connected to this mapper.

| Dataset | Relevant strengths | Integration work |
| --- | --- | --- |
| [ORAD-3D](https://github.com/chaytonmin/ORAD-3D-Dataset-For-Off-Road-AD) | RGB, LiDAR, calibration, KISS-ICP poses, semantic occupancy; water/puddles, grass-on-road, rocks and road classes | File reader and ROS adapter; verify pose convention, timestamps, and its right/forward/up axes. The listed full download is about 350 GB. Its published class list has no separate tree-trunk class. |
| [GOOSE](https://goose-dataset.de/docs/class-definitions/) | Fine classes include tree_trunk, tree_crown, grasses and water; [labeled image and point-cloud formats](https://goose-dataset.de/docs/dataset-structure/) | Labeled frame exports alone do not establish a continuous pose stream. Inspect a specific raw bag's TF/odometry and calibration before accumulation. |

My recommendation is to evaluate GOOSE for the semantic stage. Its official
[setup page](https://goose-dataset.de/docs/setup/) provides PP-LiteSeg and DDRNet
weights, including 64-class versions. The 12-category versions merge vegetation
and would not preserve a separate trunk class. This recommendation still needs
an inference check on the actual RELLIS images and a review of rare-class quality.

## Next work toward actual semantic voxel colors

1. Inspect projection alignment on several hard boundaries and across vehicle
   movement. Obtain a longer moving second-dataset segment for map consistency.
2. Evaluate a pretrained off-road segmentation model on saved camera frames,
   retaining probabilities and its exact class-ID mapping. Unsupported or
   uncertain classes remain unknown.
3. Map model classes into vegetation, trunks, ground, water, and unknown. These
   IDs must come from the model's ontology, not from the current green cubes.
4. Transfer observations only to visible, LiDAR-supported occupied surfaces.
   Traverse rays against geometry to stop semantic updates at the first surface;
   do not fill free cells or unseen voxels behind it.
5. Fuse per-voxel class evidence across timestamps, then publish class colors
   separately from the preserved occupancy-only view.
6. Estimate local ground height, obstacle height above that ground, slope, and
   clearance from geometry. World-frame Z alone is not obstacle height. Combine
   those measurements with semantics before making traversability claims.

Image pairing and projection are now available for steps 6 and 9 of the thesis
task list. Model inference, full semantic ray traversal/occlusion handling,
multi-frame semantic probabilities, semantic voxel coloring, and traversability
evaluation are still outstanding.
