# Offline map demonstration — 2 November 2026

This version processes the local TartanDrive down_meadows excerpt before
playing the results. It avoids losing image pairs while CPU inference is busy.
The existing live launches are still available.

The MP4s render actual saved OHM voxel centres with VTK. They are not screen
recordings of RViz. The saved ROS bag lets you inspect the same map interactively
in RViz, with camera images, predictions, projection, vehicle pose and trail.

## Open the finished results

The two videos are in:

```text
~/Traversability-of-POLARIS-through-Forests/artifacts/offline_demo_tartan_2026_11_02/semantic_demo.mp4
~/Traversability-of-POLARIS-through-Forests/artifacts/offline_demo_tartan_2026_11_02/elevation_demo.mp4
```

Each is 36.85 seconds: recorded-time playback of about 32 seconds followed by a
five-second orbit around the final map. This is the whole **local excerpt**,
not the publisher's full recording. The files use MPEG-4 video in an MP4 container.

Use one Ubuntu terminal to replay in RViz:

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=112
export ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping offline_demo.launch.py
```

RViz opens and the saved result starts after four seconds. No segmentation model
runs during this playback. The final map stays visible after playback ends.
Stop with Ctrl+C before launching another replay. Do not add `--loop`.

Fixed Frame is `sensor_init`; the view follows `vehicle`. The default display is
a **15 m horizontal radius around the vehicle**, including unknown voxels.
This is a display crop of the full saved map. To see the full route, disable
`Local semantic map (15 m radius)` and enable `Full accumulated map (optional)`.
The original full-map layout also remains in `config/offline_demo.rviz`.
Camera panels have been enlarged and Views is hidden initially (available under
Panels → Views). The displays are:

- Local semantic voxels: gray unknown, brown ground, green vegetation,
  darker brown trunk, blue water and red obstacle.
- Vehicle pose and trail: recorded motion in the fixed map frame.
- Camera reference, predicted terrain classes and depth-colored projection.
- Full-map counts and legend: processing counts and limitations. These counts
  describe the entire map, not just the local display. Depending on
  saved window layout, drag or resize this image panel for readability.
- Optional elevation: disable the semantic display and enable this display to
  inspect elevation colors without overlapping the two clouds.

Elevation is **voxel world Z minus the first mapped vehicle's world Z**, in
metres. Its fixed color range is -20 to +5 m, clipped at the ends. It is not
height above local ground, ground clearance, slope or a traversability decision.
The saved point cloud also contains `relative_z`, `label`, `confidence` and
`observations` fields. The recorded frame has not been independently validated
as gravity aligned.

## Rebuild after editing code

Use the setup commands above, then:

```bash
colcon build --packages-select semantic_mapping --symlink-install
source install/setup.bash
```

This PC already has ROS Humble, OHM, the original model checkpoint, NumPy,
OpenCV, PyTorch and VTK. A new PC also needs those dependencies and the source
recording. VTK and a working graphics connection are needed only for rendering
the MP4s; mapping and the saved-data audit do not need a graphical window.

## Reproduce the processing

The output folder must be new. Existing results are never silently overwritten.
After the setup commands, run:

```bash
python3 src/semantic_mapping/tools/build_offline_demo.py \
  --output artifacts/offline_demo_my_run
```

The verified run took about five minutes and produced about 2.3 GB of derived
data. Actual time depends on the PC. It processes every input scan in order,
using interpolated recorded poses at the camera and LiDAR timestamps.
Inputs without pose coverage are counted and excluded; poses are not extrapolated.

The default input is:
`~/thesis_data/tartan_drive_2_sample/down_meadows_moving_ros2`.
This runner currently supports this Tartan topic/frame convention and one
SQLite `.db3` file. Passing another dataset path does not adapt its calibration.

For a deliberately partial quick check:

```bash
python3 src/semantic_mapping/tools/build_offline_demo.py \
  --max-scans 20 --output artifacts/offline_demo_my_quick_check
```

That output is marked partial in the report and rendered video.

Validate and render a completed run:

```bash
python3 src/semantic_mapping/tools/check_offline_demo.py \
  --input artifacts/offline_demo_my_run
python3 src/semantic_mapping/tools/render_offline_demo.py \
  --input artifacts/offline_demo_my_run
python3 src/semantic_mapping/tools/render_offline_demo.py \
  --input artifacts/offline_demo_my_run --color-mode elevation
```

Rendering needs an accessible display/graphics connection in WSL. A sandbox can
block that connection even when mapping works. If a render is interrupted,
preserve or rename its incomplete MP4 before retrying; the renderer refuses to
overwrite it.

Replay that newly processed run:

```bash
ros2 launch semantic_mapping offline_demo.launch.py \
  bag:="$PWD/artifacts/offline_demo_my_run/replay"
```

The replay bag contains derived visualization outputs. It is not a replacement
for the original sensor bag for recalibration or sensor reprocessing.

## What the new code does

| File | Purpose and important functions |
|---|---|
| `src/ohm_offline_mapper.cpp` | Keeps an OHM map alive between requests. Receives the transformed ray origin and endpoints, calls the same CPU `integrateRays` used by the existing mapper, and returns only `isOccupied` voxel centres. |
| `semantic_mapping/offline_demo.py` | `NearestImage` chooses an unused exposure within 50 ms. `OfflineOhm.integrate` waits for OHM to finish each scan. `cloud_message` preserves semantic attributes and adds relative Z. |
| `tools/build_offline_demo.py` | Reads the bag without ROS delivery queues, interpolates recorded poses, integrates geometry, runs the existing GOOSE predictor, then updates the existing `SemanticEvidence`. Saves the replay, arrays, images and processing report. |
| `tools/check_offline_demo.py` | Checks every saved map against its replay message, class thresholds, packed colors, elevation values, unique exposures and image timestamps. |
| `tools/render_offline_demo.py` | Draws actual 0.35 m occupied cubes, vehicle pose and path with VTK. Combines these with corresponding camera images and writes recorded-time MP4s with counts and a legend. |
| `scripts/offline_demo_view.py` | Pairs each saved cloud with its exact-timestamp vehicle pose, then publishes a local display crop. It retains unknown voxels and every original point attribute. |
| `config/offline_demo_local.rviz` | Default local RViz view with larger camera panels and a toggle for the full map. `offline_demo.rviz` retains the full-map layout. |
| `launch/offline_demo.launch.py` | Opens that layout and plays the saved result with a 100-message read-ahead buffer. |
| `test/test_offline_demo.py` | Checks matching, exposure reuse, serialization and a translated-origin OHM case where free space must remain absent. |

The model, 10 m mapping range, 0.35 m resolution, 0.6 confidence threshold and
minimum two observations were retained. Camera-to-LiDAR association uses each
sensor's acquisition pose. Depth filtering remains the current measured-LiDAR
method; optional solid-voxel occlusion is available with
`--strict-voxel-occlusion`. Its grazing-ground limitations still apply.

## Results and limits

- 320 input scans examined; 318 mapped and fused. The final two scans fall beyond
  available odometry coverage and were excluded.
- 318 unique camera exposures inferred. No pairs were discarded because an
  inference worker was busy.
- 26,325 final occupied voxels; 8,947 have camera evidence and 5,774 have confirmed
  class labels. Coverage is 21.9% of all occupied voxels and 64.5% of those with
  camera evidence. These percentages are not accuracy measurements.
- Final class counts: 538 ground, 5,203 vegetation, 1 trunk, 3 water, 29 obstacle,
  and 20,551 unknown. The small water/trunk counts are unvalidated predictions.
- About 31.76 s of mapped motion and 81.31 m endpoint displacement.
- All 318 map/image outputs passed the saved-data audit. The regression suite
  has 38 tests, including five new offline checks.

The 20,551 gray voxels consist of 17,378 without camera evidence, 2,309 with only
one observation, 844 with multiple observations but low confidence, and 20 whose
most likely class remains unknown. The recorded vehicle Z changes by -11.63 m;
this explains the slope of the accumulated route in the display but does not
independently validate odometry. A long map trail is not a new obstacle.

The previous live test classified 460 voxels from 24 fused frames. This change
improves processing completeness; it does not demonstrate improved semantic
accuracy. Maps also differ because the offline run integrates more scans.

The physical calibration and semantic accuracy remain unverified. Trunks,
water and obstacle predictions require checking against reference labels.
Some unknown cells are outside camera coverage; others have insufficient or
uncertain evidence. No safe-driving decisions are produced. Local-ground
height, slope, roughness, clearance and vehicle-specific traversability remain
future work. These are TartanDrive observations, not a physical POLARIS trial.

The GOOSE checkpoint supplies predictions; the recording comes from TartanDrive.
Original model/dataset credits and notices remain in the repository.

## Saved evidence and backup

The result folder contains `report.json`, `validation.json`, `frames.jsonl`,
per-frame `voxels.npz` and camera panels, the two videos, and `replay/`.
Reports record source-bag, model and calibration hashes. The initial elevation
render used too narrow a scale; `before_elevation_scale/` preserves that video
and database before only the elevation RGB fields were corrected. The semantic
geometry, labels, confidence, timestamps and camera images did not change.

Large generated results are ignored by Git. A small verified result summary is
kept under `artifacts/offline_validation_2026_11_02/`.

Only two pre-existing files were edited for this feature: `CMakeLists.txt` adds
the separate executable/test, and `.gitignore` excludes large generated runs.
The 66-file verified snapshot is at
`src/semantic_mapping/backups/before_offline_demo_2026_11_02/`.
The old mapping, model, fusion, calibration and launch files were preserved.
The original `~/semantic_map_ws` source was also preserved.

To return to the previous behavior, stop this replay and run the old
`tartan_recorded_calibration.launch.py` with the source bag as described in
`TARTAN_CALIBRATION_CHECK.md`. No source restoration is required.
