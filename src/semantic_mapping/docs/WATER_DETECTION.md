# Water on the map — 2 November 2026

## Why water was almost invisible

LiDAR light (905 nm) is largely absorbed by water or mirrored away from the
sensor, so puddles return few or no points. The semantic fusion only labels
**measured LiDAR endpoints**, so the camera's water pixels had almost nothing to
attach to. The full Tartan map had just **3 water voxels**, although the camera
predicts water in about 2% of all image pixels over the drive.

## Method

`semantic_mapping/water_detection.py` and `tools/build_water_replay.py`:

1. For every saved frame, take the saved class image: the same GOOSE groups and
   0.6 confidence threshold as the map, sampled every 8 pixels (8,704 rays).
2. Cast each pixel ray from the camera pose **at the image timestamp**, using
   recorded odometry and the recorded camera mount. Intersect it with that
   frame's mapped ground (`terrain_analysis`). Holes in the LiDAR ground up to
   8 cells (2.8 m) are filled from the surrounding ground height.
3. Discard a ray that enters an above-ground occupied voxel first, since it sees
   an object, not the ground. Stop at the 10 m mapping range.
4. Accumulate camera pixel votes per ground column over frames. A column is
   **water** when it was seen in at least 3 frames with at least 5 labelled
   pixels, and at least 50% of those are water.
5. Mark whether LiDAR returned anything in that column:
   - **bright blue**: water with no LiDAR return. Camera and the LiDAR gap agree,
     which is the strongest cue.
   - **light blue**: water with a LiDAR return, e.g. shallow water over mud or a
     camera mistake.

Tests: `test/test_water_detection.py`, four synthetic cases: hole filling,
ray–ground intersection with occlusion, optical-axis directions, and evidence
thresholds.

## Result on the TartanDrive excerpt

| | Value |
|---|---:|
| Pixel rays cast / hit visible ground | 2,767,872 / 1,245,660 |
| Water columns, no LiDAR return | 63 |
| Water columns, LiDAR return | 127 |
| Water area (0.35 m cells) | 23.3 m² |
| Previous fused water voxels | 3 |
| Water within 2 m of the driven path | 99% (median 0.58 m) |

The water lies in the vehicle's track, where the camera images show standing
water in the wheel ruts. Figure: `artifacts/water_demo_2026_11_02/water_top_down.png`.
It shows camera classes painted on the ground, water cells and the path.

**Limits**

- Water is a camera prediction. The labelled RELLIS frame showed GOOSE recall of
  only 13% for a puddle (`TERRAIN_AND_VALIDATION.md`), so water is
  **under-detected**, not over-detected.
- Sun glare on a wet track can look like water. The two cannot be separated
  here.
- Heights in filled LiDAR holes are interpolated, not measured.
- There are no Tartan water labels, so precision and recall are unmeasured.

## RViz

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=112
export ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping water_demo.launch.py rate:=0.5
```

`config/water_demo.rviz` is derived from the LiDAR feature layout, which is
unchanged:

- **Water on ground (camera + LiDAR gaps)**, on: bright and light blue tiles,
  drawn 0.2 m above the ground cell so they stay visible over brown ground
  voxels.
- **Camera classes projected on ground**, optional: the camera's majority class
  per ground column, including LiDAR holes.
- **LiDAR-only classes**, permeability, linearity and the camera comparison are
  as before.
- **LiDAR classes + water: key**: the colour key, with water area in m².

## Rebuild

```bash
/usr/bin/python3 src/semantic_mapping/tools/build_water_replay.py \
  --run artifacts/offline_demo_ego_filtered_2026_11_02 \
  --replay artifacts/lidar_features_demo_2026_11_02/replay \
  --output artifacts/water_my_build                              # ~2 min
/usr/bin/python3 src/semantic_mapping/tools/plot_water_top_down.py --input artifacts/water_my_build
ros2 launch semantic_mapping water_demo.launch.py bag:="$PWD/artifacts/water_my_build/replay"
```

New files: `semantic_mapping/water_detection.py`, `test/test_water_detection.py`,
`tools/build_water_replay.py`, `tools/plot_water_top_down.py`,
`config/water_demo.rviz`, `launch/water_demo.launch.py`, this document.

`CMakeLists.txt` gained one test line; it was backed up and hash-verified in
`backups/before_water_layer_2026_11_02/`. The suite now has 61 tests, all
passing.
