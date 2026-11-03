# Accuracy improvements, v2 — 3 November 2026

Steps 1–4 of the improvement plan, each a separately selectable option. The v1
commands and results are unchanged and still reproduce.

| Step | Change | Where |
|---|---|---|
| 1 | Camera model input keeps the image aspect ratio (512×960 for Tartan instead of a 512×512 squash) | `semantic_mapping/goose_inference.py`, `--inference-short-side 512` |
| 2 | Vehicle hood masked in the camera image: no labels from those pixels | `config/tartan_image_mask.json`, `--image-mask` |
| 3 | 0.2 m voxels and 20 m range (were 0.35 m and 10 m) | `--resolution 0.2 --max-range 20` |
| 4 | Cloth-simulation ground filter instead of lowest voxel | `ground_method: cloth` in `config/traversability_v2.yaml` |
| + | Traversability also relaxes obstacles that LiDAR finds permeable (low or tall vegetation). Camera water on LiDAR-returned surfaces becomes caution, not a hard stop. | `analyze(..., compliant=)`, `hazard_cost: 0.6` |

## Step 1 — camera input, measured on the labelled RELLIS frame

| Model input | Pixel accuracy | Mean IoU | Vegetation | Obstacle | Water | Ground |
|---|---:|---:|---:|---:|---:|---:|
| 512×512 squash (v1) | 0.60 | 0.28 | 0.71 | 0.25 | 0.13 | 0.03 |
| **512×832, aspect kept (v2)** | **0.68** | **0.36** | **0.82** | **0.48** | 0.12 | 0.03 |
| 768×1216, aspect kept | 0.69 | 0.36 | 0.78 | 0.37 | 0.25 | 0.06 |
| 512×512 + mirror average | 0.62 | 0.30 | 0.74 | 0.31 | 0.11 | 0.03 |
| 768×1216 + mirror average | 0.66 | 0.33 | 0.77 | 0.33 | 0.18 | 0.05 |

The 512 short side was chosen: it equals 768 in mean IoU at less than half the
CPU cost. Mirror averaging was **rejected** because it lowered the scores once
the aspect ratio was kept. This is one frame, so the numbers are indicative,
not dataset accuracy. Evidence:
`artifacts/segmentation_input_check_2026_11_03/comparison.json`.

**Water remains unreliable with either input.** Two Tartan examples from the
same folder:

- `frame00201_old_false_water.jpg`: the old input painted dry brown grass as
  water. The new one does not.
- `frame00235_new_false_water.jpg`: the new input paints dark, shaded, damp
  soil at the bend as a large water area.

Most of the v2 water lies on surfaces where LiDAR *does* return points (759 of
879 cells). That argues against standing water. The display now separates:

- **water**: camera water *and* no LiDAR return (bright blue, 120 cells);
- **camera-only water**: LiDAR sees a surface, so it is doubtful (light blue,
  759 cells).

## Steps 2–4 — results on the TartanDrive excerpt

| | v1 (2 Nov) | v2 (3 Nov) |
|---|---:|---:|
| Voxel / range | 0.35 m / 10 m | 0.2 m / 20 m |
| Occupied voxels | 24,370 | 182,996 |
| Voxels with a confirmed camera class | 5,774 | 31,056 |
| Share of occupied voxels confirmed | 23.7% | 17.0% (bigger map; camera sees less of it) |
| Mapped ground columns | 12,819 | 85,271 |
| Vehicle height above mapped ground, 5–95% | 1.83–2.16 m | **1.81–2.01 m** |
| Vehicle tilt vs. mapped ground normal, median / 95% | 3.5° / 9.1° | **2.7° / 6.0°** |
| Driven path non-traversable, camera semantics | 0.3% | 14.2% |
| Driven path non-traversable, **camera + LiDAR vegetation** | — | **1.6%** |
| Driven path with no mapped cell under the vehicle | 9.7% | **2.8%** |
| Above-ground voxels the camera calls vegetation that LiDAR also calls vegetation | 84% | 80% |
| Processing (this PC) | 4.4 min | 14 min, then about 30 min for all layers |

How to read the table:

- **More detail and coverage:** finer voxels and 20 m range give 7.5× more
  map, 5.4× more confirmed camera labels and far fewer blind spots under the
  vehicle.
- **Internal consistency improved:** the vehicle's height above the mapped
  ground varies half as much, and the ground tilt matches the vehicle tilt
  better. This is the clearest geometric quality gain.
- **Why camera semantics alone raised path false alarms (0.3% → 14.2%):** at
  0.2 m, tall grass shows up as obstacle layers, and most grass voxels are not
  confirmed by the camera. Adding LiDAR permeability lets traversability
  recognise them as vegetation, which cuts false alarms to **1.6%**. This
  camera + LiDAR fusion is the result to present.
- **Step 4, cloth ground:** on the labelled RELLIS scan, bush improves from 60%
  to 64% consistent, but tree stays at about 5%. The labelled "tree" points
  within 10 m are low branches about 0.3 m above grass, which no ground method
  separates at voxel scale. This is a label and resolution limit.

## RViz

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=112
export ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping demo_v2.launch.py rate:=0.5
```

`config/demo_v2.rviz` uses 0.2 m boxes. By default it shows LiDAR-only classes
and water (bright = camera + no LiDAR return; light = camera only). Optional
displays, best turned on one at a time:

- traversability from camera semantics only;
- **traversability fused (camera + LiDAR vegetation)**;
- geometry only;
- permeability and linearity;
- camera classes above ground and projected on the ground.

With about 180k voxels, RViz may run below 20 fps. Use `rate:=0.5`.

## Rebuild everything

```bash
src/semantic_mapping/tools/build_v2_demo.sh MY_SUFFIX     # ~45 min, ~21 GB derived data
ros2 launch semantic_mapping demo_v2.launch.py bag:="$PWD/artifacts/water_demo_v2_MY_SUFFIX/replay"
```

Each replay copies the previous one, so the four v2 folders hold about 21 GB.
Only `artifacts/water_demo_v2_*` is needed for viewing. The other replay
folders can be deleted after checking, but keep the `offline_v2_*` run, since
all layers are rebuilt from it.

## Files and backup

- **New files:** `semantic_mapping/goose_inference.py`,
  `test/test_goose_inference.py`, `config/tartan_image_mask.json`,
  `config/traversability_v2.yaml`, `config/demo_v2.rviz`,
  `launch/demo_v2.launch.py`, `tools/build_v2_demo.sh`, this document.
- **Edited, with options added and defaults unchanged:** `build_offline_demo.py`,
  `check_offline_demo.py` (reads the resolution), `terrain_analysis.py`,
  `lidar_features.py`, the four layer tools, the two RELLIS checks and
  `CMakeLists.txt`. All were backed up and hash-verified first in
  `backups/before_accuracy_v2_2026_11_03/`.
- **Tests:** 65, all passing. New tests cover the aspect size, mirror and mask,
  cloth bridging, LiDAR compliance and hazard cost.

## Still open

- Water: camera false positives on dark wet soil. Only camera + no-LiDAR-return
  water should be presented as likely water.
- Calibration accuracy: step 5 (clicked point pairs) is still needed.
- Accuracy numbers: a larger labelled set is needed (step 6, RELLIS-3D
  download).
- Vehicle limits are still placeholders, not POLARIS values.
