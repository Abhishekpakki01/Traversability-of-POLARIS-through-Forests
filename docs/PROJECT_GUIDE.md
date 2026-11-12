# Project guide — 3 November 2026

Repository: `~/Traversability-of-POLARIS-through-Forests`
The original workspace `~/semantic_map_ws` is kept separately as a working comparison.

---

## 1. How to run things: one terminal

### A. Watch the demo in RViz (needs no setup)

Open **one** Ubuntu terminal and run:

```bash
cd ~/Traversability-of-POLARIS-through-Forests
src/semantic_mapping/tools/view_demo.sh explore
```

The script sources ROS, starts RViz and plays the recording, all in this
terminal.

| You want | Command |
|---|---|
| Whole map, free camera, large Displays list | `src/semantic_mapping/tools/view_demo.sh explore` |
| Follow the vehicle, with camera panels | `src/semantic_mapping/tools/view_demo.sh` |
| GPU not working (CPU drawing) | `src/semantic_mapping/tools/view_demo.sh light` |
| Slower playback | add a rate, e.g. `src/semantic_mapping/tools/view_demo.sh explore 0.25` |
| **Run the pipeline now** from the raw recording (first 30 scans, about 3 minutes), then show it | `src/semantic_mapping/tools/quick_run.sh` (or `quick_run.sh 60` for more scans) |

While it plays, type these **in the terminal**: SPACE pauses/resumes, RIGHT
ARROW steps one message, UP/DOWN change speed.

In **RViz**:
- left-drag rotates, middle-drag pans, and the scroll wheel zooms;
- tick or untick layers in the Displays list.

When playback ends, RViz stays open with the final map. Press **Ctrl+C** in the
terminal to close everything. Then you can run the next command in the same
terminal.

### B. Any other command: paste the setup block once, then run commands one after another

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=112
export ROS_LOCALHOST_ONLY=1
```

You need the setup block again only in a **new** terminal. A second terminal
is needed only if you want to run something *while* a demo is still playing.

**Accuracy checks** (seconds each; each output folder must not exist yet):

```bash
/usr/bin/python3 src/semantic_mapping/tools/check_trajectory_gps.py --run artifacts/offline_v2_2026_11_03 --output artifacts/my_gps_check
/usr/bin/python3 src/semantic_mapping/tools/check_map_sharpness.py --run artifacts/offline_v2_2026_11_03 --output artifacts/my_sharpness_check
/usr/bin/python3 src/semantic_mapping/tools/check_labelled_frame.py --inference-short-side 512 --output artifacts/my_camera_check
/usr/bin/python3 src/semantic_mapping/tools/check_lidar_features_rellis.py --terrain-config src/semantic_mapping/config/traversability_v2.yaml --output artifacts/my_lidar_check
cat artifacts/my_gps_check/report.json          # read any result like this
```

**Rebuild all v2 results from the recording** (about 45 minutes, about 22 GB):

```bash
src/semantic_mapping/tools/build_v2_demo.sh MY_SUFFIX
src/semantic_mapping/tools/view_demo.sh explore   # the viewer opens the 2026_11_03 results; see the note below
```

> To view a rebuilt suffix: `ros2 launch semantic_mapping demo_v2.launch.py layout:=demo_v2_explore.rviz bag:="$PWD/artifacts/water_demo_v2_MY_SUFFIX/replay"`

**Compile after code edits, then run all 68 tests:**

```bash
colcon build --packages-select semantic_mapping --symlink-install
source install/setup.bash
OHM_OFFLINE_EXECUTABLE=$PWD/install/semantic_mapping/lib/semantic_mapping/ohm_offline_mapper /usr/bin/python3 -m pytest -q src/semantic_mapping/test
```

**Older v1 views** (still working; Ctrl+C to stop):

```bash
ros2 launch semantic_mapping terrain_demo.launch.py rate:=0.5          # traversability carpet
ros2 launch semantic_mapping lidar_features_demo.launch.py rate:=0.5   # LiDAR-only classes
ros2 launch semantic_mapping water_demo.launch.py rate:=0.5            # water layer
```

**Make pictures and videos for sharing** (about 12 minutes; RViz windows open and close by themselves):

```bash
/usr/bin/python3 src/semantic_mapping/tools/capture_demo_media.py --output artifacts/my_media
```

See `src/semantic_mapping/docs/RVIZ_MEDIA.md`.

**Check the GPU** (if RViz is slow):

```bash
nvidia-smi                               # should list the GeForce GTX 1050
glxinfo -B | grep -i renderer            # needs: sudo apt install mesa-utils; should say D3D12 (NVIDIA ...)
```

If it says `llvmpipe`: run `wsl --shutdown` in Windows PowerShell and reopen
Ubuntu, or restart the PC.

---

## 2. What was done (2–3 November)

The offline demo itself (sequential processing without dropped frames) was
built with Codex on 2 November. Everything below came afterwards.

| # | Work | Result |
|---|---|---|
| 1 | Rebuilt the offline demo with the newest code | Identical in all 318 frames: reproducible |
| 2 | Found that **about 5% of LiDAR points, and 20% within 10 m, were the vehicle itself** and added a filter | False obstacles along the driven path removed (path false alarms 98% → 3.5%) |
| 3 | Terrain analysis: ground, slope, roughness, obstacle height, traversability | Limits are **placeholders**, not POLARIS values |
| 4 | LiDAR-only features: permeability (how see-through), shape (line, plane or scatter) | Agrees with the camera on vegetation 80–84% of the time |
| 5 | Water layer: camera water pixels projected onto the ground and combined with LiDAR gaps | 3 water voxels became a visible water layer; camera water is **unreliable** (false water on dark wet soil) |
| 6 | Camera model input keeps the aspect ratio, and the vehicle hood is masked | Labelled frame: accuracy 0.60 → 0.68, obstacle IoU 0.25 → 0.48 |
| 7 | Finer map: 0.2 m voxels, 20 m range, cloth-simulation ground | 7.5× more map, 5.4× more labelled voxels |
| 8 | Camera + LiDAR fusion for traversability | Driven-path false alarms 14% → **1.6%** |
| 9 | Map poses checked against **independent GPS/INS** | Within 6–18 cm, map vertical within 0.35°, drift 0.4% per 10 m |
| 10 | Map sharpness | Ground 2.8 cm thick over all scans, against 1.9 cm in one scan |
| 11 | RViz usability | GPU confirmed, light/explore layouts, `view_demo.sh` keeps playback under terminal control |

Evidence folders are in `artifacts/`, and explanations in
`src/semantic_mapping/docs/`: `TERRAIN_AND_VALIDATION.md`,
`LIDAR_FEATURES.md`, `WATER_DETECTION.md`, `ACCURACY_V2.md`,
`MAPPING_ACCURACY.md`.

---

## 3. Code changes (all under `src/semantic_mapping/`)

Rule followed throughout: new behaviour goes into **new files or opt-in
options**, so the old commands give the old results. Every edited file was
backed up first and hash-verified in `backups/before_*_2026_11_0*/`.

### New core modules (`semantic_mapping/`)

| File | What it does |
|---|---|
| `terrain_analysis.py` | Per 0.2/0.35 m column: ground height (lowest voxel or cloth filter), slope and roughness from a plane fit, obstacle height. Geometry-only and geometry+semantics traversability states. Driven-path and vehicle-consistency checks. |
| `lidar_features.py` | Per voxel: point statistics, raw intensity, pass-through count (rays that go *through* a surface). Gives permeability, linearity, planarity, scattering and verticality, and from them the LiDAR-only classes. |
| `water_detection.py` | Casts camera pixel rays onto the mapped ground (fills small LiDAR holes, drops occluded rays) and accumulates water evidence per ground cell. |
| `goose_inference.py` | Runs the same GOOSE model with the correct image aspect ratio, an optional mirror average (tested and rejected) and image masks such as the hood. |

### New tools (`tools/`)

| File | What it does |
|---|---|
| `find_vehicle_self_returns.py` | Measures the box of LiDAR points that stay fixed to the vehicle (its own body) |
| `analyze_terrain.py` | Terrain report and figures for one run |
| `build_terrain_replay.py` | Adds the traversability carpet to a replay bag |
| `build_lidar_feature_replay.py` | Adds LiDAR-only classes and fused traversability to a replay bag |
| `build_water_replay.py`, `plot_water_top_down.py` | Water layer and its figure |
| `check_labelled_frame.py` | Camera prediction against the labelled RELLIS image |
| `check_lidar_features_rellis.py` | LiDAR classes against the labelled RELLIS scan |
| `check_trajectory_gps.py` | Map poses against GPS/INS |
| `check_map_sharpness.py` | Ground thickness: single scan, all scans, deliberately perturbed poses |
| `build_v2_demo.sh` | Rebuilds the whole v2 chain with one command |
| `view_demo.sh` | Opens RViz so you control playback from your terminal |
| `quick_run.sh` | Runs the whole v2 chain live on the first N raw scans into `~/thesis_runs/`, then opens RViz |
| `capture_demo_media.py`, `capture_rviz_window.ps1` | Pictures and videos of RViz with the camera beside the map |
| `prepare_calibration_check.py`, `click_calibration_check.py`, `click_calibration.sh` | Clicked camera–LiDAR alignment check |
| `check_outline_alignment.py`, `nudge_calibration_check.py`, `nudge_calibration.sh` | Camera–LiDAR offset from outlines against the sky, and a check by eye |
| `check_outline_alignment_rellis.py`, `check_label_alignment_rellis.py` | Control tests on RELLIS-3D; the outline method failed its control |
| `build_results_pack.py` | Collects result numbers and figures, each traced to its source report |
| `diagnose_rellis_coverage.py` | Finds why tree and bush voxels get no class, on development sequences only |
| `lidar_features.fill_sparse`, `evaluate_rellis_lidar_map.py --fill-sparse` | Fills sparse voxels from classified neighbours; tested on held-out sequences |

### Existing files that were edited (options added, defaults unchanged)

| File | Change |
|---|---|
| `tools/build_offline_demo.py` | `--exclude-vehicle-box`, `--resolution`, `--max-range`, `--inference-short-side`, `--image-mask`, `--threads` |
| `tools/check_offline_demo.py` | Reads the voxel size from the run instead of assuming 0.35 m |
| `src/ohm_occupancy_pose_node.cpp` | Optional `vehicle_box` parameter removes the vehicle's own returns; off by default |
| `scripts/offline_demo_view.py` | Also crops the LiDAR layer around the vehicle, can cap the update rate, shows the final map after playback, and shuts down cleanly |
| `CMakeLists.txt` | Registers the new tests |

### New configuration (`config/`)

| File | Contents |
|---|---|
| `traversability_placeholder.yaml`, `traversability_v2.yaml` | Vehicle limits (**placeholders**), ground method (`cloth` in v2), hazard cost |
| `tartan_image_mask.json` | Hood outline in the camera image |
| `rellis_label_groups.json`, `rellis_lidar_expectations.json` | How RELLIS labels are compared with our classes (fixed before testing) |
| `terrain_demo.rviz`, `lidar_features_demo.rviz`, `water_demo.rviz`, `demo_v2.rviz`, `demo_v2_light.rviz`, `demo_v2_explore.rviz` | RViz layouts |

New launch files: `terrain_demo`, `lidar_features_demo`, `water_demo` and
`demo_v2` (`.launch.py`).

New tests: 30 more (68 in total, all passing) in `test/test_terrain_analysis.py`,
`test_lidar_features.py`, `test_water_detection.py`,
`test_goose_inference.py`, `test_trajectory_gps.py` and `test_map_sharpness.py`.

---

## 4. Still to do

| Priority | Task | Status / needs |
|---|---|---|
| 1 | Thesis writing (deadline 23 November) | Author's own work; not started in the repo |
| 2 | POLARIS limits (clearance, maximum slope, step height, dimensions) | Supervisor or vehicle documentation; placeholders until then |
| 3 | Supervisor decisions: GOOSE weights, AI-use approval, evaluation scope, real-time scope | Supervisor reply |
| 4 | Camera–LiDAR alignment on TartanDrive | Still unmeasured. RELLIS-3D has a label-based reference (0.0° horizontal, −0.5° vertical); TartanDrive needs a recording with clear vertical structures |
| 5 | A forest recording, since the meadow has almost no trees | Optional; download approval (~1 GB). Tree classes are covered by RELLIS-3D |
| 6 | Rebuild the TartanDrive maps with `fill_sparse` | Optional; the demos and videos were made without it |

Done since the first version of this list: RELLIS-3D class accuracy (camera
test split and LiDAR voxels, see `docs/RELLIS_EVALUATION.md`) and the RViz
pictures and videos (`docs/RVIZ_MEDIA.md`).

Optional disk clean-up (keep `offline_v2_*` and `water_demo_v2_*`):
`artifacts/offline_demo_rerun_2026_11_02` (2.3 GB duplicate),
`artifacts/terrain_demo_v2_2026_11_03` and
`artifacts/lidar_features_demo_v2_2026_11_03` (about 11 GB, intermediate replays).
