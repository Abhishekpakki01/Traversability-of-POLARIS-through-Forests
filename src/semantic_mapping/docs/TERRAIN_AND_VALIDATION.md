# Terrain analysis and validation checks — 2 November 2026

This stage adds four things on top of the offline demo (`OFFLINE_DEMO.md`):

1. A reproducibility check of the offline run.
2. Removal of LiDAR returns from the recording vehicle's own body (optional flag).
3. Terrain geometry (ground height, slope, roughness, obstacle height) and a
   geometry-only and geometry+semantics traversability state, with two
   self-checks against the recorded drive.
4. A comparison of GOOSE predictions with the one human-labelled RELLIS frame
   available on this PC.

Nothing here commands a vehicle. **The vehicle limits are placeholders, not
POLARIS specifications**, so the states illustrate the method; they are not a
POLARIS driving decision.

## 1. Reproducibility of the offline run

The uncommitted `NearestImage` change (camera exposures must strictly increase)
was made after the saved run. The full run was rebuilt with the current code into
`artifacts/offline_demo_rerun_2026_11_02/` and compared with
`artifacts/offline_demo_tartan_2026_11_02/`:

- every report field except timing is identical;
- all 318 camera timestamps are identical;
- all arrays in all 318 saved frames (geometry, labels, confidence,
  observations, poses) are identical.

Evidence: `artifacts/reproducibility_check_2026_11_02/comparison.json`. The
2.3 GB rebuilt folder can be deleted; the comparison file is the record.

## 2. Ego-vehicle self-returns (a mapping fault that was found)

**Finding.** The terrain self-check (section 3) showed 98% of driven poses as
non-traversable. The cause: LiDAR returns from the recording vehicle itself. Measured over 40 scans,
about **5.2% of all returns and 19.5% of returns within 10 m** fall in the vehicle box
(corrected on 6 November from an earlier figure of 28%, which could not be reproduced). Those returns were integrated as occupied voxels at the LiDAR's
height (~2 m above ground) all along the route — the "long map trail" in the
offline demo is the vehicle body, not terrain. The live launches have the same
fault; they were not changed.

**How it was measured.** Terrain moves through the vehicle frame while driving;
the vehicle's body does not. `tools/find_vehicle_self_returns.py` samples every
4th scan (80 scans), puts points into 0.2 m vehicle-frame cells, and reports the
cells occupied in more than 80% of scans: 68 cells within x −1.6…0.2 m,
y −1.4…1.2 m, z −0.8…0.2 m (LiDAR at the vehicle origin; ground at about
z = −2 m). With a 0.2 m margin it proposes the box
`-1.8 -1.6 -1.0 0.4 1.4 0.4`. After cropping, no cell is persistent above 80%;
the only cells above 50% are the first ground ring about 3 m away (real terrain).

**Change.** `build_offline_demo.py` gained `--exclude-vehicle-box XMIN YMIN ZMIN
XMAX YMAX ZMAX` (vehicle frame). It removes those points from both geometry and
camera depth filtering, counts them (`vehicle_box_points_removed`), and records
the box in `report.json` (`vehicle_box_m`). **It is off by default**, so the
previous command reproduces the previous result exactly.

**Effect** (whole excerpt, same model/thresholds):

| | Original run | Vehicle box removed |
|---|---:|---:|
| Scans mapped / fused | 318 / 318 | 318 / 318 |
| Points removed by box | 0 | 613,790 |
| Final occupied voxels | 26,325 | 24,370 |
| Confirmed class labels | 5,774 | 5,774 (identical counts) |
| Unknown (gray) voxels | 20,551 | 18,596 |
| Saved-data audit (`check_offline_demo.py`) | passed | passed |

All 1,955 removed voxels were unknown-class: the body is outside the camera
view, so semantic labels did not change. Points inside the box that were real
terrain (e.g. a branch touching the roll cage) are also removed; this is a
deliberate, documented trade-off. The box applies only to this recording's
vehicle and sensor mount — **POLARIS needs its own measurement** with the same
tool.

## 3. Terrain geometry and traversability states

Files: `semantic_mapping/terrain_analysis.py` (method),
`tools/analyze_terrain.py` (runner, report, figures),
`config/traversability_placeholder.yaml` (limits),
`test/test_terrain_analysis.py` (12 synthetic tests).

### Method, per 0.35 m map column

- **Ground**: the lowest occupied voxel, unless it is far above the local lowest
  surface within 1.5 m (more than tan 35° × 1.5 m + one voxel). Such columns,
  e.g. canopy-only, take the mean of nearby measured ground. Columns without
  any voxel stay **unobserved**; free space is never filled.
- **Slope and roughness**: least-squares plane through measured ground in a
  7×7-cell (2.45 m) window; slope from its gradient, roughness as residual
  standard deviation.
- **Obstacle height**: tallest voxel above ground but below the overhang
  clearance (2.0 m); higher voxels are treated as passable canopy.
- **Geometric cost** = max(slope / max slope, roughness / max roughness,
  obstacle height / ground clearance), clipped to 1. Cost ≥ 1 is
  non-traversable, ≥ 0.5 caution, otherwise traversable.
- **Semantics** (confirmed labels only: ≥ 2 observations, ≥ 0.6 confidence):
  trunk or obstacle labels between ground and overhang make a column
  non-traversable; water at or above the ground makes it non-traversable; a
  geometric obstacle consisting entirely of confirmed vegetation no taller than
  1.0 m is relaxed to cost 0.6 (caution). Unknown-label voxels keep the
  geometric decision. Geometry-only and combined states are both saved, so the
  effect of semantics can be reported separately.

### Resolution limits (from the synthetic tests)

- Heights come in 0.35 m layers: one layer is the smallest resolvable obstacle.
- Slope: medians within about 2°; single windows alias by up to about 5.5°.
- Roughness has a quantization floor of about 0.1 m (0.35 / √12).
- Map Z is assumed gravity-up; this has not been independently verified.

### Self-checks against the recorded drive

1. **Driven path** — the recording vehicle drove through these cells, so they
   are positive (traversable) examples *for that vehicle*. The worst state in a
   0.6 m footprint around each of the 318 poses measures false alarms. It does
   **not** measure hazard detection; there are no negative examples.
2. **Vehicle consistency** — the vehicle origin's height above the mapped
   ground, and the angle between the vehicle's up-axis and the mapped ground
   normal. The map and the vehicle share the same recorded poses, so this tests
   internal consistency, not absolute odometry accuracy.

### Results on the TartanDrive excerpt

| Driven-path worst state (318 poses) | Original run | Vehicle box removed |
|---|---:|---:|
| Non-traversable, geometry only | 98.1% | 3.5% |
| Non-traversable, geometry + semantics | 98.1% | 0.3% |
| Caution, geometry + semantics | 0% | 68.2% |
| Traversable, geometry + semantics | 0.3% | 21.7% |
| Unobserved footprint | 0% | 9.7% |

| Vehicle consistency (box removed) | Median | 5–95% |
|---|---:|---:|
| Origin height above mapped ground | 2.00 m | 1.83–2.16 m |
| Up-axis vs. ground normal | 3.5° | 0.7–9.1° |

How to read these:

- Before the fix, the vehicle's own body made almost every driven cell an
  obstacle, and the vehicle height above ground ranged down to 0.16 m —
  the consistency check exposed the fault.
- With the box removed, the height above ground is stable (±0.17 m, about
  half a voxel) and the tilt agrees with the mapped ground to a few degrees:
  map geometry and poses are internally consistent over the 81 m drive.
- **Caution** on the path is mostly slope: 175 of 217 caution poses are
  dominated by slope (path cells: median 10°, 95th percentile 16°, against the
  20° placeholder), 32 by roughness and 10 by obstacle height. This matches a
  downhill meadow with 11.6 m of recorded descent.
- **Semantics** reduce path false alarms from 3.5% to 0.3% by relaxing 412
  columns of confirmed low vegetation. This depends on the assumption that low
  vegetation is drivable; a bush can hide a rock.
- **Unobserved** footprints (31 poses, spread over the drive) are cells the
  LiDAR never saw: it is about 2 m up and its first ground ring is about 3 m
  out. They were previously hidden by self-return voxels.
- Over all observed columns: 12.9% traversable, 50.5% caution and 36.6%
  non-traversable (geometry + semantics). Off-path areas include dense
  vegetation; with no labelled terrain, these are not accuracy figures.

Figures (box removed) in `artifacts/terrain_analysis_ego_filtered_2026_11_02/`:
`traversability_states.png` (geometry-only and combined, driven path in
black), `terrain_descriptors.png` (slope and obstacle height),
`vehicle_consistency.png`. The same analysis of the original run is in
`artifacts/terrain_analysis_2026_11_02/` for comparison.

## 4. One labelled RELLIS frame

`tools/check_labelled_frame.py` compares grouped GOOSE predictions with the
human-labelled RELLIS example `utils/example/frame000104-1581624663_149`.
`config/rellis_label_groups.json` states how each RELLIS class is grouped.
`tree` is reported but not scored, because RELLIS does not separate trunk from
crown. Sky and void are not scored either.

**One frame is a sanity check, not dataset accuracy.**

| Group | IoU | Precision | Recall |
|---|---:|---:|---:|
| Vegetation (grass, bush) | 0.71 | 0.85 | 0.81 |
| Obstacle (person, fence, object, vehicle) | 0.25 | 0.26 | 0.92 |
| Water (puddle) | 0.13 | 1.00 | 0.13 |
| Ground (mud) | 0.03 | 0.03 | 0.38 |

Pixel accuracy over scored pixels: 0.60.

- The person is found (96% of its pixels are predicted as obstacle).
- **The flooded, muddy track is mostly predicted as ground (61%).** Only the
  central pool is predicted as water. This supports reporting water detection
  as unreliable, consistent with only 3 water voxels in the Tartan map.
- A band of distant field on the left is predicted as obstacle, giving low
  obstacle precision.
- Trees are predicted as vegetation (93%), not trunk. This fits the single trunk
  voxel in the Tartan map.

Evidence: `artifacts/rellis_labelled_frame_check_2026_11_02/`, which contains
`summary.jpg`, the full-resolution panels and `report.json`, including the full
confusion table by RELLIS class.

## Commands

Setup in each terminal:

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=112
export ROS_LOCALHOST_ONLY=1
```

`python3` on this PC resolves to `~/thesis_env`, which lacks pytest. The commands
below use the system Python explicitly.

```bash
# Measure self-returns and propose a box (~1 min; output file must be new)
/usr/bin/python3 src/semantic_mapping/tools/find_vehicle_self_returns.py \
  --output artifacts/my_self_returns/self_returns.json

# Offline run with the box removed (~5 min, ~2.3 GB; output folder must be new)
/usr/bin/python3 src/semantic_mapping/tools/build_offline_demo.py \
  --exclude-vehicle-box -1.8 -1.6 -1.0 0.4 1.4 0.4 \
  --output artifacts/offline_demo_my_filtered_run
/usr/bin/python3 src/semantic_mapping/tools/check_offline_demo.py \
  --input artifacts/offline_demo_my_filtered_run

# Terrain analysis of any offline run (seconds)
/usr/bin/python3 src/semantic_mapping/tools/analyze_terrain.py \
  --input artifacts/offline_demo_my_filtered_run \
  --output artifacts/terrain_my_run

# Labelled RELLIS frame (seconds)
/usr/bin/python3 src/semantic_mapping/tools/check_labelled_frame.py \
  --output artifacts/rellis_my_check

# All tests (50; the environment variable enables the OHM bridge test)
OHM_OFFLINE_EXECUTABLE=$PWD/install/semantic_mapping/lib/semantic_mapping/ohm_offline_mapper \
  /usr/bin/python3 -m pytest -q src/semantic_mapping/test
```

To use real POLARIS limits, copy `config/traversability_placeholder.yaml`, enter
the documented values and set `vehicle_parameters_verified: true`. Then pass it
with `--config`. The runner rejects misspelled keys.

The replay in RViz is unchanged. To view the filtered map, use
`ros2 launch semantic_mapping offline_demo.launch.py bag:="$PWD/artifacts/offline_demo_ego_filtered_2026_11_02/replay"`.

## RViz terrain demo

`tools/build_terrain_replay.py` copies every message of an offline replay
unchanged into a new bag. At each saved map's timestamp it adds terrain layers
computed from that same map, so the traversability map grows scan by scan as
the vehicle drives. This took about 20 s for the filtered run. No model or
terrain analysis runs during playback.

```bash
# Setup block as above, then (the bag is already built on this PC):
ros2 launch semantic_mapping terrain_demo.launch.py
# Optional: slower playback for a meeting
ros2 launch semantic_mapping terrain_demo.launch.py rate:=0.5
# Rebuild the bag from another offline run (output folder must be new)
/usr/bin/python3 src/semantic_mapping/tools/build_terrain_replay.py \
  --input artifacts/offline_demo_ego_filtered_2026_11_02 \
  --output artifacts/terrain_demo_my_build
ros2 launch semantic_mapping terrain_demo.launch.py \
  bag:="$PWD/artifacts/terrain_demo_my_build/replay"
```

What RViz shows by default (`config/terrain_demo.rviz`):

| Display | Meaning |
|---|---|
| Traversability (geometry + semantics) | One 0.35 m cell per mapped ground column, at its ground height. Green traversable, amber caution, red non-traversable, gray no ground plane. |
| Above-ground voxels (class colours) | Occupied voxels above the ground layer: vegetation green, trunk brown, obstacle red, water blue, gray unknown class. |
| Vehicle pose / trail | Recorded motion. |
| Camera, predicted classes, projection | The same camera panels as the offline demo. |
| Traversability: map legend | Live percentages, scan and time, and the placeholder-limits warning. |

Optional displays that are off by default:

- `Traversability (geometry only, optional)`: switch it on, and switch the
  combined one off, to show what the camera semantics change. Low vegetation
  turns from red to amber.
- The earlier local or full semantic map and the elevation view.

Fixed Frame is `sensor_init`, and the view orbits the vehicle. Stop with Ctrl+C
before relaunching, and do not use `--loop`. The terrain clouds use *Volatile*
QoS, which is what `ros2 bag play` provides. If a display stays empty after
editing the layout, check that it isn't set to Transient Local.

A suggested way to describe it: *"LiDAR and recorded poses build an occupancy
map, and camera classes from a pretrained GOOSE model are fused onto the
measured surfaces. From the map I estimate ground height, slope, roughness
and obstacle height per 0.35 m cell, and combine them with the confirmed
classes into a traversability state. I found and removed the vehicle's own
LiDAR returns, which had covered the driven path with false obstacles. The
thresholds are placeholders until we have POLARIS limits. Accuracy is
checked only against the driven path and one labelled RELLIS frame."*

## Changed files and how to go back

New files: `semantic_mapping/terrain_analysis.py`, `tools/analyze_terrain.py`,
`tools/find_vehicle_self_returns.py`, `tools/check_labelled_frame.py`,
`config/traversability_placeholder.yaml`, `config/rellis_label_groups.json`,
`tools/build_terrain_replay.py`, `launch/terrain_demo.launch.py`, `config/terrain_demo.rviz`,
`test/test_terrain_analysis.py`, this document.

Edited existing files:

- `tools/build_offline_demo.py`: the optional `--exclude-vehicle-box` flag. It is
  off by default.
- `CMakeLists.txt`: registers the new test.

Both were backed up first. So was all of today's uncommitted offline-demo work
(12 files, verified by hash), in `backups/before_terrain_analysis_2026_11_02/`.

To return to the previous behaviour, omit `--exclude-vehicle-box`. No restoration
is needed. The original `~/semantic_map_ws` was not touched.

## Still open

- Real POLARIS dimensions and terrain limits, and a POLARIS self-return box.
- The live mapper now has the filter as the `vehicle_box` parameter, but it is off
  by default, so the live launches still integrate self-returns unless it is set.
- No labelled Tartan data: traversability and class accuracy beyond the one
  RELLIS frame are unmeasured. The driven-path check has positive examples only.
- Physical camera calibration and gravity alignment remain unverified.
- Planner integration does not exist.
