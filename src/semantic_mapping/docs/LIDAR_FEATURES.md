# LiDAR-only features — 2 November 2026

This stage lets the LiDAR distinguish terrain features by itself, without the
camera. It covers three steps:

1. **Permeability and reflectance per voxel**: how often laser rays pass
   *through* a voxel's measured surface, and its mean raw intensity.
2. **Shape features**: covariance eigenvalues of the measured points in each
   voxel and in its 3×3×3 neighbourhood (1.05 m).
3. **A check against the human-labelled RELLIS LiDAR scan.**

The features feed a rule-based LiDAR-only class for each above-ground voxel:
low vegetation, tall vegetation, trunk-like, solid obstacle or unknown. The
thresholds were chosen beforehand and were **not fitted** to any labelled data.

## Method

`semantic_mapping/lidar_features.py` keeps, for every voxel, the count, sum and
outer products of its measured points, an intensity sum and a pass-through
count. Statistics accumulate scan by scan. At each frame, features use only the
scans up to that frame.

- **Pass-through (permeability)**: for each ray and each stored voxel it
  crosses, compute the closest approach of the measured ray segment to the
  voxel's point distribution, as a Mahalanobis distance with 5 cm
  regularisation. It counts as a pass only if:
  - the ray comes within 3σ of the distribution;
  - the ray ends *beyond* the distribution, so a ray ending on the same surface
    in a neighbouring cell is a hit, not a pass;
  - the voxel is not the ray's own endpoint.

  Permeability = passes / (hits + passes). Foliage lets rays through; trunks,
  rocks and walls do not. This follows the idea behind OHM's NDT traversability
  mode, implemented here in Python so it is transparent and tested.
- **Shape**: from eigenvalues λ1 ≥ λ2 ≥ λ3, linearity = (λ1−λ2)/λ1,
  planarity = (λ2−λ3)/λ1 and scattering = λ3/λ1. Verticality is the vertical
  component of the main direction (for trunks) or of the normal (for ground).
- **Intensity**: raw sensor value, not calibrated for range or incidence angle.
  It is saved and shown, but not used by the class rules.
- **Classes** (`lidar_classes`). Height above ground comes from
  `terrain_analysis`. A voxel needs at least 3 points and at least 5 hits plus
  passes:
  - in the ground layer → ground;
  - linear and vertical (linearity ≥ 0.6, verticality ≥ 0.8 at either scale)
    and not permeable → trunk-like;
  - permeable (≥ 0.25) → vegetation: low below 1.0 m, tall above;
  - impermeable and above the 0.3 m clearance → solid obstacle;
  - otherwise unknown.

Tests: `test/test_lidar_features.py`, six synthetic cases with known answers.
They cover a line, a plane and a blob; rays through sparse foliage versus a
solid wall; a ray grazing over thin ground; a ray stopping short of a surface;
and the class rules.

## Result 1 — labelled RELLIS scan (one scan, sanity check)

`tools/check_lidar_features_rellis.py` uses `utils/example/000104.bin` and
`.label` (Ouster). It applies the same 10 m range limit as the mapping pipeline,
leaving 49,848 points in 2,388 voxels. The LiDAR classes accepted for each
RELLIS label are fixed in `config/rellis_lidar_expectations.json`, written
before running.

| RELLIS class (points) | Consistent LiDAR class | Main LiDAR classes | Median permeability |
|---|---:|---|---:|
| grass (14,550) | 99% | ground 93%, low vegetation 6% | 0.48 |
| concrete (12,162) | 97% | ground 97% | 0.52 |
| tree (4,075) | 4% | ground 49%, low vegetation 35%, solid 9% | 0.00 |
| bush (1,760) | 60% | low vegetation 59%, ground 21% | 0.23 |
| puddle (1,258) | 98% | ground 98% | 0.52 |
| person (1,123) | 82% | trunk-like 79% | 0.00 |

Overall, 85% of scored points get a consistent class. Ground surfaces dominate
that figure, so read the per-class rows instead:

- **Ground and upright solid objects separate well.** The person is linear and
  vertical (linearity 0.69) and impermeable, so it is labelled trunk-like.
  This is geometrically right, but LiDAR alone cannot tell a person from a pole.
- **Bush** is partly found (median permeability 0.23, close to the 0.25
  threshold).
- **Tree fails (4%).** Within 10 m, the "tree" points in this scan are mostly
  low understory only about 0.3 m above the grass (median z −0.92 m against
  −1.24 m for grass). Often no ground is visible beneath them, so the
  lowest-voxel ground estimate takes them as ground. A single scan also gives
  little pass-through evidence (median permeability 0). The thresholds were
  **deliberately not re-tuned** on this one scan.
- Ground permeability is about 0.5 because grazing rays pass close to thin
  ground. This doesn't affect the classes, since ground is decided by height,
  but permeability is only meaningful above ground.
- The unlabelled points around the sensor are the RELLIS vehicle's own body,
  the same self-return effect found in TartanDrive.

Evidence in `artifacts/rellis_lidar_features_2026_11_02/`: `report.json` (full
table), `feature_medians.png` and `top_down_comparison.png`.

## Result 2 — TartanDrive excerpt (accumulated over 318 scans)

`tools/build_lidar_feature_replay.py` re-reads the original sensor bag with the
offline run's own poses, 10 m range and vehicle box. It adds `/lidar/features`
and `/lidar/legend` to a copy of the terrain-demo replay. All map voxels are
published, with ground-layer voxels as ground; the 5,088 original
messages are copied unchanged. The run took 2 min 36 s.

The final map has 11,720 above-ground voxels:

| LiDAR-only class | Voxels |
|---|---:|
| Low vegetation | 4,319 |
| Tall vegetation | 3,504 |
| Solid obstacle | 1,726 |
| Trunk-like | 185 |
| Unknown (too little evidence) | 1,986 |

There are no Tartan labels, so the useful comparison is with the camera, an
**independent** source:

| Camera's confirmed class (above ground) | Voxels | LiDAR says vegetation | LiDAR says solid | Median permeability |
|---|---:|---:|---:|---:|
| Vegetation | 2,724 | 84% (low 46%, tall 38%) | 13% | 0.49 |
| Obstacle | 21 | 76% | 5% | 0.51 |
| Unknown (no confirmed class) | 8,970 | 62% | 15% | 0.44 |

- Where the camera is confident about vegetation, LiDAR permeability and shape
  agree 84% of the time. Two separate sensors and methods support the label.
- The camera's few "obstacle" voxels are mostly permeable to the laser. Either
  the camera prediction is wrong (as with the distant field predicted as
  obstacle in the labelled RELLIS camera frame), or permeability is misleading
  there. This is a disagreement worth showing, not a resolved result.
- **LiDAR gives a class to many voxels the camera never confirmed.** Only 24%
  of above-ground voxels have a confirmed camera class; LiDAR features cover
  most of the rest. This is where LiDAR-only features add coverage.

Agreement between two unvalidated sources is not accuracy.

## RViz

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=112
export ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping lidar_features_demo.launch.py   # rate:=0.5 for slower
```

Displays in `config/lidar_features_demo.rviz` (the terrain demo layout is
unchanged):

| Display | Default | Meaning |
|---|---|---|
| LiDAR-only classes (ground + above ground) | on | Brown ground, light green low vegetation, dark green tall vegetation, violet trunk-like, red solid obstacle, gray unknown |
| Traversability carpet (optional) | off | Green / amber / red traversability. Off by default because its colours mean something else; don't show both at once |
| LiDAR permeability | off | Dark = solid, bright cyan = see-through |
| LiDAR linearity | off | Bright = trunk- or pole-like |
| Camera classes above ground (compare) | off | The camera's labels for the same voxels |
| LiDAR-only classes: legend | on | Class shares and scan number |

For a demonstration, switch between "LiDAR-only classes" and "Camera classes
above ground (compare)". Then turn on "LiDAR permeability" to show *why* LiDAR
calls something vegetation. Each display can be clicked with RViz's Select
tool to read every field of a voxel.

## Rebuild commands

```bash
/usr/bin/python3 src/semantic_mapping/tools/check_lidar_features_rellis.py \
  --output artifacts/rellis_lidar_my_check                      # seconds
/usr/bin/python3 src/semantic_mapping/tools/build_lidar_feature_replay.py \
  --run artifacts/offline_demo_ego_filtered_2026_11_02 \
  --replay artifacts/terrain_demo_2026_11_02/replay \
  --output artifacts/lidar_features_my_build                    # ~3 min
ros2 launch semantic_mapping lidar_features_demo.launch.py \
  bag:="$PWD/artifacts/lidar_features_my_build/replay"
```

## Files and backup

New files: `semantic_mapping/lidar_features.py`, `test/test_lidar_features.py`,
`tools/check_lidar_features_rellis.py`, `tools/build_lidar_feature_replay.py`,
`config/rellis_lidar_expectations.json`, `config/lidar_features_demo.rviz`,
`launch/lidar_features_demo.launch.py`, this document.

The only edited existing file is `CMakeLists.txt`, which registers the test.
It was backed up and hash-verified in
`backups/before_lidar_features_2026_11_02/`. No mapping, model, fusion or
existing launch files changed. The test suite now has 57 tests, all passing.

## Limits and next steps

- Heights use 0.35 m voxels and a lowest-voxel ground estimate. Low dense
  understory can be mistaken for ground (the RELLIS tree result). A proper ground
  segmentation, such as Patchwork++ or a cloth simulation filter, is the next
  improvement.
- Permeability needs several viewpoints; a single scan gives sparse evidence.
- Intensity is uncalibrated and unused in the rules.
- Only one labelled LiDAR scan exists on this PC. Larger RELLIS-3D or GOOSE
  LiDAR label sets would allow real per-class precision and recall, and fitted
  or learned thresholds.
