# RELLIS-3D evaluation — 4 November 2026

RELLIS-3D has human labels for camera images and LiDAR points. Neither the GOOSE
model nor the LiDAR thresholds were fitted to it. The class grouping
(`config/rellis_label_groups.json`) and the accepted LiDAR classes
(`config/rellis_lidar_expectations.json`) were set before these tests.

The data is in `~/thesis_data/rellis_3d_full/` and is not in Git.

## Camera model on the official test split

```bash
/usr/bin/python3 src/semantic_mapping/tools/evaluate_rellis_images.py --output artifacts/my_rellis_camera
```

All 1,672 images of `test.lst`, which come from sequences 00000 to 00002, with 512×832
model input.

| Group | IoU | Precision | Recall |
|---|---:|---:|---:|
| Vegetation | 0.82 | 0.97 | 0.84 |
| Ground | 0.36 | 0.43 | 0.71 |
| Water | 0.09 | 0.91 | 0.09 |
| Obstacle | 0.04 | 0.05 | 0.55 |

Pixel accuracy is 0.81 and mean IoU is 0.33.

How to read this:

- Pixel accuracy is high mainly because vegetation covers about 90% of scored
  pixels. Mean IoU is the fairer summary.
- Water is mostly missed: 64% of puddle pixels are called ground. The Tartan drive also
  showed false water on damp soil, so camera water should not be relied on.
- Obstacle precision is very low because bush is often called obstacle (22% of bush
  pixels), and real obstacles are rare in these scenes.
- Tree and sky are not scored, because RELLIS does not separate trunk from crown.
- Mud is often called vegetation (53%).
- The model was not trained on RELLIS-3D, so this measures how well it transfers, and
  its classes are grouped. It is not a benchmark of the model itself.

Report: `artifacts/rellis_test_split_camera_2026_11_04/report.json`.

## LiDAR-only classes on voxel maps

```bash
/usr/bin/python3 src/semantic_mapping/tools/evaluate_rellis_lidar_map.py --output artifacts/my_rellis_lidar
```

Fifteen stretches (3 per sequence, 30 scans each, every third scan) were built with
the v2 settings: 0.2 m voxels, 20 m range and cloth ground. Each voxel takes the
majority human label of its points.

| Measure | Result |
|---|---:|
| Scored voxels | 1,247,999 |
| Voxels with an accepted class | 55% |
| Mean over the 14 labelled classes | 44% |
| Voxels given a class | 71% |
| Accepted class, among classified voxels | 77% |

| Human label | Voxels | Accepted class | Left unknown |
|---|---:|---:|---:|
| Bush | 420,561 | 47% | 29% |
| Tree | 371,929 | 35% | 45% |
| Grass | 349,710 | 86% | 14% |
| Concrete | 69,414 | 67% | 12% |
| Person | 22,605 | 27% | 34% |

How to read this:

- Grass is scored leniently, because ground and low vegetation both count.
- Many tree and bush voxels get no class, usually because too few points or rays
  reached them. This is the main weakness for forest use.
- Classes with few voxels, such as object and vehicle, are not reliable.
- This measures agreement with labels, not traversability.

Report: `artifacts/rellis_lidar_voxels_2026_11_04/report.json`.

## Published calibration

`tools/check_label_alignment_rellis.py` projects human-labelled LiDAR points onto
human-labelled images. Over 250 frames the labels match on 82% of points, and the
best match is at 0.0° across and −0.5° vertical. The published transform must be
inverted: as given, the labels match on only 28%.

Report: `artifacts/calibration_outline_2026_11_04/rellis_label_reference/report.json`.

## Filling sparse voxels from their neighbours (9 November)

A diagnosis on sequences 00000 and 00001 found that tree voxels are left unknown
mainly because they get fewer than 3 points (39% of tree voxels), not because the
rules are wrong. Three times more scans only cut unknown tree voxels from 48% to 40%.

The rule tested, `fill_sparse` in `lidar_features.py`: an unknown voxel with fewer
than 3 points, or fewer than 5 hits plus passes, takes the most common known class of
its 26 neighbours, if at least 3 of them have one. The rule and the decision were
written down before testing (`artifacts/rellis_tree_coverage_2026_11_06/PLAN.md`).
It passed on the development sequences and was then scored once on 00002 to 00004.

| Held-out sequences 00002–00004 | As measured | With fill |
|---|---:|---:|
| Voxels with an accepted class | 60.8% | 76.4% |
| Mean over labelled classes | 45.1% | 55.3% |
| Tree, accepted / unknown | 39.9% / 40.1% | 59.5% / 9.0% |
| Bush, accepted / unknown | 47.9% / 30.9% | 66.5% / 5.2% |
| Grass, accepted | 85.4% | 97.5% |
| Concrete, accepted | 59.9% | 68.4% |

Filled voxels on their own were accepted 75% of the time, close to the 77% of measured
voxels that get a class. Development sequences 00000 and 00001 showed the same pattern:
overall 48.0% to 64.6%, tree 32.4% to 53.7%.

How to read this:

- Filled classes are inferred from neighbours, not measured, so the map should keep
  them marked as inferred.
- Grass is scored leniently, because ground and low vegetation both count.
- The fill is an option. The TartanDrive maps and demos were built without it.

```bash
/usr/bin/python3 src/semantic_mapping/tools/evaluate_rellis_lidar_map.py --sequences 00002 00003 00004 --fill-sparse --output artifacts/my_fill_check
```

Reports: `artifacts/rellis_tree_coverage_2026_11_06/`.

