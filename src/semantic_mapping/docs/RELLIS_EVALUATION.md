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
| Obstacle | 0.05 | 0.05 | 0.55 |

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
