# RELLIS-3D evaluation — 4 November 2026

RELLIS-3D has human labels for camera images and LiDAR points. Neither the GOOSE
model nor the LiDAR thresholds were fitted to it. The class grouping
(`config/rellis_label_groups.json`) and the accepted LiDAR classes
(`config/rellis_lidar_expectations.json`) were set before these tests.

The data is in `~/thesis_data/rellis_3d_full/` and is not in Git.

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
