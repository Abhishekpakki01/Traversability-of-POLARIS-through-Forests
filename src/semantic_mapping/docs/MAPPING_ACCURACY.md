# How do we know the map is right? — 3 November 2026

The map's accuracy splits into four separate questions. Each needs its own
evidence:

| Question | Check | Status |
|---|---|---|
| 1. Are the poses right (where each scan was taken)? | Compare with independent GPS/INS | **Done: 6–18 cm** |
| 2. Do the scans fit together (no smearing)? | Thickness of flat ground in the accumulated map | **Done: 2.8 cm vs 1.9 cm single scan** |
| 3. Do camera pixels land on the right LiDAR points? | Clicked point pairs → reprojection error | Open (needs about 15 min of clicking) |
| 4. Are the class labels right? | Human-labelled data | Partial: one RELLIS frame and one scan; more needs a download |

## 1. Poses versus GPS/INS

`tools/check_trajectory_gps.py --run <offline run> --output <new folder>`

TartanDrive's NovAtel GPS/INS (`~/thesis_data/tartan_drive_2_sample/gps_odom.npy`)
covers the excerpt and is independent of the LiDAR odometry the map uses. After
a rigid best fit (rotation and translation, no scale), over the 318 poses and
31.8 s:

| Measure | Result |
|---|---:|
| Position difference RMSE, rigid fit | 0.18 m (max 0.35 m) |
| Position difference RMSE, also fitting the GPS antenna offset | 0.06 m (max 0.13 m) |
| Estimated antenna offset in the vehicle frame | (−0.11, −0.28, 0.26) m |
| Path length, LiDAR odometry / GPS | 102.0 m / 101.0 m |
| Height change, LiDAR odometry / GPS | −11.6 m / −12.1 m |
| Distance error per 10 m segment, median / 95% | 0.4% / 1.0% |
| **Tilt of map Z against gravity** | **0.35°** |

How to read this:

- The GPS reference point is not at the vehicle origin, so fitting an offset is
  justified. Those 3 extra parameters make 0.06 m the optimistic figure; 0.18 m
  is the conservative one. Both are at or below the 0.2 m voxel size.
- The tilt result confirms an assumption that was previously unverified: map Z
  is gravity-up to within 0.35°, so heights and slopes are not biased by a
  tilted frame.
- GPS has its own errors (RTK status is not in this sample). This is
  agreement between two independent systems, not ground truth.

Figure: `artifacts/trajectory_vs_gps_2026_11_03/trajectory_vs_gps.png`.

## 2. Map sharpness

`tools/check_map_sharpness.py --run <offline run> --output <new folder>`

If poses, the LiDAR mounting or timing were wrong, the same ground seen from
many places would smear into a thick layer. Thickness here is the spread across
the surface of 0.4 m flat, horizontal patches:

| | Flat patches | Median thickness | 90% |
|---|---:|---:|---:|
| Single scans (sensor noise floor) | 109 | 1.9 cm | 3.7 cm |
| 159 scans, recorded poses | 7,230 | **2.8 cm** | 4.1 cm |
| 159 scans, poses perturbed by σ = 0.5° and 5 cm | 2,776 | 4.6 cm | 6.5 cm |

The accumulated map is within about 1 cm of single-scan noise. A pose error as
small as 0.5° or 5 cm visibly thickens the ground and destroys 60% of the flat
patches, so the check is sensitive. The scans therefore register consistently.
Evidence: `artifacts/map_sharpness_2026_11_03/report.json`.

## Checks from earlier stages (all still valid)

- Vehicle height above mapped ground (v2): 1.81–2.01 m (5–95%). Tilt against
  the mapped ground: median 2.7°.
- Driven path: 1.6% of poses non-traversable with camera + LiDAR vegetation.
  These are positive examples only.
- Saved-data audit: all 318 frames are consistent (`offline_v2_*/audit.json`).
- One labelled RELLIS camera frame: pixel accuracy 0.68, mean IoU 0.36
  (`ACCURACY_V2.md`).
- One labelled RELLIS LiDAR scan: ground surfaces 97–99%, bush 64%, tree about
  5% (`LIDAR_FEATURES.md`).

## What is still needed, and how

**Camera–LiDAR calibration (question 3).** The automatic edge test was
inconclusive on this meadow (`ACCURACY_V2.md`). A reliable method:

1. In about 5 frames, click about 10 points where a LiDAR feature is clear in
   the image, such as trunk edges, posts or sharp ground edges.
2. Report the pixel error between the projected LiDAR point and the clicked
   pixel. Fewer than about 5 px at 10 m corresponds to roughly 10 cm.

**Class accuracy (question 4).** RELLIS-3D publishes thousands of labelled
camera frames and LiDAR scans, together with poses. Running this pipeline on a
labelled RELLIS sequence gives per-class precision, recall and IoU, both per
image and per map voxel. This is the evaluation the thesis needs. It requires
a download (several GB).

## Viewing the map in RViz

Start it from your own terminal so you stay in control:

```bash
cd ~/Traversability-of-POLARIS-through-Forests
src/semantic_mapping/tools/view_demo.sh explore      # whole map, free camera
src/semantic_mapping/tools/view_demo.sh              # follow the vehicle, with camera panels
src/semantic_mapping/tools/view_demo.sh light        # if RViz falls back to CPU rendering
```

- **While it plays:** SPACE pauses and resumes, RIGHT ARROW steps one message,
  UP/DOWN change speed. Type these in the terminal, not in RViz.
- **After playback:** RViz stays open with the final map. Press Ctrl+C in the
  terminal to close it.
- **Speed:** an optional second argument sets the rate, e.g.
  `view_demo.sh explore 0.25`.

- **Rotate:** with *Interact* selected, left-drag in the 3D view.
  Middle-drag (or Shift + left-drag) pans, and the scroll wheel zooms.
- **Why it felt stuck:** each update delivered a 9 MB cloud of 183k voxels, so
  RViz ran at about 5 fps and barely reacted to the mouse. `demo_v2` now shows
  LiDAR classes within 15 m of the vehicle, updated at 2 Hz (the newest map is always shown, including after playback ends). The full map is
  still available as "LiDAR-only classes, full map (slow)".
- **While playing,** the camera follows the vehicle (Target Frame `vehicle`).
  To look around freely, set Views → Target Frame to `sensor_init`.
- **Slow machine, or no GPU:** use the light layout, which draws flat squares
  instead of boxes:
  `ros2 launch semantic_mapping demo_v2.launch.py rate:=0.5 layout:=demo_v2_light.rviz`

### Is RViz using the GPU?

On 3 November, RViz on this PC was drawing on the CPU: the OpenGL renderer was
`llvmpipe`, `nvidia-smi` in Ubuntu failed, and the kernel log showed
`dxgkio_query_adapter_info: Ioctl failed`. WSL could not reach the Windows
GPU. Fix this on the Windows side:

1. In Windows PowerShell, run `nvidia-smi`. If it fails, the NVIDIA driver is
   not running: update or reinstall it, check Device Manager → Display
   adapters, and reboot.
2. Then run `wsl --update` and `wsl --shutdown` in PowerShell (this closes all
   Ubuntu terminals), and reopen Ubuntu.
3. Check in Ubuntu: `nvidia-smi` should list the GPU, and
   `glxinfo -B | grep renderer` (from `sudo apt install mesa-utils`) should
   say `D3D12 (NVIDIA ...)`, not `llvmpipe`.

On a laptop, also plug in the charger and make sure the NVIDIA GPU is not
disabled in power or graphics settings.
