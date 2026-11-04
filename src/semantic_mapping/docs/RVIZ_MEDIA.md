# Pictures and videos from RViz — 4 November 2026

`tools/capture_demo_media.py` makes pictures and videos from the real RViz view,
with the camera image of the same scan beside the map.

```bash
cd ~/Traversability-of-POLARIS-through-Forests
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=112
export ROS_LOCALHOST_ONLY=1
~/thesis_env/bin/pip install imageio-ffmpeg     # once: the H.264 video encoder
/usr/bin/python3 src/semantic_mapping/tools/capture_demo_media.py --output artifacts/my_media
```

This takes about 12 minutes, and RViz windows open and close by themselves. They
ignore the mouse and keyboard while recording, because a stray drag or scroll
moved the view in the first attempt. `--only 1 4` makes only those runs, numbered
as in `RUNS` in the script.

## How it works

1. The recording is stepped one scan at a time, so no scan is skipped and the
   videos play at the real speed of 10 scans per second.
2. RViz draws each scan from a generated layout: one map layer, no side panels.
   The saved layouts are not changed.
3. `tools/capture_rviz_window.ps1` saves the RViz window on the Windows side.
   Grabbing it from Linux gives a black image, because WSLg draws it through the
   GPU. Only the RViz window is saved, not the rest of the screen.
4. The camera image and predicted classes of the same scan are placed beside it.

## What is in the frames

- **Left:** the RViz view. The white line is the path driven so far, and the
  magenta arrow is the vehicle.
- **Right:** the camera image, the predicted classes, and the colour key.
- The traversability percentages are computed from the fused layer on screen.
  The key stored in the replay (`/terrain/status_image`) gives the camera-only
  layer's percentages, so it is not used here.

These are an offline replay of one TartanDrive 2.0 excerpt. Vehicle limits are
placeholders, and water is unvalidated: at scan 232 the model marks dark wet
ground as water.

`*.mp4` files are not stored in Git. The pictures are in
`artifacts/rviz_media_2026_11_04/`.
