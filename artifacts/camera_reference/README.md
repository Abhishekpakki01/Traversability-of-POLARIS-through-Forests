# Camera reference for the RELLIS voxel map

These PNGs come from `/pylon_camera_node/image_raw` in
`~/RELLIS-3D/example_filtered_ros2`. They were converted from `bayer_rggb8`
to color with `cv_bridge`, without contrast enhancement, segmentation, or
projection onto the voxel map.

| File | Original acquisition timestamp (ROS seconds) |
| --- | --- |
| `camera_reference_00s.png` | 1581791678.408547950 |
| `camera_reference_15s.png` | 1581791693.509166350 |
| `camera_1581791708_reference.png` | 1581791708.508545566 |

The last frame is 41.454434 ms earlier than the approximate ROS time
1581791708.55 visible in the shared voxel-map screenshot. The map accumulates
many previous scans; this image is a time reference, not a projection overlay.

## Live camera image in RViz

Keep the existing pose-aware mapper and RViz. In another terminal run:

```bash
source /opt/ros/humble/setup.bash
python3 ~/semantic_map_ws/src/semantic_mapping/scripts/camera_color_preview.py \
  --ros-args -p use_sim_time:=true
```

Start this script before replaying the bag. Stop previous playback and restart
the mapper for a fresh map, then replay with the camera included:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play ~/RELLIS-3D/example_filtered_ros2 --clock --delay 3 \
  --qos-profile-overrides-path ~/semantic_map_ws/qos_override.yaml \
  --topics /tf /tf_static /os1_cloud_node/points \
  /pylon_camera_node/image_raw /pylon_camera_node/camera_info
```

In RViz use **Add -> By topic -> /pylon_camera_node/image_color -> Image**.
This opens an image panel alongside the 3D map. A Camera display for calibrated
3D overlays is a later geometry-validation step. Keep Fixed Frame set to `odom`.
The previous LiDAR-only replay command does not publish camera images.

The preview preserves the original image timestamp and frame ID. It does not
yet pair images with LiDAR or check camera poses; full Task 6 synchronization
remains separate work. No build is required for the preview script. Stop its
terminal with Ctrl+C to return to the existing mapper-only workflow.

## Verification

The first recorded frame was checked for color conversion, header preservation,
1920x1200 resolution, and `bgr8` output. An isolated ROS playback check received
five live output frames and verified their resolution, encoding, camera frame,
and acquisition-time range. RViz's installed Image display supports displaying
the resulting standard color topic; the GUI view must still be checked locally.

A separate recording from a different scene is needed for the supervisor's
next environment check. The local ROS 1 bag and converted ROS 2 bag contain the
same example recording.
