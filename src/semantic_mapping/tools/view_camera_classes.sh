#!/usr/bin/env bash
# Show camera classes merged into the LiDAR point cloud (TartanDrive), with the camera beside the map.
#   tools/view_camera_classes.sh            the saved full drive
#   tools/view_camera_classes.sh RUN_FOLDER a run from build_offline_demo.py or quick_run.sh
# Layers (tick them in the Displays list): camera classes on this scan's LiDAR points (default),
# camera colours on the points, both accumulated over the drive, and the fused camera-class voxel map.
# Keys in this terminal while it plays: SPACE pause/resume, RIGHT ARROW one step. Ctrl+C closes RViz.
set -eo pipefail
repo="$(cd "$(dirname "$0")/../../.." && pwd)"
run="${1:-$repo/artifacts/offline_camera_points_2026_11_16}"
rate=${2:-0.5}
[ -d "$run/replay" ] || { echo "No replay in $run (build it with build_offline_demo.py or quick_run.sh)"; exit 1; }
source /opt/ros/humble/setup.bash
source "$HOME/semantic_map_ws/install/setup.bash"
source "$repo/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-112} ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping demo_v2.launch.py play:=false layout:=camera_classes.rviz > /tmp/view_camera_classes_$$.log 2>&1 &
viewer=$!
trap 'kill -INT $viewer 2>/dev/null; wait $viewer 2>/dev/null; echo "Closed."' EXIT
echo "RViz is starting (camera classes). Playback begins in 5 s at rate $rate."
sleep 5
ros2 bag play "$run/replay" --clock --rate "$rate" --read-ahead-queue-size 100 || true
echo "Playback finished; RViz stays open. Press Ctrl+C here to close it."
wait $viewer
