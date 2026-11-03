#!/usr/bin/env bash
# Open the v2 demo so YOU control it from this terminal.
#   tools/view_demo.sh                 follow the vehicle (camera panels, 15 m LiDAR view)
#   tools/view_demo.sh explore         whole map, free camera, large Displays list
#   tools/view_demo.sh light           for software (CPU) rendering
# Keys in this terminal while it plays: SPACE pause/resume, RIGHT ARROW one step,
# UP/DOWN faster/slower. After playback RViz stays open; press Ctrl+C here to close everything.
set -eo pipefail
case "${1:-follow}" in
  follow) layout=demo_v2.rviz ;; explore) layout=demo_v2_explore.rviz ;; light) layout=demo_v2_light.rviz ;;
  *) echo "usage: $0 [follow|explore|light] [rate]"; exit 1 ;;
esac
rate=${2:-0.5}
repo="$(cd "$(dirname "$0")/../../.." && pwd)"
bag="$repo/artifacts/water_demo_v2_2026_11_03/replay"
source /opt/ros/humble/setup.bash
source "$HOME/semantic_map_ws/install/setup.bash"
source "$repo/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-112} ROS_LOCALHOST_ONLY=1
ros2 launch semantic_mapping demo_v2.launch.py play:=false layout:=$layout > /tmp/view_demo_$$.log 2>&1 &
viewer=$!
trap 'kill -INT $viewer 2>/dev/null; wait $viewer 2>/dev/null; echo "Closed."' EXIT
echo "RViz is starting ($layout). Playback begins in 5 s at rate $rate."
sleep 5
ros2 bag play "$bag" --clock --rate "$rate" --read-ahead-queue-size 100 || true
echo
echo "Playback finished; RViz stays open with the final map. Rotate: left-drag, pan: middle-drag, zoom: wheel."
echo "Press Ctrl+C here to close RViz."
wait $viewer
