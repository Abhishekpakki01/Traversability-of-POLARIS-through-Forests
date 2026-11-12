#!/usr/bin/env bash
# Run the whole v2 pipeline LIVE from the raw TartanDrive recording, on its first N scans,
# then show the result in RViz with the camera beside the map.
# Same settings as build_v2_demo.sh (0.2 m voxels, 20 m range, vehicle box, cloth ground).
#
#   src/semantic_mapping/tools/quick_run.sh            30 scans, about 3 minutes on the thesis laptop
#   src/semantic_mapping/tools/quick_run.sh 60         more scans, about twice as long
#   src/semantic_mapping/tools/quick_run.sh 30 --no-view   build only, no RViz
#
# Output goes to ~/thesis_runs/quick_run_<date_time> (outside the Git repo; about 1 GB per 30 scans).
# Keys while it plays: SPACE pause/resume, RIGHT ARROW one step. Ctrl+C closes RViz.
set -eo pipefail
scans=${1:-30}
view=${2:-}
repo="$(cd "$(dirname "$0")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$HOME/semantic_map_ws/install/setup.bash"
source "$repo/install/setup.bash"
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-112} ROS_LOCALHOST_ONLY=1
cd "$repo"
tools=src/semantic_mapping/tools
config=src/semantic_mapping/config
py=/usr/bin/python3
out="$HOME/thesis_runs/quick_run_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$out/logs"
start=$SECONDS

echo "1/4 LiDAR map + camera classes from the raw recording ($scans scans) ..."
$py $tools/build_offline_demo.py --max-scans "$scans" --exclude-vehicle-box -1.8 -1.6 -1.0 0.4 1.4 0.4 \
  --resolution 0.2 --max-range 20 --inference-short-side 512 \
  --image-mask $config/tartan_image_mask.json --threads 6 --output "$out/run" > "$out/logs/map.log" 2>&1
echo "2/4 terrain and traversability ..."
$py $tools/build_terrain_replay.py --input "$out/run" --config $config/traversability_v2.yaml \
  --output "$out/terrain" > "$out/logs/terrain.log" 2>&1
echo "3/4 LiDAR-only classes ..."
$py $tools/build_lidar_feature_replay.py --run "$out/run" --replay "$out/terrain/replay" \
  --terrain-config $config/traversability_v2.yaml --ray-step 0.1 --output "$out/lidar" > "$out/logs/lidar.log" 2>&1
echo "4/4 water layer ..."
$py $tools/build_water_replay.py --run "$out/run" --replay "$out/lidar/replay" \
  --terrain-config $config/traversability_v2.yaml --fill-iterations 14 --output "$out/water" > "$out/logs/water.log" 2>&1
echo "Done in $((SECONDS-start)) s: $out"

[ "$view" = "--no-view" ] && exit 0
ros2 launch semantic_mapping demo_v2.launch.py play:=false layout:=demo_v2.rviz > "$out/logs/view.log" 2>&1 &
viewer=$!
trap 'kill -INT $viewer 2>/dev/null; wait $viewer 2>/dev/null; echo "Closed."' EXIT
echo "RViz is starting. Playback begins in 5 s at a quarter of real speed."
sleep 5
ros2 bag play "$out/water/replay" --clock --rate 0.25 --read-ahead-queue-size 100 || true
echo "Playback finished; RViz stays open. Press Ctrl+C here to close it."
wait $viewer
