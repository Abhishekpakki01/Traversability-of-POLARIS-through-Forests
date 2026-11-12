#!/usr/bin/env bash
# Run the whole v2 pipeline LIVE from the raw TartanDrive recording, on its first N scans,
# then show the result in RViz with the camera beside the map.
# Same settings as build_v2_demo.sh (0.2 m voxels, 20 m range, vehicle box, cloth ground).
# The terminal shows each command as it runs and a short summary of what each step produced;
# the full output of every step is kept in <output>/logs/.
#
#   src/semantic_mapping/tools/quick_run.sh            30 scans, about 3 minutes on the thesis laptop
#   src/semantic_mapping/tools/quick_run.sh 10         about 2 minutes
#   src/semantic_mapping/tools/quick_run.sh 10 --no-view   build only, no RViz
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

# Print the key numbers of one step from the JSON report at the end of its log.
summary() {
  $py - "$1" "$2" <<'EOF'
import json, sys
step, log = sys.argv[1], sys.argv[2]
text = open(log).read()
d = json.loads(text[text.index('\n{') + 1:])
pct = lambda x: f'{100 * x:.1f}%'
def show(label, value):
    print(f'    {label:<38} {value}')
if step == 'map':
    f = d['last_frame']
    show('scans mapped', f"{d['mapped_scans']} of {d['requested_scan_limit']} requested ({d['source_scans']} in the recording)")
    show('camera images matched / labelled', f"{d['matched_images']} / {d['inferred_images']}")
    show('scans skipped for a missing pose', d['missing_lidar_pose'])
    show('vehicle-body points removed', f"{d['vehicle_box_points_removed']:,}")
    show('occupied 0.2 m voxels', f"{f['occupied_voxels']:,}")
    show('voxels with a confirmed camera class', f"{f['classified_voxels']:,}")
    show('camera classes', ', '.join(f'{k} {v:,}' for k, v in f['class_counts'].items() if v and k != 'unknown'))
    show('median time per scan', f"{d['processing_seconds_median']:.1f} s")
elif step == 'terrain':
    f = d['final']
    show('ground cells', f"{f['ground_cells']:,}")
    show('voxels above the ground', f"{f['above_ground_voxels']:,}")
    show('drivable / caution / blocked', f"{pct(f['traversable'])} / {pct(f['caution'])} / {pct(f['non_traversable'])}")
    show('vehicle limits', 'PLACEHOLDERS, not POLARIS figures')
elif step == 'lidar':
    show('voxels above the ground', f"{d['final_above_ground_voxels']:,}")
    show('LiDAR-only classes', ', '.join(f'{k} {v:,}' for k, v in d['final_class_counts'].items() if v))
elif step == 'water':
    show('camera rays cast / hitting the ground', f"{d['rays_cast']:,} / {d['rays_hit_ground']:,}")
    show('water area', f"{d['final_water_area_m2']:.1f} m² (camera-based, unvalidated)")
EOF
}

# step <name> <title> <command...>: show the command, run it, then its time and summary.
step() {
  local name=$1 title=$2; shift 2
  local t0=$SECONDS
  echo
  echo "== $title"
  echo "   \$ $*" | sed "s|$HOME|~|g"
  "$@" > "$out/logs/$name.log" 2>&1 || { echo "   FAILED: see $out/logs/$name.log"; exit 1; }
  echo "   done in $((SECONDS - t0)) s"
  summary "$name" "$out/logs/$name.log" || echo "   (summary unavailable: see $out/logs/$name.log)"
}

echo "Running the pipeline on the first $scans scans of the raw TartanDrive recording"
echo "Output: $out" | sed "s|$HOME|~|g"

step map "1/4  LiDAR map + camera classes (a few seconds per scan)" \
  $py $tools/build_offline_demo.py --max-scans "$scans" --exclude-vehicle-box -1.8 -1.6 -1.0 0.4 1.4 0.4 \
  --resolution 0.2 --max-range 20 --inference-short-side 512 \
  --image-mask $config/tartan_image_mask.json --threads 6 --output "$out/run"
step terrain "2/4  Terrain and traversability" \
  $py $tools/build_terrain_replay.py --input "$out/run" --config $config/traversability_v2.yaml --output "$out/terrain"
step lidar "3/4  LiDAR-only classes" \
  $py $tools/build_lidar_feature_replay.py --run "$out/run" --replay "$out/terrain/replay" \
  --terrain-config $config/traversability_v2.yaml --ray-step 0.1 --output "$out/lidar"
step water "4/4  Water layer" \
  $py $tools/build_water_replay.py --run "$out/run" --replay "$out/lidar/replay" \
  --terrain-config $config/traversability_v2.yaml --fill-iterations 14 --output "$out/water"

echo
echo "All steps done in $((SECONDS - start)) s. Full output of each step: $out/logs/" | sed "s|$HOME|~|g"

[ "$view" = "--no-view" ] && exit 0
ros2 launch semantic_mapping demo_v2.launch.py play:=false layout:=demo_v2.rviz > "$out/logs/view.log" 2>&1 &
viewer=$!
trap 'kill -INT $viewer 2>/dev/null; wait $viewer 2>/dev/null; echo "Closed."' EXIT
echo "RViz is starting. Playback begins in 5 s at a quarter of real speed."
sleep 5
ros2 bag play "$out/water/replay" --clock --rate 0.25 --read-ahead-queue-size 100 || true
echo "Playback finished; RViz stays open. Press Ctrl+C here to close it."
wait $viewer
