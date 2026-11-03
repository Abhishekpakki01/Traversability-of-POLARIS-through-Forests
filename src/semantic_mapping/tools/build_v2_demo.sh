#!/usr/bin/env bash
# Build the whole v2 demo chain into new folders (none may exist yet):
#   offline map (0.2 m, 20 m, aspect-correct camera classes, hood mask)
#   -> audit -> terrain analysis -> terrain / LiDAR-feature / water replays.
# Usage (after the normal setup block): src/semantic_mapping/tools/build_v2_demo.sh SUFFIX
# About 30-40 minutes on the thesis PC; several GB of derived data.
set -euo pipefail
suffix=${1:?give a suffix, e.g. 2026_11_03}
cd "$(dirname "$0")/../../.."
tools=src/semantic_mapping/tools
config=src/semantic_mapping/config
run=artifacts/offline_v2_$suffix
py=/usr/bin/python3
[ -e "$run" ] || $py $tools/build_offline_demo.py --exclude-vehicle-box -1.8 -1.6 -1.0 0.4 1.4 0.4 \
  --resolution 0.2 --max-range 20 --inference-short-side 512 \
  --image-mask $config/tartan_image_mask.json --threads 6 --output "$run"
$py $tools/check_offline_demo.py --input "$run" > "$run/audit.json"
$py $tools/analyze_terrain.py --input "$run" --config $config/traversability_v2.yaml --output artifacts/terrain_analysis_v2_$suffix
$py $tools/build_terrain_replay.py --input "$run" --config $config/traversability_v2.yaml --output artifacts/terrain_demo_v2_$suffix
$py $tools/build_lidar_feature_replay.py --run "$run" --replay artifacts/terrain_demo_v2_$suffix/replay \
  --terrain-config $config/traversability_v2.yaml --ray-step 0.1 --output artifacts/lidar_features_demo_v2_$suffix
$py $tools/build_water_replay.py --run "$run" --replay artifacts/lidar_features_demo_v2_$suffix/replay \
  --terrain-config $config/traversability_v2.yaml --fill-iterations 14 --output artifacts/water_demo_v2_$suffix
$py $tools/plot_water_top_down.py --input artifacts/water_demo_v2_$suffix
echo "Done. View: ros2 launch semantic_mapping demo_v2.launch.py bag:=\"$PWD/artifacts/water_demo_v2_$suffix/replay\""
