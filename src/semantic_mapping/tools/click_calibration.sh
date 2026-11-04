#!/usr/bin/env bash
# Open the camera-LiDAR click check. Clicks save as you go; run it again to continue.
set -eo pipefail
repo="$(cd "$(dirname "$0")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$HOME/semantic_map_ws/install/setup.bash"
source "$repo/install/setup.bash"
exec /usr/bin/python3 "$repo/src/semantic_mapping/tools/click_calibration_check.py" \
  --input "${1:-$repo/artifacts/calibration_clicks_2026_11_04}"
