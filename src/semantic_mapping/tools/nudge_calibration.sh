#!/usr/bin/env bash
# Open the outline nudge check. Finished frames save; run it again to continue.
set -eo pipefail
repo="$(cd "$(dirname "$0")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$HOME/semantic_map_ws/install/setup.bash"
source "$repo/install/setup.bash"
exec /usr/bin/python3 "$repo/src/semantic_mapping/tools/nudge_calibration_check.py" \
  --input "${1:-$repo/artifacts/calibration_outline_2026_11_04/nudge_frames}"
