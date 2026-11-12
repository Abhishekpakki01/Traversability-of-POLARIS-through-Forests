# Set up a terminal for this project in one line (source it, do not run it):
#   source ~/Traversability-of-POLARIS-through-Forests/src/semantic_mapping/tools/ros_env.sh
# It moves to the repo, loads ROS 2 Humble, the original workspace and this workspace,
# puts the system Python (which has torch) first, and keeps ROS traffic on this PC.
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$repo"
source /opt/ros/humble/setup.bash
source "$HOME/semantic_map_ws/install/setup.bash"
source "$repo/install/setup.bash"
export PATH="/usr/bin:$PATH"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-112}
export ROS_LOCALHOST_ONLY=1
echo "Ready: $repo (ROS domain $ROS_DOMAIN_ID)"
