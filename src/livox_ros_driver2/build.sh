#!/usr/bin/env bash
# Build the bundled ROS 2 driver inside the unified workspace.
set -euo pipefail

case "${1:-humble}" in
  humble|ROS2)
    if [ "$#" -gt 0 ]; then
      shift
    fi
    ;;
  *)
    echo "本工作区使用 ROS 2 Humble；请使用 ./build.sh humble 或根目录 colcon build。" >&2
    exit 2
    ;;
esac

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
# ROS setup scripts do not consistently support nounset.
set +u
source /opt/ros/humble/setup.bash
set -u
cd -- "$workspace_root"
exec colcon build --packages-up-to livox_ros_driver2 "$@"
