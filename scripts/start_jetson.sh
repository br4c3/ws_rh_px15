#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"
WORKSPACE_SETUP="$PROJECT_ROOT/install/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi

if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  echo "Workspace is not built. Run: colcon build --symlink-install" >&2
  exit 1
fi

source "$ROS_SETUP"
source "$WORKSPACE_SETUP"

cd "$PROJECT_ROOT/qcs"
exec python3 bridge/jetson_gateway.py
