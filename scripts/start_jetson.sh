#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"
WORKSPACE_SETUP="$PROJECT_ROOT/install/setup.bash"
PX4_DDS_AGENT="${PX4_DDS_AGENT:-MicroXRCEAgent}"
PX4_DDS_TRANSPORT="${PX4_DDS_TRANSPORT:-udp4}"
PX4_DDS_PORT="${PX4_DDS_PORT:-8888}"
PX4_DDS_DEVICE="${PX4_DDS_DEVICE:-/dev/ttyTHS1}"
PX4_DDS_BAUDRATE="${PX4_DDS_BAUDRATE:-921600}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi

if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  echo "Workspace is not built. Run: colcon build --symlink-install" >&2
  exit 1
fi

# ROS/colcon setup scripts inspect optional variables such as
# AMENT_TRACE_SETUP_FILES. Temporarily disable nounset while sourcing them.
set +u
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
set -u

if ! command -v "$PX4_DDS_AGENT" >/dev/null 2>&1; then
  echo "PX4 DDS Agent not found: $PX4_DDS_AGENT" >&2
  echo "Install Micro XRCE-DDS Agent or set PX4_DDS_AGENT to its path." >&2
  exit 1
fi

case "$PX4_DDS_TRANSPORT" in
  udp4)
    agent_args=(udp4 -p "$PX4_DDS_PORT")
    ;;
  serial)
    if [[ ! -e "$PX4_DDS_DEVICE" ]]; then
      echo "PX4 DDS serial device not found: $PX4_DDS_DEVICE" >&2
      exit 1
    fi
    agent_args=(
      serial
      --dev "$PX4_DDS_DEVICE"
      -b "$PX4_DDS_BAUDRATE"
    )
    ;;
  *)
    echo "Unsupported PX4_DDS_TRANSPORT: $PX4_DDS_TRANSPORT (use udp4 or serial)" >&2
    exit 1
    ;;
esac

agent_pid=""
gateway_pid=""

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$gateway_pid" ]]; then
    kill "$gateway_pid" 2>/dev/null || true
  fi
  if [[ -n "$agent_pid" ]]; then
    kill "$agent_pid" 2>/dev/null || true
  fi
  wait "$gateway_pid" "$agent_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Starting PX4 DDS Agent: $PX4_DDS_AGENT ${agent_args[*]}"
"$PX4_DDS_AGENT" "${agent_args[@]}" &
agent_pid=$!

cd "$PROJECT_ROOT/qcs"
python3 bridge/jetson_gateway.py &
gateway_pid=$!

echo "Jetson gateway listening on ${JETSON_GCS_HOST:-0.0.0.0}:${JETSON_GCS_PORT:-8765}"
wait -n "$agent_pid" "$gateway_pid"

if ! kill -0 "$agent_pid" 2>/dev/null; then
  echo "PX4 DDS Agent stopped unexpectedly." >&2
  exit 1
fi
if ! kill -0 "$gateway_pid" 2>/dev/null; then
  echo "Jetson gateway stopped unexpectedly." >&2
  exit 1
fi
