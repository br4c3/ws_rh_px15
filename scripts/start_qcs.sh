#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
QCS_ROOT="$PROJECT_ROOT/qcs"
DEFAULT_JETSON_GCS_URL="http://192.168.144.26:8765"

if [[ ! -d "$QCS_ROOT/node_modules" ]]; then
  echo "QCS dependencies are missing. Run: cd qcs && npm install" >&2
  exit 1
fi

if ! command -v node >/dev/null 2>&1 || ! command -v npm >/dev/null 2>&1; then
  NVM_ROOT="${NVM_DIR:-${HOME}/.nvm}"
  if [[ -s "$NVM_ROOT/nvm.sh" ]]; then
    # nvm.sh reads optional variables that may be unset.
    set +u
    # shellcheck source=/dev/null
    source "$NVM_ROOT/nvm.sh"
    set -u
  fi
fi

if ! command -v node >/dev/null 2>&1 || ! command -v npm >/dev/null 2>&1; then
  echo "Node.js/npm not found." >&2
  echo "Install Node.js, or load nvm with: source ~/.nvm/nvm.sh" >&2
  exit 1
fi

if [[ -z "${QGC_PATH:-}" ]]; then
  qgc_candidates=(
    "${HOME}/Downloads/QGroundControl-v5.0.8-x86_64.AppImage"
    "${HOME}/Downloads/QGroundControl.AppImage"
    "${HOME}/apps/QGroundControl.AppImage"
  )

  for qgc_candidate in "${qgc_candidates[@]}"; do
    if [[ -x "$qgc_candidate" ]]; then
      export QGC_PATH="$qgc_candidate"
      break
    fi
  done
fi

export JETSON_GCS_URL="${JETSON_GCS_URL:-$DEFAULT_JETSON_GCS_URL}"

echo "Starting QCS"
echo "  Node: $(node --version)"
echo "  Jetson gateway: $JETSON_GCS_URL"

if command -v curl >/dev/null 2>&1; then
  gateway_health="$(
    curl \
      --silent \
      --show-error \
      --fail \
      --connect-timeout 2 \
      --max-time 3 \
      "$JETSON_GCS_URL/health" 2>/dev/null \
      || true
  )"
  if [[ -z "$gateway_health" ]]; then
    echo "WARNING: Jetson gateway is not reachable at $JETSON_GCS_URL" >&2
    echo "         Start scripts/start_jetson.sh on the Jetson and check its IP." >&2
  elif [[ "$gateway_health" == *'"px4Connected":true'* ]]; then
    echo "  PX4 DDS: connected"
  else
    echo "WARNING: Jetson gateway is reachable, but PX4 DDS is not connected." >&2
    echo "         Gateway response: $gateway_health" >&2
  fi
fi

cd "$QCS_ROOT"
export ELECTRON_OZONE_PLATFORM_HINT="${ELECTRON_OZONE_PLATFORM_HINT:-auto}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QSG_RHI_BACKEND="${QSG_RHI_BACKEND:-opengl}"
exec npm start
