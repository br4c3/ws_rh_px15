#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
QCS_ROOT="$PROJECT_ROOT/qcs"

if [[ ! -d "$QCS_ROOT/node_modules" ]]; then
  echo "QCS dependencies are missing. Run: cd qcs && npm install" >&2
  exit 1
fi

cd "$QCS_ROOT"
export ELECTRON_OZONE_PLATFORM_HINT="${ELECTRON_OZONE_PLATFORM_HINT:-x11}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QSG_RHI_BACKEND="${QSG_RHI_BACKEND:-opengl}"
exec npm start
