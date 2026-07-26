#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
QCS_ROOT="$PROJECT_ROOT/qcs"

if [[ ! -d "$QCS_ROOT/node_modules" ]]; then
  echo "QCS dependencies are missing. Run: cd qcs && npm install" >&2
  exit 1
fi

cd "$QCS_ROOT"
exec npm start
