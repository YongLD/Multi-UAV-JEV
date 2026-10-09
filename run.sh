#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ -f .env ]]; then
  set -a
  source .env
  set +a
fi
if [[ ! -x .venv/bin/python ]]; then
  echo "Run bash setup.sh first." >&2
  exit 1
fi
exec .venv/bin/python scripts/run.py "$@"
