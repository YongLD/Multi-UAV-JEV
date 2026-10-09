#!/usr/bin/env bash
# Uses the matching native runtime from the downloaded NeoHorse model release.
set -euo pipefail
cd "$(dirname "$0")"
: "${MODEL_DIR:?Set MODEL_DIR to your complete NeoHorse-Jev-4B model bundle}"
MODEL_BIN="${MODEL_BIN:-.venv-model/bin/neohorse-decision}"
if [[ ! -x "$MODEL_BIN" ]]; then
  echo "Missing $MODEL_BIN. See docs/model-setup.md for installation." >&2
  exit 1
fi
exec "$MODEL_BIN" serve --model-dir "$MODEL_DIR" \
  --device "${MODEL_DEVICE:-cuda}" --host 127.0.0.1 --port "${MODEL_PORT:-8000}"
