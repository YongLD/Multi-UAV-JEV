#!/usr/bin/env bash
# Install simulation dependencies and fetch only the Skydio X2 asset subtree.
set -euo pipefail
cd "$(dirname "$0")"
PYTHON_BIN="${PYTHON_BIN:-python3}"
"$PYTHON_BIN" -c 'import sys; assert sys.version_info >= (3, 11), "Python 3.11+ required; Python 3.12 recommended"'
"$PYTHON_BIN" -m venv .venv
.venv/bin/python -m pip install -r requirements.txt

MENAGERIE_REF=4d038b3feae26ec82b46a4d586379114012a8ac7
if [[ ! -f sim/mujoco_menagerie/skydio_x2/x2.xml ]]; then
  git init sim/mujoco_menagerie
  if ! git -C sim/mujoco_menagerie remote get-url origin >/dev/null 2>&1; then
    git -C sim/mujoco_menagerie remote add origin https://github.com/google-deepmind/mujoco_menagerie.git
  fi
  git -C sim/mujoco_menagerie sparse-checkout init --cone
  git -C sim/mujoco_menagerie sparse-checkout set skydio_x2
  git -C sim/mujoco_menagerie fetch --depth 1 --filter=blob:none origin "$MENAGERIE_REF"
  git -C sim/mujoco_menagerie checkout --detach FETCH_HEAD
fi
ln -sfn mujoco_menagerie/skydio_x2/assets sim/assets
echo "Simulation installed. Start a Jev endpoint, then: bash run.sh"
