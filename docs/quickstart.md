# Quick start

The simulation process and the model service are independent. A browser only needs access to the lightweight viewer; it does not run the 4B model.

## 1. Install the simulator

Recommended: Linux, Python 3.12, Git, and a working EGL/OpenGL driver. `setup.sh` creates a simulation-only virtual environment and fetches the pinned Skydio X2 subtree from MuJoCo Menagerie. City meshes are already bundled.

```bash
git clone https://github.com/YongLD/Multi-UAV-JEV.git
cd Multi-UAV-JEV
PYTHON_BIN=python3.12 bash setup.sh
```

`PYTHON_BIN` defaults to `python3`; Python 3.11 or later is required. The validated MuJoCo version is pinned in `requirements.txt`. `imageio-ffmpeg` supplies the video encoder used by the replay writer.

On macOS, use `MUJOCO_GL=glfw` and keep graphical rendering available. On Linux headless machines, use `MUJOCO_GL=egl`; software rendering can use `osmesa` if its system libraries are installed. GPU model serving has been validated on Linux, not on macOS.

## 2. Start or connect to Jev

Follow [model-setup.md](model-setup.md) to install the matching native NeoHorse runtime. In a separate terminal:

```bash
MODEL_DIR=/path/to/NeoHorse-Jev-4B CUDA_VISIBLE_DEVICES=0 bash run_model.sh
```

Check readiness:

```bash
curl -fsS http://127.0.0.1:8000/health
```

Or connect to an existing endpoint implementing `POST /v1/systemone` and `GET /health`:

```bash
export JEV_ENDPOINT=http://127.0.0.1:8000/v1/systemone
```

The runner fails early if the health endpoint is unavailable. A failing request is logged; the simulator does not invent a replacement tactical decision. Until a fresh valid action arrives, the existing controller setpoints remain active.

## 3. Run and view

```bash
bash run.sh
```

Open `http://127.0.0.1:8080`. The viewer binds to loopback. Mouse drag changes the third-person viewing offset; the camera keeps following the pursuer's heading. The wheel changes zoom; double click restores alignment. The small overview stays north-up. These viewing controls never alter model observations.

The runner owns the viewer and simulation processes. `Ctrl+C` stops both. `--once` ends after one episode and also stops the viewer. Use a separate viewer process to inspect saved frames afterward:

```bash
DUEL_LIVE_DIR="$PWD/runs/live" DUEL_VIEW_PORT=8080 .venv/bin/python sim/urban_view.py
```

## Remote server, local browser

Run setup, model serving and `run.sh` on your server. Forward only the viewer port from your laptop:

```bash
ssh -N -L 8080:127.0.0.1:8080 your-user@your-server
```

Open `http://127.0.0.1:8080` locally. If this local port is occupied, forward another one, for example `-L 18086:127.0.0.1:8080` and open the matching local address. SSH connection reuse can be configured in your own SSH config; no machine-specific credentials are needed in this repository.

## Configuration

`run.sh` loads `.env` if it exists. Copy `.env.example` and edit its values. CLI options take precedence over the corresponding environment defaults.

| Variable | Default | Purpose |
| --- | --- | --- |
| `JEV_ENDPOINT` | `http://127.0.0.1:8000/v1/systemone` | Decision endpoint. |
| `JEV_MODEL` | `NeoHorse-Jev-4B` | Model name in requests. |
| `JEV_API_KEY` | empty | Optional Bearer token; never written to episode logs. |
| `VIEW_PORT` | `8080` | Local viewer port. |
| `DATA_DIR` | `./runs` | Live artifacts and archived cases. |
| `EPISODE_SECONDS` | `120` | Maximum simulated duration per episode. |
| `START_SEED` | `1200` | Initial seed; incremented after each episode. |
| `DUEL_POLICY_MODE` | `conditions` | Candidate descriptions; see [policy.md](policy.md). |
| `MUJOCO_GL` | Linux: `egl`; otherwise: `glfw` | MuJoCo rendering backend. |

Use a distinct data directory and viewer port for another instance. A lock prevents two runners from writing the same directory. The generated scene XML is shared inside one checkout; use separate checkouts for concurrent experiments.

## Troubleshooting

- **No EGL context:** inspect the machine's EGL/OpenGL driver or select an installed alternative rendering backend.
- **Missing `x2.xml` or meshes:** rerun `setup.sh`; `sim/assets` must point to `mujoco_menagerie/skydio_x2/assets`.
- **401 from the model:** set `JEV_API_KEY` to the server's configured token.
- **429/529:** the inference worker is busy. Use one simulation per native model service for a meaningful performance run.
- **422:** check the model name, runtime version and payload/token budget. The full structured state can be large; this project does not silently truncate it.
- **No replay:** only completed episodes with ≥5 hits and ≥the previous best qualify. Short verification runs often produce no replay.
- **Viewer port already used:** change `VIEW_PORT`; the runner rejects a conflict instead of attaching to unrelated data.
- **Tracking loss / contact:** inspect the model state, latency and action log. The baseline does not guarantee successful pursuit or collision avoidance.
