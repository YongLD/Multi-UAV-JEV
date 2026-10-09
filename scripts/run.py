"""Own the viewer and episode processes; archive compact completed cases."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parent.parent


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--once", action="store_true", help="Run and archive one episode, then stop the viewer")
    parser.add_argument("--seconds", type=float, default=float(os.environ.get("EPISODE_SECONDS", 120)))
    parser.add_argument("--seed", type=int, default=int(os.environ.get("START_SEED", 1200)))
    parser.add_argument("--no-viewer", action="store_true", help="Collect data without a browser server")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    data = Path(os.environ.get("DATA_DIR", str(ROOT / "runs"))).expanduser().resolve()
    live, cases = data / "live", data / "cases"
    live.mkdir(parents=True, exist_ok=True)
    cases.mkdir(parents=True, exist_ok=True)
    lock = (data / "runner.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(f"An instance is already using {data}", file=sys.stderr)
        return 1
    endpoint = os.environ.get("JEV_ENDPOINT", "http://127.0.0.1:8000/v1/systemone")
    health = endpoint.rsplit("/v1/", 1)[0] + "/health"
    try:
        with urlopen(health, timeout=5) as response:
            json.load(response)
    except Exception as exc:
        print(f"Jev service unavailable: {exc}. Start it with bash run_model.sh; see docs/model-setup.md.", file=sys.stderr)
        return 1
    if not (ROOT / "sim/mujoco_menagerie/skydio_x2/x2.xml").is_file():
        print("Missing Skydio X2 assets. Run bash setup.sh first.", file=sys.stderr)
        return 1
    env = os.environ.copy()
    env.setdefault("MUJOCO_GL", "egl" if sys.platform.startswith("linux") else "glfw")
    env["JEV_ENDPOINT"] = endpoint
    env["DUEL_LIVE_DIR"] = str(live)
    env["DUEL_VIEW_PORT"] = env.get("VIEW_PORT", "8080")
    viewer = episode = None
    try:
        if not args.no_viewer:
            # Reject a port conflict rather than serving another instance's data.
            with socket.socket() as probe:
                probe.bind(("127.0.0.1", int(env["DUEL_VIEW_PORT"])))
            viewer = subprocess.Popen([sys.executable, "sim/urban_view.py"], cwd=ROOT, env=env)
            print(f"Viewer: http://127.0.0.1:{env['DUEL_VIEW_PORT']}/", flush=True)
        seed = args.seed
        while True:
            if viewer and viewer.poll() is not None:
                raise RuntimeError("Viewer exited; check the port and console output")
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            case = cases / f"{stamp}_seed{seed}"
            print(f"Starting episode seed={seed}", flush=True)
            episode = subprocess.Popen([sys.executable, "sim/duel.py", "--urban", "--seconds", str(args.seconds),
                                        "--seed", str(seed), "--live-dir", str(live), "--endpoint", endpoint], cwd=ROOT, env=env)
            code = episode.wait()
            episode = None
            if code:
                return code
            case.mkdir()
            for name in ("decisions.jsonl", "events.jsonl", "referee.jsonl", "summary.json", "policy.json"):
                shutil.copy2(live / name, case / name)
            print(f"Archived: {case}", flush=True)
            if args.once:
                return 0
            seed += 1
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping simulation and viewer.", file=sys.stderr)
        return 0
    except (OSError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        for child in (episode, viewer):
            if child and child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
