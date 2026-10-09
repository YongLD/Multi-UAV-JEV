"""Upstream-style live/replay browser for the city duel."""
from __future__ import annotations

import os
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from camera_controls import DEFAULT_CAMERA, validate_camera

LIVE = Path(os.environ.get("DUEL_LIVE_DIR", str(Path(__file__).resolve().parent / "../runs/live")))
PORT = int(os.environ.get("DUEL_VIEW_PORT", "8080"))

PAGE = Path(__file__).with_name('urban.html').read_text(encoding='utf-8')


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Frame polling is frequent; avoid growing a server log on every JPEG.
        pass

    def do_POST(self):
        if self.path == "/api/camera":
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 2048:
                    raise ValueError("invalid body length")
                value = validate_camera(json.loads(self.rfile.read(size)))
            except (ValueError, TypeError):
                return self.send_error(400)
            LIVE.mkdir(parents=True, exist_ok=True)
            temporary = LIVE/"camera.json.tmp"
            temporary.write_text(json.dumps(value))
            temporary.replace(LIVE/"camera.json")
            return self.bytes(json.dumps(value).encode(), "application/json")
        if self.path != "/api/restart":
            return self.send_error(404)
        LIVE.mkdir(parents=True, exist_ok=True)
        (LIVE/"restart.request").write_text("restart\n", encoding="utf-8")
        self.bytes(b'{"queued":true}', "application/json")

    def do_GET(self):
        if self.path == "/api/camera":
            if (LIVE/"camera.json").is_file():
                return self.file(LIVE/"camera.json", "application/json")
            return self.bytes(json.dumps(DEFAULT_CAMERA).encode(), "application/json")
        if self.path == "/":
            return self.bytes(PAGE.encode(), "text/html; charset=utf-8")
        if self.path.startswith("/api/status"):
            return self.file(LIVE/"status.json", "application/json")
        if self.path.startswith("/api/replay_summary"):
            if not (LIVE/"replay.mp4").is_file():
                return self.send_error(404)
            return self.file(LIVE/"replay_summary.json", "application/json")
        if self.path.startswith("/frame.jpg"):
            return self.file(LIVE/"frame.jpg", "image/jpeg")
        if self.path.startswith("/replay/latest.mp4"):
            return self.file(LIVE/"replay.mp4", "video/mp4", ranges=True)
        if self.path == "/data/decisions.jsonl":
            return self.file(LIVE/"decisions.jsonl", "application/x-ndjson")
        if self.path == "/data/events.jsonl":
            return self.file(LIVE/"events.jsonl", "application/x-ndjson")
        self.send_error(404)

    def bytes(self, body: bytes, kind: str):
        self.send_response(200)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def file(self, path: Path, kind: str, *, ranges=False):
        if not path.is_file():
            return self.send_error(404)
        size = path.stat().st_size
        start, end = 0, size-1
        header = self.headers.get("Range", "") if ranges else ""
        if header.startswith("bytes="):
            try:
                first, last = header[6:].split("-", 1)
                if not first:
                    start = max(0, size-int(last))
                else:
                    start = int(first)
                    end = min(int(last), size-1) if last else end
                if start < 0 or start > end or start >= size:
                    raise ValueError
            except ValueError:
                self.send_response(416)
                self.send_header("Content-Range", f"bytes */{size}")
                self.end_headers()
                return
        self.send_response(206 if header else 200)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(end-start+1))
        self.send_header("Cache-Control", "no-store")
        if ranges:
            self.send_header("Accept-Ranges", "bytes")
        if header:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        try:
            with path.open("rb") as stream:
                stream.seek(start)
                remaining = end-start+1
                while remaining:
                    chunk = stream.read(min(128*1024, remaining))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass


if __name__ == "__main__":
    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()
