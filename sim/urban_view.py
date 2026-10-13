"""Full-frame waterfront browser, sharing the tested duel endpoints/replay API."""
from pathlib import Path
from http.server import ThreadingHTTPServer
import os
import json
import threading
import re
from fleet_config import read_config, validate_config
from replay_store import list_replays, replay_path, replay_catalog

os.environ.setdefault('DUEL_LIVE_DIR', str(Path(__file__).resolve().parent.parent / 'runs/live'))
os.environ.setdefault('DUEL_VIEW_PORT', '8080')
import duel_view as base

base.PAGE = Path(__file__).with_name('urban.html').read_text(encoding='utf-8')
base.DEFAULT_CAMERA = {'mode': 'follow', 'azimuth': 0., 'elevation': -14., 'distance': 5.8}


class UrbanHandler(base.Handler):
    protocol_version = 'HTTP/1.1'
    config_lock = threading.Lock()

    def do_POST(self):
        if self.path not in ('/api/fleet', '/api/fleet/focus'):
            return super().do_POST()
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 1024:
                raise ValueError('invalid body size')
            payload = json.loads(self.rfile.read(size))
            with self.config_lock:
                previous = read_config(base.LIVE)
                if self.path == '/api/fleet/focus':
                    value = validate_config({'count': previous['count'], 'focus': payload['focus']})
                else:
                    value = validate_config(payload)
                base.LIVE.mkdir(parents=True, exist_ok=True)
                temporary = base.LIVE/'fleet.json.tmp'
                temporary.write_text(json.dumps(value))
                temporary.replace(base.LIVE/'fleet.json')
                try:actual=json.loads((base.LIVE/'status.json').read_text()).get('fleet_count',previous['count'])
                except (OSError,ValueError):actual=previous['count']
                queued=self.path=='/api/fleet' and (value['count'] != previous['count'] or value['count'] != actual)
                if queued:
                    (base.LIVE/'restart.request').write_text('fleet_change\n')
            return self.bytes(json.dumps({**value, 'queued': queued}).encode(), 'application/json')
        except (ValueError, TypeError, KeyError):
            return self.send_error(400, 'count must be 1, 2 or 4; focus must be an active drone')

    def do_GET(self):
        path = self.path.split('?')[0]
        if path in ('/api/replays','/api/replay_summary','/replay/latest.mp4'):
            replays = list_replays(base.LIVE)
            if path == '/api/replays':
                return self.bytes(json.dumps(replay_catalog(replays)).encode(),'application/json')
            if not replays:
                return self.send_error(404,'no qualified replay')
            if path == '/api/replay_summary':
                return self.bytes(json.dumps(replays[0]).encode(),'application/json')
            return self.file(replay_path(base.LIVE,replays[0]),'video/mp4',ranges=True)
        match = re.fullmatch(r'/replay/sets/([a-f0-9]{32})/(all|drone_[0-3])\.mp4', path)
        if match:
            return self.file(base.LIVE/'replay_sets'/match.group(1)/(match.group(2)+'.mp4'),
                             'video/mp4', ranges=True)
        if path == '/api/fleet':
            return self.bytes(json.dumps(read_config(base.LIVE)).encode(), 'application/json')
        if path == '/api/status':
            try:
                status = json.loads((base.LIVE/'status.json').read_text())
                status['fleet_requested_count'] = read_config(base.LIVE)['count']
                return self.bytes(json.dumps(status).encode(), 'application/json')
            except (OSError, ValueError):
                return self.send_error(503, 'simulation starting')
        for name in ('scene', 'rgb', 'depth', 'overview'):
            if self.path.split('?')[0] == '/'+name+'.jpg':
                # Open the atomic frame once: stat() followed by open() can
                # measure one frame and read its replacement at a different size.
                try:
                    frame = (base.LIVE/(name+'.jpg')).read_bytes()
                except OSError:
                    return self.send_error(503, 'camera frame starting')
                return self.bytes(frame, 'image/jpeg')
        match = re.fullmatch(r'/drone_([0-3])\.jpg',path)
        if match:
            try:
                status = json.loads((base.LIVE/'status.json').read_text())
                index = int(match.group(1))
                if index >= status.get('fleet_count',1):
                    return self.send_error(404,'inactive drone')
                frame = (base.LIVE/(f'drone_{index}.jpg' if status.get('fleet_count',1)>1 else 'scene.jpg')).read_bytes()
                return self.bytes(frame,'image/jpeg')
            except (OSError,ValueError):return self.send_error(503,'camera starting')
        return super().do_GET()


if __name__ == '__main__':
    ThreadingHTTPServer.request_queue_size = 128
    ThreadingHTTPServer(('127.0.0.1', base.PORT), UrbanHandler).serve_forever()
