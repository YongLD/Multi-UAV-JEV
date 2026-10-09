"""Full-frame waterfront browser, sharing the tested duel endpoints/replay API."""
from pathlib import Path
from http.server import ThreadingHTTPServer
import os

os.environ.setdefault('DUEL_LIVE_DIR', str(Path(__file__).resolve().parent.parent / 'runs/live'))
os.environ.setdefault('DUEL_VIEW_PORT', '8080')
import duel_view as base

base.PAGE = Path(__file__).with_name('urban.html').read_text()
base.DEFAULT_CAMERA = {'mode': 'follow', 'azimuth': 0., 'elevation': -14., 'distance': 5.8}


class UrbanHandler(base.Handler):
    def do_GET(self):
        for name in ('scene', 'rgb', 'depth', 'overview'):
            if self.path.split('?')[0] == '/'+name+'.jpg':
                return self.file(base.LIVE/(name+'.jpg'), 'image/jpeg')
        return super().do_GET()


if __name__ == '__main__':
    ThreadingHTTPServer(('127.0.0.1', base.PORT), UrbanHandler).serve_forever()
