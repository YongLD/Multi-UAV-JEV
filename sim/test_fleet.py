"""Behavior checks for cooperative tracking and fleet selection."""
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from urllib.request import Request, urlopen
from urllib.error import HTTPError
import numpy as np

from fleet_config import read_config, validate_config
from fleet_tracking import SharedTargetTrack, separation_velocity
from onboard_target import rotate


def report(name, position, yaw, measured_world=None, confidence=.9):
    target = {'visible': measured_world is not None}
    if measured_world is not None:
        target.update(relative_position_m=rotate(np.asarray(measured_world)-position, -yaw).tolist(),
                      track_confidence=confidence, position_uncertainty_m=.1,
                      relative_velocity_mps=[0., 0., 0.])
    return {'id': name, 'position': np.array(position, dtype=float), 'yaw': yaw,
            'velocity': np.zeros(3), 'target': target}


class TrackingTests(unittest.TestCase):
    def test_teammate_observation_transforms_into_receivers_frame(self):
        source = report('P-01', [0, 0, 12], 0, [20, 10, 15])
        receiver = report('P-02', [10, 0, 15], math.pi/2)
        track = SharedTargetTrack(); track.update([source, receiver], 0)
        received = track.relative(receiver, 0)
        np.testing.assert_allclose(received['relative_position_m'], [10, -10, 0], atol=.01)
        self.assertFalse(received['visible'])  # sharing must not invent local camera visibility
        self.assertTrue(received['shared_available'])
        self.assertEqual(received['observation_sources'], ['P-01'])

    def test_two_views_fuse_without_double_transform_and_outlier_is_rejected(self):
        reports = [report('P-01', [0, 0, 12], 0, [20, 10, 15]),
                   report('P-02', [10, -5, 14], math.pi/2, [20, 10, 15]),
                   report('P-03', [1, 1, 10], math.pi, [60, 60, 40], .1)]
        track = SharedTargetTrack(); track.update(reports, 0)
        np.testing.assert_allclose(track.position, [20, 10, 15], atol=.01)
        self.assertEqual(track.sources, ['P-01', 'P-02'])
        self.assertEqual(track.rejected, 1)

    def test_predictions_do_not_refresh_measurement_age(self):
        track = SharedTargetTrack(); source = report('P-01', [0, 0, 12], 0, [20, 0, 12])
        track.update([source], 0)
        source['target'] = {'visible': False}
        state = track.update([source], 1)
        self.assertEqual(state['mode'], 'predicted')
        self.assertEqual(state['age_s'], 1)
        self.assertFalse(track.update([source], 3.1)['available'])
        self.assertFalse(track.relative(source, 3.1)['shared_available'])

    def test_separation_brakes_approaching_peer_and_preserves_direction(self):
        correction, closest = separation_velocity(np.array([0., 0., 12.]), np.array([4., 0., 0.]),
                                                   [{'position': np.array([5., 0., 12.]), 'velocity': np.array([-4., 0., 0.])}])
        self.assertLess(correction[0], 0)
        self.assertLessEqual(np.linalg.norm(correction), 3.)
        self.assertEqual(closest, 5.)


class FleetAPITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import urban_view
        cls.module = urban_view
        cls.directory = tempfile.TemporaryDirectory()
        urban_view.base.LIVE = Path(cls.directory.name)
        cls.server = urban_view.ThreadingHTTPServer(('127.0.0.1', 0), urban_view.UrbanHandler)
        cls.url = f'http://127.0.0.1:{cls.server.server_port}'
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True); cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown(); cls.server.server_close(); cls.directory.cleanup()

    def post(self, path, value):
        with urlopen(Request(self.url+path, json.dumps(value).encode(), {'Content-Type': 'application/json'})) as response:
            return json.load(response)

    def test_count_changes_physical_runner_request_and_focus_does_not_restart(self):
        live = self.module.base.LIVE
        for count in (1, 2, 4, 1):
            self.post('/api/fleet', {'count': count, 'focus': 0})
            self.assertEqual(read_config(live)['count'], count)
        self.post('/api/fleet', {'count': 4})
        self.assertTrue((live/'restart.request').exists())
        (live/'restart.request').unlink()
        self.post('/api/fleet/focus', {'focus': 3})
        self.assertEqual(read_config(live)['focus'], 3)
        self.assertFalse((live/'restart.request').exists())
        for invalid in (0, 3, True, '4', None):
            with self.assertRaises(HTTPError) as caught:
                self.post('/api/fleet', {'count': invalid})
            self.assertEqual(caught.exception.code, 400)
        self.assertEqual(read_config(live)['count'], 4)

    def test_each_monitor_uses_its_own_frame_and_inactive_frames_are_hidden(self):
        live=self.module.base.LIVE
        (live/'status.json').write_text(json.dumps({'fleet_count':4}))
        for index in range(4):
            data=b'camera-'+str(index).encode()
            (live/f'drone_{index}.jpg').write_bytes(data)
            with urlopen(self.url+f'/drone_{index}.jpg') as response:self.assertEqual(response.read(),data)
        (live/'status.json').write_text(json.dumps({'fleet_count':2}))
        with self.assertRaises(HTTPError):urlopen(self.url+'/drone_3.jpg')
        (live/'status.json').write_text(json.dumps({'fleet_count':1}))
        (live/'scene.jpg').write_bytes(b'follow-single');(live/'rgb.jpg').write_bytes(b'onboard-single')
        with urlopen(self.url+'/drone_0.jpg') as response:self.assertEqual(response.read(),b'follow-single')

    def test_pending_count_requeues_a_lost_restart_request(self):
        live=self.module.base.LIVE
        (live/'fleet.json').write_text(json.dumps({'count':4,'focus':0}))
        (live/'status.json').write_text(json.dumps({'fleet_count':2}))
        (live/'restart.request').unlink(missing_ok=True)
        response=self.post('/api/fleet',{'count':4})
        self.assertTrue(response['queued']);self.assertTrue((live/'restart.request').exists())


if __name__ == '__main__':
    unittest.main()
