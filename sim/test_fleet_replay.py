import json
from pathlib import Path
import tempfile
import unittest
from urllib.request import Request, urlopen

import imageio.v2 as imageio
import numpy as np

from fleet_replay import FleetReplayRecorder
from test_fleet import FleetAPITests


class RecordingTests(unittest.TestCase):
    def test_recorded_views_keep_own_rgb_depth_actions_flight_and_overview(self):
        from hud import _depth_rgb
        from PIL import Image
        with tempfile.TemporaryDirectory() as temporary:
            live=Path(temporary)
            for count in (2,4):
                views=[np.full((360,640,3),30+i*25,dtype=np.uint8) for i in range(count)]
                packets=[]
                for i in range(count):
                    actions=('turn_left','turn_right','climb','descend','forward')+(('rescue','self_rescue') if i==3 else ('fire',))
                    rgb=np.full((144,192,3),(25+i*45,180-i*25,50+i*40),dtype=np.uint8)
                    depth=np.linspace(1+i,35-i,144*192).reshape(144,192)
                    overview=np.full((288,448,3),(70,110,150),dtype=np.uint8)
                    packets.append({'rgb':rgb,'depth':depth,'overview':overview,
                        'scene':{'target':{'visible':True,'camera':'front','range_m':12+i,'relative_height_m':-1.},
                                 'sector_range_m':dict.fromkeys(('left2','left','front','right','right2'),20.)},
                        'decision':{'action':actions[-1],'probabilities':{actions[-1]:.7}},
                        'telemetry':{'focused_drone_id':f'P-{i+1:02d}','fleet_count':count,'display_mode':'REPLAY',
                                     'altitude_m':10.+i,'speed_mps':3.+i,'counts':dict.fromkeys(actions,i+2),
                                     'allowed_actions':actions,'calls':i+1,'errors':0,'latency_s':.25,
                                     'rain_factor':1.,'collisions':i,'hits':i,'shots':i+3,'time_s':.5,
                                     'sensor_age_s':.05,'overview_age_s':.1}})
                recorder=FleetReplayRecorder(live,count,20)
                for i in range(6):recorder.append(views,sim_time=i/20,instruments=packets,publish_live=True)
                recorder.close();summary={'hits':5,'fleet_count':count}
                self.assertTrue(recorder.retain(live,summary))
                self.assertTrue(summary['replay_instruments'])
                self.assertEqual(summary['replay_ui'],'single_drone_full_hud')
                frames=[]
                for view in summary['replay_views']:
                    folder=live/'replay_sets'/summary['replay_id']
                    reader=imageio.get_reader(folder/Path(view['url']).name)
                    try:
                        self.assertEqual(reader.count_frames(),6)
                        self.assertEqual(reader.get_meta_data()['fps'],20)
                        frame=reader.get_data(1);frames.append(frame)
                    finally:reader.close()
                    packet=packets[len(frames)-1]
                    self.assertEqual(frame.shape,(720,1280,3))
                    expected=[np.asarray(Image.fromarray(packet['rgb']).resize((118,89))),
                              np.asarray(Image.fromarray(_depth_rgb(packet['depth'],35)).resize((118,89),Image.Resampling.NEAREST)),
                              np.asarray(Image.fromarray(packet['overview']).resize((274,156)))]
                    actual=[frame[162:251,1006:1124],frame[162:251,1130:1248],frame[516:672,976:1250]]
                    for a,e in zip(actual,expected):self.assertLess(np.abs(a.astype(float)-e).mean(),10)
                    live_frame=np.asarray(Image.open(live/f'drone_{len(frames)-1}.jpg'))
                    self.assertEqual(live_frame.shape,(720,1280,3))
                    for a,e in zip([live_frame[162:251,1006:1124],live_frame[162:251,1130:1248],live_frame[516:672,976:1250]],expected):
                        self.assertLess(np.abs(a.astype(float)-e).mean(),10)
                    for y0,y1,x0,x1 in ((16,255,16,230),(377,703,16,294),(583,703,714,946)):
                        self.assertGreater(float(frame[y0:y1,x0:x1].std()),8)
                self.assertGreater(np.abs(frames[0][16:255,16:230].astype(float)-frames[1][16:255,16:230]).mean(),.1)

    def test_instrument_snapshot_is_detached_from_live_drone_state(self):
        from fleet_sim import replay_instrument_snapshot
        from types import SimpleNamespace
        agent=SimpleNamespace(id='P-02',v=0,replay_rgb=np.zeros((144,192,3),dtype=np.uint8),
            replay_depth=np.ones((144,192)),local_target={'visible':True,'range_m':12.},
            scene={'sector_range_m':{'front':10.}},decision={'action':'fire','probabilities':{'fire':.7},'latency_s':.3},
            counts={'fire':2},allowed_actions=('forward','fire'),collisions=1,distance=9.,observed_at=.9,
            worker=SimpleNamespace(model='model',calls=3,errors=0),controller=SimpleNamespace(hits=1,shots=2),
            position=lambda data:np.array([0.,0.,12.]),
            own=lambda data:dict(altitude_m=12.,speed_mps=4.,yaw_deg=10.,roll_deg=1.,pitch_deg=2.))
        data=SimpleNamespace(qvel=np.zeros(6));overview=np.zeros((288,448,3),dtype=np.uint8)
        packet=replay_instrument_snapshot(agent,data,1.,2,np.array([100,100]),overview,.8,5)
        agent.counts['fire']=99;agent.decision['probabilities']['fire']=0;agent.local_target['range_m']=90.
        agent.scene['sector_range_m']['front']=1.;previous_rgb=agent.replay_rgb;agent.replay_rgb=np.ones_like(previous_rgb)
        self.assertEqual(packet['telemetry']['counts']['fire'],2)
        self.assertEqual(packet['decision']['probabilities']['fire'],.7)
        self.assertEqual(packet['scene']['target']['range_m'],12.)
        self.assertEqual(packet['scene']['sector_range_m']['front'],10.)
        self.assertIs(packet['rgb'],previous_rgb)
        self.assertEqual(packet['telemetry']['focused_drone_id'],'P-02')
        agent.local_target={}
        self.assertFalse(replay_instrument_snapshot(agent,data,1.,2,np.array([100,100]),overview,.8,5)['scene']['target']['visible'])

    def test_all_views_have_equal_frames_and_mosaic_contains_each_camera(self):
        with tempfile.TemporaryDirectory() as temporary:
            live = Path(temporary)
            for count in (2, 4):
                recorder = FleetReplayRecorder(live, count, 20)
                for t in range(20):
                    recorder.append([np.full((64, 112, 3), 40+i*45+t, dtype=np.uint8) for i in range(count)],sim_time=t/20)
                recorder.close()
                summary = {'hits': 5, 'fleet_count': count}
                self.assertTrue(recorder.retain(live, summary))
                self.assertEqual(len(summary['replay_views']), count)
                self.assertAlmostEqual(summary['replay_max_frame_gap_s'],.05)
                self.assertEqual(len(json.loads((live/'replay_sets'/summary['replay_id']/'frame_times.json').read_text())),20)
                streams = [summary['replay_main_url']]+[v['url'] for v in summary['replay_views']]
                for index, url in enumerate(streams):
                    path = live/'replay_sets'/Path(url).parts[-2]/Path(url).name
                    reader = imageio.get_reader(path)
                    try:
                        self.assertEqual(reader.count_frames(), 20)
                        self.assertEqual(reader.get_meta_data()['fps'], 20)
                        frame = reader.get_data(3)
                        final=reader.get_data(19)
                        self.assertGreater(float(final[35:55,30:90].mean()),float(frame[35:55,30:90].mean())+10)
                        if index:
                            self.assertAlmostEqual(float(frame[35:55,30:90].mean()), 43+(index-1)*45, delta=5)
                        else:
                            for i in range(count):
                                row, col = divmod(i, 2)
                                self.assertAlmostEqual(float(frame[row*64+35:row*64+55,col*112+30:col*112+90].mean()), 43+i*45, delta=5)
                    finally:
                        reader.close()
                old = json.loads((live/'replay_summary.json').read_text())
                rejected = FleetReplayRecorder(live, count, 10)
                rejected.append([np.zeros((64,112,3), dtype=np.uint8) for _ in range(count)])
                rejected.close()
                self.assertFalse(rejected.retain(live, {'hits': 4}))
                self.assertEqual(json.loads((live/'replay_summary.json').read_text()), old)


class ReplayAPITests(FleetAPITests):
    def test_catalog_lists_two_per_group_and_best_compatibility_route(self):
        from replay_store import retain_single_replay
        live=self.module.base.LIVE
        for seed,hits in enumerate((12,28,17,6),1):
            video=live/f'episode_{seed}.mp4';video.write_bytes(f'video-{seed}'.encode())
            retain_single_replay(live,video,{'hits':hits,'seed':seed,'fleet_count':1})
        with urlopen(self.url+'/api/replays') as response:
            catalog=json.load(response)
        self.assertEqual(catalog['limit_per_fleet'],2)
        self.assertEqual(catalog['total_limit'],6)
        self.assertEqual([e['hits'] for e in catalog['groups']['1']],[28,17])
        self.assertEqual(catalog['groups']['2'],[])
        self.assertEqual(catalog['groups']['4'],[])
        with urlopen(self.url+'/api/replay_summary') as response:self.assertEqual(json.load(response)['hits'],28)
        with urlopen(self.url+'/replay/latest.mp4') as response:self.assertEqual(response.read(),b'video-2')

    def test_immutable_view_supports_seek_ranges(self):
        identity = 'a'*32
        folder = self.module.base.LIVE/'replay_sets'/identity
        folder.mkdir(parents=True)
        (folder/'drone_3.mp4').write_bytes(b'0123456789')
        with urlopen(Request(self.url+f'/replay/sets/{identity}/drone_3.mp4', headers={'Range':'bytes=3-6'})) as response:
            self.assertEqual(response.status, 206)
            self.assertEqual(response.read(), b'3456')
            self.assertEqual(response.headers['Content-Range'], 'bytes 3-6/10')


if __name__ == '__main__':
    unittest.main()
