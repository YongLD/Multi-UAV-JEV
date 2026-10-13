from collections import deque
from types import SimpleNamespace
import unittest

import numpy as np

from fleet_visuals import WorldProjector, world_trajectory, camera_overlay, remember_shot, shot_status, prefer_active_shot


class VisualProjectionTests(unittest.TestCase):
    def test_cooldown_request_preserves_the_just_executed_hit(self):
        hit={'time_s':2.,'number':1,'hit':True,'cooldown_ok':True}
        cooldown={'time_s':2.1,'number':2,'hit':False,'cooldown_ok':False}
        self.assertIs(prefer_active_shot(hit,cooldown,2.1),hit)
        status=shot_status(hit,2.2,cooldown)
        self.assertEqual(status['state'],'hit')
        self.assertEqual(status['attempt_state'],'cooldown')
        self.assertEqual(status['number'],1)
        self.assertIs(prefer_active_shot(hit,cooldown,3.3),cooldown)

    def test_each_camera_projects_same_world_path_into_its_own_view(self):
        camera=SimpleNamespace(pos=np.zeros(3),forward=np.array([1.,0,0]),up=np.array([0.,0,1]))
        first=WorldProjector(camera,640,360)
        camera.pos[:]=[0,4,0]
        second=WorldProjector(camera,640,360)
        self.assertNotEqual(first([10,2,0]),second([10,2,0]))
        self.assertEqual(first([10,0,0]),(320.,180.))
        self.assertIsNone(first([-1,0,0]))
        self.assertIsNone(first([1,float('nan'),0]))

    def test_stale_observation_has_no_predicted_line(self):
        track=SimpleNamespace(summary=lambda now:{'available':False},history=deque())
        predictor=SimpleNamespace(position=np.zeros(3))
        path=world_trajectory(track,predictor,10.)
        self.assertEqual(path,{'history':[],'predictions':[],'current':None})

    def test_result_labels_and_cooldown_do_not_create_a_tracer(self):
        camera=SimpleNamespace(pos=np.array([-5.,0,2]),forward=np.array([1.,0,0]),up=np.array([0.,0,1]))
        project=WorldProjector(camera,640,360)
        path={'history':[np.array([10,1,2])],'predictions':[],'current':np.array([10,1,2])}
        hit=remember_shot({'hit':True,'cooldown_ok':True},[0,0,2],0,[10,1,2],2.,1)
        self.assertEqual(shot_status(hit,2.1)['state'],'hit')
        self.assertIsNotNone(camera_overlay(project,path,hit,2.2)['shot_effect']['end_xy'])
        self.assertNotIn('shot_effect',camera_overlay(project,path,hit,3.3))
        for state,result in [('miss',{'hit':False,'cooldown_ok':True}),('cooldown',{'hit':False,'cooldown_ok':False}),('blocked',{'hit':False,'cooldown_ok':False,'blocked':True})]:
            self.assertEqual(shot_status({**result,'time_s':2,'number':1},2.1)['state'],state)


class VisualCompositorTests(unittest.TestCase):
    def test_prediction_and_shot_effect_are_baked_into_the_recorded_frames(self):
        from urban_hud import UrbanHud
        from fleet_replay import FleetReplayRecorder
        import imageio.v2 as imageio
        from pathlib import Path
        import tempfile
        base=np.zeros((360,640,3),dtype=np.uint8)
        hud=UrbanHud(640,360)
        blank=np.asarray(hud.decorate(base,{},None).convert('RGB'))
        predicted={'trajectory_overlay':{'history_xy':[(80,250),(130,240)],'predicted_xy':[
            {'xy':(200,220),'horizon_s':1.,'confidence':.8},{'xy':(270,200),'horizon_s':2.,'confidence':.6}]}}
        path=np.asarray(hud.decorate(base,predicted,None).convert('RGB'))
        self.assertGreater(np.count_nonzero(path[180:280,60:290]-blank[180:280,60:290]),100)
        effect={'age_s':.2,'number':1,'hit':True,'cooldown_ok':True,'start_xy':(320,180),'end_xy':(450,170),'display_duration_s':1.2,'trace_duration_s':.45}
        fired=np.asarray(hud.decorate(base,{**predicted,'shot_effect':effect},None).convert('RGB'))
        self.assertGreater(np.count_nonzero(fired[145:190,330:465]-path[145:190,330:465]),100)
        with tempfile.TemporaryDirectory() as temporary:
            live=Path(temporary);recorder=FleetReplayRecorder(live,2,5)
            for _ in range(4):recorder.append([fired,path])
            recorder.close();summary={'hits':5,'fleet_count':2}
            self.assertTrue(recorder.retain(live,summary))
            for i,view in enumerate(summary['replay_views']):
                reader=imageio.get_reader(live/'replay_sets'/Path(view['url']).parts[-2]/Path(view['url']).name)
                try:frame=reader.get_data(1)
                finally:reader.close()
                green=(frame[145:190,330:465,1].astype(int)>frame[145:190,330:465,0].astype(int)+30)
                if i==0:self.assertGreater(np.count_nonzero(green),40)
                else:self.assertLess(np.count_nonzero(green),20)


if __name__=='__main__':unittest.main()
