"""Mission behavior checks: forward cutoff, timed interception and leadership separation."""
from collections import deque
import math
import unittest
import numpy as np
from fleet_roles import RoleBook
from fleet_missions import MissionPlanner,TrajectoryPredictor


class ObservedTrack:
    def __init__(self,now=3.,turn=0.,speed=4.,confidence=.95):
        self.observed_at=now;self.confidence=confidence;self.uncertainty=.1
        self.history=deque()
        for t in np.arange(0,now+.001,.15):
            p=np.array([10+speed*t,0.,16.]) if not turn else np.array([10+speed*math.sin(turn*t)/turn,speed*(1-math.cos(turn*t))/turn,16.])
            self.history.append((float(t),p))
        self.position=self.history[-1][1].copy()
        self.velocity=np.array([speed*math.cos(turn*now),speed*math.sin(turn*now),0.])

    def summary(self,now):
        age=now-self.observed_at
        return {'available':age<=3,'age_s':age,'confidence':self.confidence*math.exp(-age/1.5)}


def reports():
    return [{'id':f'P-{i+1:02d}','position':np.array(p,dtype=float),'velocity':np.zeros(3),'yaw':0.,'alive':True,
             'target':{'visible':True,'track_confidence':.8,'range_m':20.}} for i,p in enumerate(((0,0,16),(40,12,16),(28,-12,16),(24,0,20)))]


class MissionTests(unittest.TestCase):
    def test_three_missions_have_different_goals_and_only_tracking_matches_velocity(self):
        book=RoleBook();planner=MissionPlanner(book,4);track=ObservedTrack();r=reports()
        planner.update(r,track,3.,'P-01')
        goals=[planner.plans[i]['point'] for i in range(3)]
        self.assertLess(goals[0][0],track.position[0])
        self.assertGreater(goals[1][0],track.position[0]+book.config['tactics']['block_front_m'])
        self.assertGreater(goals[2][0],track.position[0])
        self.assertGreater(np.linalg.norm(goals[1]-goals[2]),5.)
        self.assertGreater(np.linalg.norm(planner.motion(0)['feedforward']),0)
        np.testing.assert_allclose(planner.motion(1)['feedforward'],0)
        np.testing.assert_allclose(planner.motion(2)['feedforward'],0)
        self.assertLessEqual(planner.plans[2]['own_eta_s'],planner.plans[2]['target_eta_s'])

    def test_block_point_is_committed_and_becomes_a_stationary_ambush(self):
        book=RoleBook();planner=MissionPlanner(book,4);track=ObservedTrack();r=reports()
        planner.update(r,track,3.,'P-01');point=planner.plans[1]['point'].copy()
        track.position+=track.velocity*.5;track.observed_at=3.5;track.history.append((3.5,track.position.copy()))
        r[1]['position']=point.copy();planner.update(r,track,3.5,'P-01')
        np.testing.assert_allclose(planner.plans[1]['point'],point)
        self.assertEqual(planner.plans[1]['status'],'block_hold')
        np.testing.assert_allclose(planner.motion(1)['feedforward'],0.)
        np.testing.assert_allclose(planner.motion(1)['face'],track.position,atol=.1)

    def test_turn_history_changes_prediction_instead_of_a_fixed_forward_offset(self):
        book=RoleBook();track=ObservedTrack(turn=math.radians(12));predictor=TrajectoryPredictor()
        predictor.update(track,book.config['tactics'])
        self.assertEqual(predictor.model,'constant_turn_rate')
        self.assertGreater(np.linalg.norm(predictor.predict(3.,5.,True)-predictor.predict(3.,5.,False)),5.)
        planner=MissionPlanner(book,4);planner.update(reports(),track,3.,'P-01')
        self.assertEqual(planner.plans[2]['prediction_model'],'constant_turn_rate')
        self.assertGreater(np.linalg.norm(planner.plans[1]['point']-planner.plans[2]['point']),8.)

    def test_unreachable_and_uncertain_predictions_are_explicit(self):
        book=RoleBook();planner=MissionPlanner(book,4);track=ObservedTrack(speed=9.);r=reports()
        r[2]['position']=np.array([-40.,-20.,16.])
        planner.update(r,track,3.,'P-01')
        self.assertFalse(planner.plans[2]['feasible']);self.assertEqual(planner.plans[2]['status'],'intercept_unreachable')
        track.confidence=.1;planner.update(r,track,3.3,'P-01')
        for index in (1,2):
            self.assertIsNone(planner.plans[index]['point'])
            self.assertEqual(planner.plans[index]['status'],'waiting_observation')

    def test_far_prediction_damps_height_and_near_interception_faces_actual_target(self):
        book=RoleBook();track=ObservedTrack()
        track.history=deque((t,np.array([p[0],p[1],16+.8*t])) for t,p in track.history)
        track.position=track.history[-1][1].copy();track.velocity[2]=.8
        predictor=TrajectoryPredictor();predictor.update(track,book.config['tactics'])
        self.assertLessEqual(predictor.predict(3.,10.)[2]-track.position[2],1.7)
        planner=MissionPlanner(book,4);r=reports();r[2]['position']=track.position+np.array([0.,-5.,0.])
        planner.update(r,track,3.,'P-01')
        self.assertEqual(planner.plans[2]['status'],'intercept_window')
        self.assertLess(np.linalg.norm(planner.plans[2]['point']-track.position),6.)
        np.testing.assert_allclose(planner.motion(2)['face'],track.position)


class CoordinatorMissionTests(unittest.TestCase):
    def test_best_observation_source_keeps_blockade_task_while_primary_is_healthy(self):
        from fleet_control import TeamCoordinator
        book=RoleBook();team=TeamCoordinator(book,4);track=ObservedTrack();r=reports()
        r[0]['target']['visible']=False;r[1]['target']['track_confidence']=.99
        own=[{'tilt_deg':0.,'roll_deg':0.,'pitch_deg':0.,'speed_mps':1.,'front_clearance_m':10.,'navigation_distance_m':20.} for _ in r]
        team.update(r,own,track,3.)
        self.assertEqual(team.effective_tracker,'P-02')
        self.assertEqual(team.primary_tracking_agent,'P-01')
        self.assertEqual(team.mission(1,3.)['kind'],'front_block')
        own[0]['tilt_deg']=80.;team.update(r,own,track,3.3)
        self.assertEqual(team.mission(1,3.3)['kind'],'tracking_relay')


if __name__=='__main__':unittest.main()
