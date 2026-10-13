import math
import unittest
import numpy as np
from fleet_roles import RoleBook
from fleet_control import TeamCoordinator
from fleet_tracking import SharedTargetTrack
from test_fleet_rescue import reports


class EscapeView:
    def __init__(self,unknown=False,blocked=False):self.unknown=unknown;self.blocked=blocked
    def corridor(self,*args):return {'clearance_m':None if self.unknown else 10.,'status':'unknown' if self.unknown else 'clear_to_limit'}
    def query(self,points):return np.full(len(points),not self.unknown),np.full(len(points),self.blocked)


class SelfRescueTests(unittest.TestCase):
    def setUp(self):
        self.book=RoleBook();self.team=TeamCoordinator(self.book,4);self.track=SharedTargetTrack();self.reports=reports()
        self.own=[dict(tilt_deg=0.,roll_deg=0.,pitch_deg=0.,speed_mps=1.,front_clearance_m=10.,navigation_distance_m=20.) for _ in self.reports]

    def trigger(self):
        self.own[3].update(speed_mps=.1,front_clearance_m=1.)
        self.team.update(self.reports,self.own,self.track,0.)
        self.team.update(self.reports,self.own,self.track,2.1)

    def view(self,t=2.1,**kwargs):return [dict(time=t,yaw=0.,position=self.reports[3]['position'],safety=EscapeView(**kwargs))]

    def test_support_has_separate_self_rescue_and_teammate_rescue_without_fire(self):
        from duel_tactics import QUESTION
        criteria=self.book.questions(QUESTION,3)['action']['criteria']
        self.assertEqual(len(criteria),7)
        self.assertIn('self_rescue',criteria);self.assertIn('rescue',criteria);self.assertNotIn('fire',criteria)
        self.assertIn('HIGHEST PRIORITY',criteria['self_rescue'])

    def test_previously_deadlocked_support_now_escapes_through_measured_clear_air(self):
        self.trigger();recovery=self.team.self_rescue
        self.assertEqual(self.team.health[3],'stuck');self.assertTrue(recovery.active)
        velocity,_,mode,recover=recovery.command(self.reports[3],self.view(),self.reports[:3],2.1)
        self.assertGreater(np.linalg.norm(velocity),.3);self.assertEqual(mode,'self_rescue_escape');self.assertFalse(recover)
        self.assertLessEqual(np.linalg.norm(velocity),1.2+1e-9)

    def test_normal_support_hover_does_not_trigger_self_rescue(self):
        self.own[3].update(speed_mps=.01,front_clearance_m=1.,navigation_distance_m=.1)
        for t in (0.,5.,10.):self.team.update(self.reports,self.own,self.track,t)
        self.assertFalse(self.team.self_rescue.active)
        self.assertFalse(self.team.self_rescue.request(self.reports[3],self.own[3],'healthy',10.))

    def test_own_righting_pauses_teammate_rescue_then_resumes_after_real_recovery(self):
        self.own[1]['tilt_deg']=80.;self.team.update(self.reports,self.own,self.track,0.)
        self.assertEqual(self.team.assignment,'P-02')
        self.own[3]['tilt_deg']=80.;self.team.update(self.reports,self.own,self.track,1.)
        self.assertIsNone(self.team.assignment)
        v,_,mode,recover=self.team.self_rescue.command(self.reports[3],self.view(t=1.),self.reports[:3],1.)
        np.testing.assert_allclose(v,0);self.assertTrue(recover);self.assertEqual(mode,'self_rescue_righting')
        self.own[3]['tilt_deg']=0.;self.reports[3]['position']+=np.array([2.5,0,0])
        self.team.update(self.reports,self.own,self.track,2.)
        self.assertTrue(self.team.self_rescue.active)
        self.team.update(self.reports,self.own,self.track,3.6)
        self.assertEqual(self.team.self_rescue.completed,1);self.assertEqual(self.team.assignment,'P-02')

    def test_unknown_or_blocked_space_scans_without_translation(self):
        for condition in ('unknown','blocked'):
            self.setUp();self.trigger()
            v,_,mode,_=self.team.self_rescue.command(self.reports[3],self.view(**{condition:True}),self.reports[:3],2.1)
            np.testing.assert_allclose(v,0);self.assertEqual(mode,'self_rescue_scan')
            v,yaw,_,_=self.team.self_rescue.command(self.reports[3],self.view(t=3.,**{condition:True}),self.reports[:3],3.)
            np.testing.assert_allclose(v,0);self.assertGreater(abs(yaw),.1)
            self.own[3].update(front_clearance_m=10.)
            self.team.update(self.reports,self.own,self.track,4.)
            self.assertEqual(self.team.self_rescue.completed,0)

    def test_recovery_needs_displacement_and_sustained_stability(self):
        self.trigger();self.own[3]['front_clearance_m']=10.
        self.team.update(self.reports,self.own,self.track,4.);self.team.update(self.reports,self.own,self.track,6.)
        self.assertTrue(self.team.self_rescue.active);self.assertEqual(self.team.self_rescue.completed,0)
        self.reports[3]['position']+=np.array([2.5,0,0])
        for t in (7.,8.):self.team.update(self.reports,self.own,self.track,t)
        self.assertEqual(self.team.self_rescue.completed,0)
        self.team.update(self.reports,self.own,self.track,8.6)
        self.assertEqual(self.team.self_rescue.completed,1)

    def test_peer_guard_preserves_a_clear_escape_away_from_an_approaching_peer(self):
        self.trigger()
        peer=dict(position=self.reports[3]['position']+np.array([4.2,0.,0]),velocity=np.array([-4.,0,0]))
        views=self.view();views.append({**views[0],'yaw':math.pi})
        velocity,_,_,_=self.team.self_rescue.command(self.reports[3],views,[peer],2.1)
        self.assertGreater(np.linalg.norm(velocity),.3);self.assertLessEqual(velocity[0],0)

    def test_retry_limit_reports_failure_without_claiming_recovery(self):
        self.book.config['self_rescue']['timeout_seconds']=5.
        self.trigger()
        for t in (7.2,9.3,14.4,16.5,21.6):self.team.update(self.reports,self.own,self.track,t)
        state=self.team.self_rescue
        self.assertEqual(state.state,'failed');self.assertEqual(state.attempt,3)
        self.assertEqual(state.failed,3);self.assertEqual(state.completed,0)
        self.assertFalse(state.request(self.reports[3],self.own[3],'stuck',22.))

    def test_safety_blocked_navigation_can_trigger_without_a_close_front_obstacle(self):
        self.own[3].update(speed_mps=.1,safety_limited=True,requested_navigation_speed_mps=3.)
        self.team.update(self.reports,self.own,self.track,0.);self.team.update(self.reports,self.own,self.track,2.1)
        self.assertEqual(self.team.health[3],'healthy');self.assertTrue(self.team.self_rescue.active)


if __name__=='__main__':unittest.main()
