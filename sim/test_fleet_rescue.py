"""Regression checks for role loading, final safety and motor-only recovery."""
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
import mujoco
from fleet_roles import RoleBook
from fleet_control import safe_velocity, TeamCoordinator
from fleet_tracking import SharedTargetTrack
from flight import Pilot


def reports():
    return [{'id':f'P-{i+1:02d}','position':np.array([i*8.,0,12.]),'velocity':np.zeros(3),
             'yaw':0.,'alive':True,'target':{'visible':True,'track_confidence':.8,'range_m':20}} for i in range(4)]


class MeasuredView:
    def __init__(self,wall_axis=None,wall_distance=100):self.axis,self.distance=wall_axis,wall_distance
    def query(self,points):
        return np.ones(len(points),dtype=bool), (points[:,self.axis]>self.distance if self.axis is not None else np.zeros(len(points),dtype=bool))


class CooperationTests(unittest.TestCase):
    def test_support_has_rescue_without_fire_and_bad_edit_keeps_previous_rules(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'role.md';path.write_bytes(Path(__file__).with_name('role.md').read_bytes())
            book=RoleBook(path)
            base={'action':{'instructions':{},'criteria':{}}}
            support=book.questions(base,3)['action']
            self.assertNotIn('fire',support['criteria']);self.assertIn('rescue',support['criteria'])
            self.assertIn('communication',support['instructions']['rules'])
            version=book.version;path.write_text('broken',encoding='utf-8');book.reload()
            self.assertEqual(book.version,version);self.assertIsNotNone(book.error)

    def test_final_separation_ray_is_stopped_at_wall_and_stale_camera_stops_motion(self):
        r=reports()[0]
        view={'safety':MeasuredView(1,1.3),'position':r['position'],'yaw':0.,'time':0.}
        result,limited=safe_velocity([0,3,0],r,[view],[],0.)
        np.testing.assert_allclose(result,0.);self.assertTrue(limited)
        result,_=safe_velocity([2,0,0],r,[view],[],.6)
        np.testing.assert_allclose(result,0.)

    def test_camera_odometry_and_peer_closing_limit_final_command(self):
        r=reports()[0];r['velocity']=np.array([4.,0.,0.])
        view={'safety':MeasuredView(),'position':r['position'],'yaw':math.pi/2,'time':0.}
        peer={'position':r['position']+np.array([5.,0.,0.]),'velocity':np.array([-4.,0.,0.])}
        result,_=safe_velocity([4,0,0],r,[view],[peer],0.)
        np.testing.assert_allclose(result,0.)
        view['safety']=MeasuredView(0,2.);r['position']=r['position']+np.array([0.,2.,0.])
        result,_=safe_velocity([0,2,0],r,[view],[],0.)
        np.testing.assert_allclose(result,0.)

    def test_rescue_targets_unstable_peer_and_only_completes_after_stability(self):
        book=RoleBook();team=TeamCoordinator(book,4);r=reports();track=SharedTargetTrack()
        own=[{'tilt_deg':0.,'roll_deg':0.,'pitch_deg':0.,'speed_mps':1.,'front_clearance_m':10.,'navigation_distance_m':20.} for _ in r]
        own[1]['tilt_deg']=80.
        team.update(r,own,track,0.)
        self.assertEqual(team.assignment,'P-02');self.assertNotEqual(team.effective_tracker,'P-02')
        self.assertFalse(team.authorize_rescue(r[0]));self.assertTrue(team.authorize_rescue(r[3]))
        team.delivered=True
        own[1]['tilt_deg']=10.;team.update(r,own,track,1.)
        self.assertEqual(team.rescues_completed,0)
        team.update(r,own,track,2.6)
        self.assertEqual(team.rescues_completed,1);self.assertIsNone(team.assignment)

    def test_recovery_restores_large_roll_with_motor_forces(self):
        model=mujoco.MjModel.from_xml_string('''<mujoco><compiler autolimits="true"/><option timestep=".002"/>
<worldbody><body name="x2"><freejoint/><geom type="box" size=".15 .2 .06" mass="1.325"/>
<site name="t1" pos="-.14 -.18 .05"/><site name="t2" pos="-.14 .18 .05"/>
<site name="t3" pos=".14 .18 .08"/><site name="t4" pos=".14 -.18 .08"/></body></worldbody>
<actuator><motor site="t1" gear="0 0 1 0 0 -.0201" ctrlrange="0 13"/>
<motor site="t2" gear="0 0 1 0 0 .0201" ctrlrange="0 13"/>
<motor site="t3" gear="0 0 1 0 0 -.0201" ctrlrange="0 13"/>
<motor site="t4" gear="0 0 1 0 0 .0201" ctrlrange="0 13"/></actuator></mujoco>''')
        data=mujoco.MjData(model);data.qpos[:3]=[0,0,15]
        data.qpos[3:7]=[math.cos(math.radians(40)),math.sin(math.radians(40)),0,0]
        pilot=Pilot(model);mujoco.mj_forward(model,data)
        for step in range(2000):
            pilot.recovery_requested=step<300
            data.ctrl[:]=pilot(data,[0,0,0],0,model.opt.timestep)
            mujoco.mj_step(model,data)
        tilt=math.degrees(math.acos(float(np.clip(data.xmat[pilot.body_id].reshape(3,3)[2,2],-1,1))))
        self.assertLess(tilt,25.);self.assertGreater(data.qpos[2],1.)


if __name__=='__main__':unittest.main()
