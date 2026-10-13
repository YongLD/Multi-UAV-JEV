"""Native dynamics check: motors and odometry must address each physical body independently."""
import tempfile
from pathlib import Path
import unittest
import numpy as np
import mujoco
import fleet_model
from flight import Pilot


class PhysicsTests(unittest.TestCase):
    def test_four_airframes_hover_with_independent_motor_and_velocity_addresses(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root/'mujoco_menagerie/skydio_x2'; source.mkdir(parents=True)
            (source/'x2.xml').write_text('''<mujoco><compiler autolimits="true"/><option timestep=".002"/>
<worldbody><body name="x2"><freejoint/><geom type="box" size=".15 .2 .06" mass="1.325"/>
<site name="imu"/><site name="thrust1" pos="-.14 -.18 .05"/><site name="thrust2" pos="-.14 .18 .05"/>
<site name="thrust3" pos=".14 .18 .08"/><site name="thrust4" pos=".14 -.18 .08"/></body></worldbody>
<actuator><motor name="thrust1" site="thrust1" gear="0 0 1 0 0 -.0201" ctrlrange="0 13"/>
<motor name="thrust2" site="thrust2" gear="0 0 1 0 0 .0201" ctrlrange="0 13"/>
<motor name="thrust3" site="thrust3" gear="0 0 1 0 0 -.0201" ctrlrange="0 13"/>
<motor name="thrust4" site="thrust4" gear="0 0 1 0 0 .0201" ctrlrange="0 13"/></actuator>
<sensor><gyro name="gyro" site="imu"/></sensor><keyframe><key name="hover"/></keyframe></mujoco>''')
            original_root, original_writer = fleet_model.ROOT, fleet_model.write_city_xml
            fleet_model.ROOT = root
            fleet_model.write_city_xml = lambda path: Path(path).write_text('<mujoco><include file="mujoco_menagerie/skydio_x2/x2.xml"/><worldbody/></mujoco>')
            try:
                for count in (1, 2, 4):
                    model = fleet_model.create_model(count)
                    self.assertEqual((model.nq, model.nv, model.nu), (7*count, 6*count, 4*count))
                data = mujoco.MjData(model)
                pilots = []
                addresses = []
                for i in range(4):
                    body = 'x2' if i == 0 else f'x2_{i+1}'
                    joint = model.body_jntadr[model.body(body).id]
                    q, v = int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint])
                    prefix = '' if i == 0 else body+'__'
                    motors = [model.actuator(prefix+f'thrust{k}').id for k in range(1, 5)]
                    data.qpos[q:q+7] = [i*6., 0., 12.+i*2, 1., 0., 0., 0.]
                    pilots.append((Pilot(model, body, motors), motors))
                    addresses.append((q, v))
                mujoco.mj_forward(model, data)
                for step in range(1000):
                    for pilot, motors in pilots:
                        data.ctrl[motors] = pilot(data, [0., 0., 0.], 0., model.opt.timestep)
                    mujoco.mj_step(model, data)
                for i, (q, v) in enumerate(addresses):
                    self.assertAlmostEqual(float(data.qpos[q+2]), 12.+i*2, delta=.1)
                    self.assertLess(np.linalg.norm(data.qvel[v:v+3]), .1)
            finally:
                fleet_model.ROOT, fleet_model.write_city_xml = original_root, original_writer


if __name__ == '__main__':
    unittest.main()
