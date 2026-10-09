"""One camera-guided chaser versus one scripted evader in an open MuJoCo city."""
from __future__ import annotations

import argparse
import io
import json
import math
import os
import time
from collections import Counter, deque
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
import imageio.v2 as imageio

from city_layout import BUILDINGS, RAIN_RADIUS_M, write_city_xml
from duel_hud import DuelHud
from duel_tactics import ACTIONS, TURN_DEG, QUESTION, POLICY_VERSION, POLICY_MODE, JevWorker, build_state
from flight import Eye, Pilot
from camera_controls import DEFAULT_CAMERA, validate_camera
from onboard_safety import DepthSafety, BRAKE_ACCEL, MARGIN
from onboard_target import CameraTargetTrack, aim_at_execution, pursuit_point_at_execution, rotate as rotate_target

ROOT = Path(__file__).resolve().parent
ROUTE = [(27, 0), (45, 0), (45, 45), (0, 45), (-45, 45), (-45, 0),
         (0, 0), (0, -45), (45, -45), (45, 0), (0, 0), (-45, 0)]
REPLAY_FPS = 20
SHOT_RANGE_M = 25.0
SHOT_HEIGHT_M = 1.0
SHOT_BEARING_DEG = 40.0
SHOT_COOLDOWN_S = 2.0


class DuelEye(Eye):
    # A small aircraft at tens of metres needs more than the base rover's 96x72 pixels.
    W, H = 192, 144
    MAX_RANGE = 70.0
    TARGET_MIN_PIXELS = 1
    TARGET_RANGE_OFFSET_M = 1.3  # visible halo radius around the identical X2 mesh
    TILT_DEG = 0.0  # Balanced view above/below the aircraft's flight layer.

    def _elevation(self, rows):
        # Exact pinhole calibration; the old linear row approximation and
        # empirical scale could label a lower image band as unobserved/clear.
        half = (self.H-1)/2
        return np.arctan((half-np.asarray(rows))/half*np.tan(np.deg2rad(self.EYE_FOVY/2)))+np.deg2rad(self.TILT_DEG)

    def look(self, data, pos, yaw, t):
        scene = super().look(data, pos, yaw, t)
        # Visual rain disk is segmented by the same onboard camera, not looked
        # up from its simulator position.
        rain_pixels = int(np.count_nonzero(self.last_segmentation
                                           == self.m.geom("rain_visual").id))
        scene["rain_visible"] = rain_pixels >= 3
        if not hasattr(self, 'safety'):
            self.safety = DepthSafety(self.EYE_FOVY, self.TILT_DEG, self.NOSE_OFFSET_M, self.MAX_RANGE)
        ignored = np.concatenate([self.target_ids, self.self_geoms, self.ignore_ids])
        self.safety.observe(self.last_depth, self.last_segmentation, ignored)
        scene['observed_clearance'] = {label: self.safety.grid[label][3]['clearance_m']
                                       for label in ('above', 'level', 'below')}
        if not hasattr(self, 'target_track'):
            self.target_track = CameraTargetTrack(self.EYE_FOVY, self.TILT_DEG, self.NOSE_OFFSET_M,
                                                 self.TARGET_RANGE_OFFSET_M, self.MAX_RANGE)
        # Semantic target marker pixels are the detector mask; no enemy pose is
        # read here. Own odometry compensates camera translation and rotation.
        target_mask = self.last_segmentation == self.m.geom('enemy_camera_halo').id
        depth, mask = self.last_depth_raw, target_mask
        sensor, offset = 'front', 0.
        if not hasattr(self, 'peripheral'):
            self.peripheral = {}
            self.last_target_camera = 'left'
        if self.target_track.measure(depth, mask) is None:
            cameras = {'left': math.pi/2, 'right': -math.pi/2, 'rear': math.pi}
            order = [self.last_target_camera]+[c for c in cameras if c != self.last_target_camera]
            for name in order:
                camera_offset = cameras[name]
                self._aim(pos, yaw+camera_offset)
                self.depth.update_scene(data, self.cam)
                candidate_depth = self.depth.render()
                self.seg.update_scene(data, self.cam)
                candidate_seg = self.seg.render()[:, :, 0]
                candidate_mask = candidate_seg == self.m.geom('enemy_camera_halo').id
                # Peripheral clearance summaries are also measured, never a map lookup.
                camera_safety = DepthSafety(self.EYE_FOVY, self.TILT_DEG, self.NOSE_OFFSET_M, self.MAX_RANGE)
                camera_safety.observe(np.clip(candidate_depth, 0, self.MAX_RANGE), candidate_seg, ignored)
                self.peripheral[name] = {'age_s': 0., 'observed_at_s': t,
                    'ahead_above_m': camera_safety.grid['above'][3]['clearance_m'],
                    'ahead_level_m': camera_safety.grid['level'][3]['clearance_m'],
                    'ahead_below_m': camera_safety.grid['below'][3]['clearance_m']}
                if self.target_track.measure(candidate_depth, candidate_mask) is not None:
                    depth, mask, sensor, offset = candidate_depth, candidate_mask, name, camera_offset
                    self.last_target_camera = name
                    break
        scene['target'] = self.target_track.update(depth, mask, pos, yaw, t,
                                                   camera_yaw=yaw+offset, camera=sensor)
        self.sensor_depth = np.clip(depth, 0, self.MAX_RANGE)
        self.sensor_offset = offset
        self._aim(pos, yaw+offset)
        scene['peripheral_space'] = {name: {k: round(t-values['observed_at_s'], 2) if k == 'age_s' else v
                                         for k, v in values.items() if k != 'observed_at_s'}
                                    for name, values in self.peripheral.items()
                                    if t-values['observed_at_s'] <= .5}
        return scene


def angle(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def attitude(q, omega):
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1-2*(x*x+y*y))
    pitch = math.asin(max(-1, min(1, 2*(w*y-z*x))))
    yaw_rate = (math.sin(roll)*omega[1]+math.cos(roll)*omega[2])/max(math.cos(pitch), .05)
    return round(math.degrees(roll), 1), round(math.degrees(pitch), 1), round(math.degrees(yaw_rate), 1)


def publish(path, obj):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj, ensure_ascii=False), encoding="utf-8")
    tmp.replace(path)


def retain_best_replay(live_dir, video_tmp, summary):
    """Keep a replay with >=5 hits and >=the retained hit count; accept ties."""
    video = live_dir/"replay.mp4"
    metadata = live_dir/"replay_summary.json"
    best_hits = 0
    if video.is_file() and metadata.is_file():
        try:
            best_hits = int(json.loads(metadata.read_text(encoding="utf-8"))["hits"])
        except (ValueError, KeyError, TypeError):
            pass
    if best_hits < 5:
        video.unlink(missing_ok=True)
        metadata.unlink(missing_ok=True)
        best_hits = 0
    if summary["hits"] >= 5 and summary["hits"] >= best_hits:
        video_tmp.replace(video)
        publish(metadata, summary)
        return True
    video_tmp.unlink(missing_ok=True)
    return False


def camera_blocked(aircraft, camera_pos, buildings=BUILDINGS, *, model=None, data=None):
    """Referee-only camera placement: avoid putting the viewer behind a building."""
    start = np.asarray(aircraft, dtype=float)
    direction = np.asarray(camera_pos, dtype=float)-start
    if model is not None:
        length = float(np.linalg.norm(direction))
        if length < .1:
            return False
        # Urban group 1 contains the visible buildings/decor. Hidden collision
        # hulls and broad building boxes must not force a needless close-up.
        distance = mujoco.mj_ray(model, data, start, direction/length,
                                 np.array([0, 1, 0, 0, 0, 0], dtype=np.uint8),
                                 True, model.body('x2').id, None)
        return 0 <= distance < length-.1
    for b in buildings:
        bounds = ((b["x"]-b["half_x"]-.2, b["x"]+b["half_x"]+.2),
                  (b["y"]-b["half_y"]-.2, b["y"]+b["half_y"]+.2),
                  (0., b["height"]+.2))
        lo, hi = 0., 1.
        for axis, (lower, upper) in enumerate(bounds):
            if abs(direction[axis]) < 1e-8:
                if not lower <= start[axis] <= upper:
                    lo, hi = 2., -1.
                    break
            else:
                a, c = (lower-start[axis])/direction[axis], (upper-start[axis])/direction[axis]
                lo, hi = max(lo, min(a, c)), min(hi, max(a, c))
        if lo <= hi and hi > .03 and lo < .97:
            return True
    return False


class Evader:
    """Scenario-owned trajectory; its world position never enters build_state."""
    def __init__(self, seed=0):
        rng = np.random.default_rng(seed)
        self.pos = np.array([rng.uniform(26., 30.), rng.uniform(-3., 3.),
                             rng.uniform(12.2, 13.2)])
        self.yaw = 0.0
        self.speed = 4.6
        self.cruise_speed = 4.6
        self.velocity = np.array([4.6, 0., 0.])
        self.index = 1

    def step(self, t, dt, rain_xy):
        goal = np.array(ROUTE[self.index], dtype=float)
        if np.linalg.norm(goal - self.pos[:2]) < 7:
            self.index = (self.index + 1) % len(ROUTE)
            goal = np.array(ROUTE[self.index], dtype=float)
        desired = math.atan2(goal[1]-self.pos[1], goal[0]-self.pos[0])
        self.yaw += np.clip(angle(desired-self.yaw), -math.radians(52)*dt, math.radians(52)*dt)
        target_speed = 4.9 + .3*math.sin(.18*t+.4)
        slowdown = .68 if np.linalg.norm(self.pos[:2]-rain_xy) < RAIN_RADIUS_M else 1.
        # Independent nonzero cruise: the chaser's braking must not stop the
        # scenario target. Rain is the only flight-speed scale here.
        self.cruise_speed += float(np.clip(target_speed-self.cruise_speed, -dt, dt))
        horizontal_speed = self.cruise_speed*slowdown
        target_z = 14. + 3.4*math.sin(.11*t) + 1.2*math.sin(.32*t)
        self.velocity = np.array([horizontal_speed*math.cos(self.yaw),
                                  horizontal_speed*math.sin(self.yaw),
                                  np.clip(target_z-self.pos[2], -.8, .8)])
        self.speed = float(np.linalg.norm(self.velocity[:2]))
        self.pos += self.velocity*dt


class ActionController:
    """Only converts Jev's selected action into stable motor-controller setpoints."""
    def __init__(self):
        self.heading = 0.
        self.altitude = 12.
        self.speed = 0.
        self.last_action = "forward"
        self.last_shot_t = -100.
        self.hits = self.shots = 0

    def apply(self, action, yaw, t, own_pos, enemy_pos):
        if action not in ACTIONS:
            raise ValueError(action)
        self.last_action = action
        if action == "turn_left":
            self.heading = yaw + math.radians(TURN_DEG)
        elif action == "turn_right":
            self.heading = yaw - math.radians(TURN_DEG)
        elif action == "climb":
            self.altitude = min(36., float(own_pos[2]) + 2.)
        elif action == "descend":
            self.altitude = max(3., float(own_pos[2]) - 2.)
        elif action == "forward":
            # Explicit current-pose straight flight, not continuation of the
            # previous maneuver. Inertial rates settle through the motor loop.
            self.heading = yaw
            self.altitude = float(own_pos[2])
        if action != "fire":
            return None
        self.shots += 1
        delta = enemy_pos - own_pos
        horizontal_bearing = angle(math.atan2(delta[1], delta[0])-yaw)
        distance = float(np.linalg.norm(delta))
        height = abs(float(delta[2]))
        cooldown_ok = t - self.last_shot_t >= SHOT_COOLDOWN_S
        hit = (distance <= SHOT_RANGE_M and height <= SHOT_HEIGHT_M
               and abs(horizontal_bearing) <= math.radians(SHOT_BEARING_DEG) and cooldown_ok)
        if cooldown_ok:
            self.last_shot_t = t
        if hit:
            self.hits += 1
        return {"distance_m": round(distance, 2), "height_gap_m": round(height, 2),
                "bearing_deg": round(math.degrees(horizontal_bearing), 1),
                "cooldown_ok": cooldown_ok, "hit": bool(hit)}

    def commands(self, yaw, z, rain_factor, scene):
        # Pilot owns motor dynamics; passing a tiny yaw error here caused only ~2 deg/s
        # of actual rotation and made repeated Jev turns visually ineffective.
        yaw_cmd = self.heading
        # Camera-derived speed governor, like the upstream guidance layer.
        # Jev still picks every heading, altitude and fire action. Slowing does
        # not pick a different maneuver; it gives that maneuver time to work.
        target = scene["target"] if scene else {}
        distance = target.get("range_m") if target.get("visible") else None
        cruise = 6.3
        if distance is not None and target.get('range_rate_mps') is not None:
            body_velocity = scene.get('own_velocity_body_mps', [0., 0., 0.])
            bearing = math.radians(target['bearing_deg'])
            elevation = math.radians(target['elevation_deg'])
            own_radial_speed = ((body_velocity[0]*math.cos(bearing)+body_velocity[1]*math.sin(bearing))*math.cos(elevation)
                                +body_velocity[2]*math.sin(elevation))
            estimated_target_radial_speed = own_radial_speed+target['range_rate_mps']
            # Match observed target motion near 15m, close when farther, slow
            # when too close. No scripted enemy speed enters this estimate.
            cruise = float(np.clip(estimated_target_radial_speed+.5*(distance-15.), 0., 6.3))
        front = scene['observed_clearance']['level'] if scene else None
        flow = scene.get('velocity_path') if scene else None
        # The existing speed governor now includes actual translation direction
        # and delay + braking distance. It never substitutes another Jev action.
        clearances = [c for c in (front, flow.get('clearance_m') if flow else None) if c is not None]
        usable = max(min(clearances)-MARGIN, 0.) if clearances else 0.
        reaction = scene.get('reaction_budget_s', .616) if scene else .616
        clearance_speed = math.sqrt((BRAKE_ACCEL*reaction)**2+2*BRAKE_ACCEL*usable)-BRAKE_ACCEL*reaction
        if front is None or (flow and flow['status'] == 'unknown'):
            clearance_speed = min(clearance_speed, 1.5)
        self.speed = min(cruise, clearance_speed)*rain_factor
        speed = self.speed
        vz = float(np.clip(1.1*(self.altitude-z), -2.0, 2.0))
        return np.array([speed*math.cos(yaw), speed*math.sin(yaw), vz]), yaw_cmd


def run(seconds, seed, live_dir, endpoint, realtime=True, urban=False):
    if urban:
        from urban_scene import write_city_xml as write_urban_xml, BUILDINGS as view_buildings
        from urban_hud import UrbanHud, decorate_overview
        scene_path = ROOT/"city_urban.xml"
        write_urban_xml(scene_path)
    else:
        scene_path = ROOT/"city.xml"
        view_buildings = BUILDINGS
        write_city_xml(scene_path)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    data.qpos[:3] = [0., 0., 12.]
    data.qpos[3:7] = [1., 0., 0., 0.]
    enemy = Evader(seed)
    enemy_id = model.body("enemy").mocapid[0]
    rain_id = model.body("rain").mocapid[0]
    data.mocap_pos[enemy_id] = enemy.pos
    mujoco.mj_forward(model, data)
    eye = DuelEye(model, target_body="enemy", ignore_geoms=("rain_visual",))
    pilot = Pilot(model)
    action_ctrl = ActionController()
    worker = JevWorker(endpoint)
    live_dir.mkdir(parents=True, exist_ok=True)
    publish(live_dir/'policy.json', {'policy_version': POLICY_VERSION, 'questions': QUESTION,
                                   'actions': list(ACTIONS), 'turn_deg': TURN_DEG,
                                   'policy_mode': POLICY_MODE, 'target_cameras': ['front', 'left', 'right', 'rear'],
                                   'model_endpoint': endpoint,
                                   'model_name': worker.model, 'model_backend': worker.backend,
                                   'model_input_mode': worker.input_mode,
                                   'evader_cruise_mps': [4.6, 5.2], 'evader_rain_factor': .68,
                                   'evader_speed_control': 'independent_nonzero_cruise'})
    # Old versions stored the latest episode even when it had no hits.
    old_replay = live_dir/"replay.mp4"
    old_metadata = live_dir/"replay_summary.json"
    reset_replay = live_dir/"replay_reset.request"
    if reset_replay.exists():
        old_replay.unlink(missing_ok=True)
        old_metadata.unlink(missing_ok=True)
        reset_replay.unlink(missing_ok=True)
    try:
        old_hits = int(json.loads(old_metadata.read_text(encoding="utf-8"))["hits"])
    except (FileNotFoundError, ValueError, KeyError, TypeError):
        old_hits = 0
    if old_hits < 5 or not old_replay.is_file():
        old_replay.unlink(missing_ok=True)
        old_metadata.unlink(missing_ok=True)
    restart_request = live_dir/"restart.request"
    restart_request.unlink(missing_ok=True)
    records = live_dir/"decisions.jsonl"
    events = live_dir/"events.jsonl"
    referee = live_dir/"referee.jsonl"
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = 3.8
    camera.elevation = -13
    view_w, view_h = (1280, 720) if urban else (1180, 880)
    # Segmentation ID colors must not be blended at mesh edges: after importing
    # hundreds of geoms, MSAA can interpolate an ID outside the scene's ID table.
    # Eye's sensor render contexts were created with offsamples=0. The spectator
    # cameras keep MSAA for smooth visual edges in the live view and replay.
    if urban:
        model.vis.quality.offsamples = 4
    renderer = mujoco.Renderer(model, view_h, view_w)
    hud = UrbanHud(view_w, view_h) if urban else DuelHud(view_w, view_h)
    if urban:
        rgb_renderer = mujoco.Renderer(model, eye.H, eye.W)
        top_renderer = mujoco.Renderer(model, 288, 448)
        top_camera = mujoco.MjvCamera()
        top_camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        top_camera.lookat[:] = [0, 0, 0]
        top_camera.distance = 250
        top_camera.azimuth = 90
        top_camera.elevation = -89.9
        overview = None
        next_top = 0.
        trails = (deque(maxlen=500), deque(maxlen=500))
    distance_flown = 0.
    previous_pos = data.qpos[:3].copy()
    video_tmp = live_dir/"replay_next.mp4"
    writer = imageio.get_writer(str(video_tmp), format="FFMPEG", fps=REPLAY_FPS,
                                codec="libx264", quality=7, macro_block_size=1)
    counts = Counter({a: 0 for a in ACTIONS})
    scene = None
    last_decision = None
    latest_navigation = None
    visual_shot = None
    collisions = 0
    contacts = set()
    next_frame = 0.
    began = time.monotonic()
    output = None
    with (records.open("w", encoding="utf-8") as decision_log,
          events.open("w", encoding="utf-8") as event_log,
          referee.open("w", encoding="utf-8") as referee_log):
        for i in range(int(seconds/model.opt.timestep)):
            t = i*model.opt.timestep
            if i % 10 == 0 and restart_request.exists():
                restart_request.unlink(missing_ok=True)
                output = "manual_restart"
                break
            if realtime:
                delay = t - (time.monotonic()-began)
                if delay > 0.001:
                    time.sleep(delay)
            rain_xy = np.array([-12+16*math.sin(t/24), 28+18*math.sin(t/31)])
            data.mocap_pos[rain_id] = [rain_xy[0], rain_xy[1], 0]
            if i % 33 == 0:
                mujoco.mj_forward(model, data)
                pos = data.qpos[:3].copy()
                yaw = yaw_of(data.qpos[3:7])
                scene = eye.look(data, pos, yaw, t)
                roll, pitch, yaw_rate = attitude(data.qpos[3:7], data.qvel[3:6])
                own = {"speed_mps": round(float(np.linalg.norm(data.qvel[:3])), 2),
                       "horizontal_speed_mps": round(float(np.linalg.norm(data.qvel[:2])), 2),
                       "velocity_body_mps": [round(float(data.qvel[0]*math.cos(yaw)+data.qvel[1]*math.sin(yaw)), 2),
                                             round(float(-data.qvel[0]*math.sin(yaw)+data.qvel[1]*math.cos(yaw)), 2),
                                             round(float(data.qvel[2]), 2)],
                       "last_decision_latency_s": last_decision['latency_s'] if last_decision else .4,
                       "last_decision_age_s": last_decision.get('age_s', .3) if last_decision else .3,
                       "altitude_m": round(float(pos[2]), 2), "roll_deg": roll,
                       "pitch_deg": pitch, "yaw_deg": round(math.degrees(yaw), 1),
                       "yaw_rate_deg_s": yaw_rate,
                       "heading_setpoint_deg": round(math.degrees(angle(action_ctrl.heading)), 1),
                       "heading_error_deg": round(math.degrees(angle(action_ctrl.heading-yaw)), 1),
                       "speed_setpoint_mps": action_ctrl.speed,
                       "altitude_setpoint_m": round(action_ctrl.altitude, 2),
                       "altitude_error_m": round(action_ctrl.altitude-float(pos[2]), 2),
                       "turn_in_progress": abs(angle(action_ctrl.heading-yaw)) > math.radians(3),
                       "vertical_in_progress": abs(action_ctrl.altitude-float(pos[2])) > .3,
                       "last_action": action_ctrl.last_action,
                       "shot_cooldown_remaining_s": round(max(0., SHOT_COOLDOWN_S-(t-action_ctrl.last_shot_t)), 2)}
                flow_bearing = math.atan2(own['velocity_body_mps'][1], own['velocity_body_mps'][0]) if own['horizontal_speed_mps'] > .2 else 0.
                scene['velocity_path'] = eye.safety.corridor(flow_bearing)
                scene['reaction_budget_s'] = max(.3, own['last_decision_latency_s'])+.216
                scene['own_velocity_body_mps'] = own['velocity_body_mps']
                def current_state():
                    nonlocal latest_navigation
                    latest_navigation = eye.safety.describe(own, ACTIONS)
                    scene['navigation'] = latest_navigation
                    tracked = scene['target']['visible'] or scene['target'].get('track_confidence', 0) > .3
                    scene['target']['aim'] = (aim_at_execution(scene['target'], own['yaw_rate_deg_s'], own['last_decision_age_s'], own['heading_error_deg'])
                                              if tracked else None)
                    scene['target']['lead'] = (pursuit_point_at_execution(scene['target'], own['velocity_body_mps'],
                                                own['yaw_rate_deg_s'], own['last_decision_age_s'], own['heading_error_deg'])
                                               if tracked else None)
                    return build_state(scene, own)
                worker.offer(current_state, t)
            for result in worker.drain():
                age = t-result["sim_time"]
                result["applied_sim_time"] = round(t, 3)
                result["age_s"] = round(age, 3)
                if "action" in result and age <= 1.2:
                    pos = data.qpos[:3].copy()
                    yaw = yaw_of(data.qpos[3:7])
                    shot = action_ctrl.apply(result["action"], yaw, t, pos, enemy.pos)
                    result["applied"] = True
                    result["executed_action"] = action_ctrl.last_action
                    result["shot"] = shot
                    counts[result["action"]] += 1
                    last_decision = result
                    if shot:
                        start = pos + .7*np.array([math.cos(yaw), math.sin(yaw), 0.])
                        end = enemy.pos.copy() if shot["hit"] else start + SHOT_RANGE_M*np.array(
                            [math.cos(yaw), math.sin(yaw), 0.])
                        visual_shot = {"time_s": t, "start": start, "end": end,
                                       "number": action_ctrl.shots, **shot}
                        event_log.write(json.dumps({"time_s": round(t, 3), "type": "shot", **shot})+"\n")
                        event_log.flush()
                else:
                    result["applied"] = False
                    result["reason"] = ('error' if 'error' in result else 'model abstained'
                                        if result.get('abstained') else 'stale observation')
                decision_log.write(json.dumps(result, ensure_ascii=False)+"\n")
                decision_log.flush()
            if i % 10 == 0:
                pos = data.qpos[:3].copy()
                yaw = yaw_of(data.qpos[3:7])
                rain_factor = .68 if np.linalg.norm(pos[:2]-rain_xy) < RAIN_RADIUS_M else 1.
                v_des, yaw_cmd = action_ctrl.commands(yaw, pos[2], rain_factor, scene)
            data.ctrl[:] = pilot(data, v_des, yaw_cmd, model.opt.timestep)
            mujoco.mj_step(model, data)
            enemy.step(t, model.opt.timestep, rain_xy)
            data.mocap_pos[enemy_id] = enemy.pos
            data.mocap_quat[enemy_id] = [math.cos(enemy.yaw/2), 0, 0, math.sin(enemy.yaw/2)]
            if i % 250 == 0:
                referee_log.write(json.dumps({
                    "time_s": round(t, 3), "chaser_xyz_m": np.round(data.qpos[:3], 3).tolist(),
                    "chaser_yaw_deg": round(math.degrees(yaw_of(data.qpos[3:7])), 2),
                    "enemy_xyz_m": np.round(enemy.pos, 3).tolist(),
                    "enemy_yaw_deg": round(math.degrees(enemy.yaw), 2),
                    "enemy_speed_mps": round(enemy.speed, 2),
                    "enemy_velocity_mps": enemy.velocity.tolist(),
                    "chaser_velocity_mps": data.qvel[:3].tolist(),
                    "enemy_total_speed_mps": float(np.linalg.norm(enemy.velocity)),
                    "chaser_total_speed_mps": float(np.linalg.norm(data.qvel[:3])),
                    "evader_speed_control": 'independent_nonzero_cruise',
                    "rain_xy_m": np.round(rain_xy, 2).tolist(),
                })+"\n")
                referee_log.flush()
            for c in range(data.ncon):
                g1, g2 = data.contact[c].geom1, data.contact[c].geom2
                b1, b2 = model.geom_bodyid[g1], model.geom_bodyid[g2]
                if (b1 == model.body("x2").id) != (b2 == model.body("x2").id):
                    pair = tuple(sorted((int(g1), int(g2))))
                    if pair not in contacts:
                        contacts.add(pair)
                        collisions += 1
                        event_log.write(json.dumps({"time_s": round(t, 3), "type": "collision",
                                                     "geoms": pair})+"\n")
                        event_log.flush()
            if t >= next_frame and scene:
                mujoco.mj_forward(model, data)
                pos = data.qpos[:3].copy()
                distance_flown += float(np.linalg.norm(pos-previous_pos))
                previous_pos = pos.copy()
                try:
                    camera_settings = validate_camera(json.loads((live_dir/"camera.json").read_text()))
                except (OSError, ValueError, TypeError):
                    camera_settings = dict(DEFAULT_CAMERA)
                    if urban:
                        camera_settings.update(distance=5.8, elevation=-14.)
                camera.lookat[:] = pos
                current_yaw = math.degrees(yaw_of(data.qpos[3:7]))
                camera.distance = camera_settings["distance"]
                camera.elevation = -89. if camera_settings["mode"] == "top" else camera_settings["elevation"]
                camera.azimuth = (current_yaw+camera_settings["azimuth"]
                                  if camera_settings["mode"] == "follow" else camera_settings["azimuth"])
                model.vis.global_.fovy = 50
                renderer.update_scene(data, camera)
                camera_geometry = {'model': model, 'data': data} if urban else {}
                if camera_settings["mode"] == "follow" and camera_blocked(pos, renderer._scene.camera[0].pos, view_buildings, **camera_geometry):
                    found_view = False
                    # Keep the viewing direction tied to the aircraft heading.
                    # Resolve obstruction by moving closer/higher along that
                    # heading, never by persisting an unrelated world angle.
                    for distance in (camera_settings['distance']*.75,
                                     camera_settings['distance']*.5, 2.):
                        for elevation in (camera_settings['elevation'], -30., -45., -65.):
                            camera.distance = max(2., distance)
                            camera.elevation = elevation
                            renderer.update_scene(data, camera)
                            if not camera_blocked(pos, renderer._scene.camera[0].pos, view_buildings, **camera_geometry):
                                found_view = True
                                break
                        if found_view:
                            break
                    if not found_view:
                        camera.distance = 2.
                        camera.elevation = camera_settings['elevation']
                        renderer.update_scene(data, camera)
                # The displayed FPV/depth pair is always the forward sensor.
                # Peripheral sensors keep contributing observations to Jev.
                eye._aim(pos, yaw_of(data.qpos[3:7]))
                depth = np.clip(eye.last_depth, 0, 35.)
                cam = renderer._scene.camera[0]
                forward = np.asarray(cam.forward)
                up = np.asarray(cam.up)
                right = np.cross(forward, up)
                relative = enemy.pos-np.asarray(cam.pos)
                camera_range = float(relative@forward)
                enemy_xy = None
                if scene["target"]["visible"] and camera_range > .1:
                    focal = view_h/(2*math.tan(math.radians(25)))
                    enemy_xy = (view_w/2+float(relative@right)/camera_range*focal,
                                view_h/2-float(relative@up)/camera_range*focal)
                roll, pitch, yaw_rate = attitude(data.qpos[3:7], data.qvel[3:6])
                status = {"running": True, "time_s": round(t, 2), "hits": action_ctrl.hits,
                          "shots": action_ctrl.shots, "action": action_ctrl.last_action,
                          "speed_mps": round(float(np.linalg.norm(data.qvel[:3])), 2),
                          "altitude_m": round(float(pos[2]), 2), "calls": worker.calls,
                          "errors": worker.errors, "last_error": worker.last_error,
                          "rain_factor": rain_factor, "target": scene["target"],
                          "counts": dict(counts), "collisions": collisions,
                          "seed": seed, "version": "urban" if urban else "classic",
                          "policy_version": POLICY_VERSION,
                          "policy_mode": POLICY_MODE,
                          "model_endpoint": endpoint,
                          "model_name": worker.model, "model_backend": worker.backend,
                          "model_input_mode": worker.input_mode, "model_abstentions": worker.abstentions,
                          "scene_version": 'urban_observable_cruise_v4' if urban else 'city_box_v1',
                          "enemy_speed_mps": round(enemy.speed, 2),
                          "enemy_total_speed_mps": round(float(np.linalg.norm(enemy.velocity)), 2),
                          "evader_speed_control": 'independent_nonzero_cruise',
                          "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": current_yaw,
                          "vertical_speed_mps": round(float(data.qvel[2]), 2),
                          "distance_flown_m": round(distance_flown, 1),
                          "free_space": {"horizontal": scene["sector_range_m"],
                                         "above": scene["observed_clearance"]["above"],
                                         "level": scene["observed_clearance"]["level"],
                                         "below": scene["observed_clearance"]["below"]},
                          "navigation": latest_navigation,
                          "rain_visible_from_camera": bool(scene.get("rain_visible", False)),
                          "viewer_camera": {"mode": camera_settings["mode"], "azimuth": camera.azimuth,
                                            "elevation": camera.elevation, "distance": camera.distance,
                                            "forward_heading_deg": round(math.degrees(math.atan2(forward[1], forward[0])), 2),
                                            "heading_offset_deg": camera_settings['azimuth'] if camera_settings['mode']=='follow' else None},
                          "display_camera": "front",
                          "last_decision": {k: last_decision[k] for k in ("action", "probabilities", "latency_s")}
                          if last_decision else None}
                telemetry = {**status, "tan_h": eye.tan_h,
                             "latency_s": last_decision["latency_s"] if last_decision else 0.}
                def project_world(point):
                    relative = np.asarray(point)-np.asarray(cam.pos)
                    distance = float(relative@forward)
                    if distance <= .1:
                        return None
                    focal = view_h/(2*math.tan(math.radians(25)))
                    return (view_w/2+float(relative@right)/distance*focal,
                            view_h/2-float(relative@up)/distance*focal)
                trajectory = scene['target'].get('trajectory')
                if urban and trajectory and eye.target_track.previous_own:
                    anchor_pos, anchor_yaw = eye.target_track.previous_own[1:]
                    history_xy = [project_world(anchor_pos+rotate_target(p['relative_xyz_m'], anchor_yaw))
                                  for p in trajectory['history']]
                    predicted_xy = [{'xy': project_world(anchor_pos+rotate_target(p['relative_xyz_m'], anchor_yaw)),
                                     'horizon_s': p['horizon_s'], 'confidence': p['confidence']}
                                    for p in trajectory['predictions']]
                    telemetry['trajectory_overlay'] = {'history_xy': history_xy, 'predicted_xy': predicted_xy}
                if visual_shot and t-visual_shot["time_s"] <= .7:
                    def project(point):
                        relative = np.asarray(point)-np.asarray(cam.pos)
                        distance = float(relative@forward)
                        if distance <= .1:
                            return None
                        focal = view_h/(2*math.tan(math.radians(25)))
                        return (view_w/2+float(relative@right)/distance*focal,
                                view_h/2-float(relative@up)/distance*focal)
                    telemetry["shot_effect"] = {
                        "age_s": t-visual_shot["time_s"], "number": visual_shot["number"],
                        "hit": visual_shot["hit"], "cooldown_ok": visual_shot["cooldown_ok"],
                        "start_xy": project(visual_shot["start"]),
                        "end_xy": project(visual_shot["end"])}
                view = renderer.render()
                if urban:
                    rgb_renderer.update_scene(data, eye.cam)
                    rgb = rgb_renderer.render()
                    if t >= next_top:
                        trails[0].append(pos.copy())
                        trails[1].append(enemy.pos.copy())
                        model.vis.global_.fovy = 48
                        top_renderer.update_scene(data, top_camera)
                        overview = decorate_overview(top_renderer.render(), top_renderer._scene.camera[0],
                            pos, enemy.pos, yaw_of(data.qpos[3:7]), enemy.yaw, trails, rain_xy)
                        next_top = t+.2
                        save_jpeg(live_dir/"overview.jpg", overview, 84)
                    save_jpeg(live_dir/"rgb.jpg", rgb, 90)
                    from hud import _depth_rgb
                    save_jpeg(live_dir/"depth.jpg", _depth_rgb(depth, 35.), 92)
                    clean_view = np.asarray(hud.decorate(view, telemetry, enemy_xy).convert("RGB"))
                    save_jpeg(live_dir/"scene.jpg", clean_view, 85)
                    image = hud.draw(view, depth, scene, last_decision or {}, telemetry,
                                     enemy_xy, rgb=rgb, overview=overview)
                else:
                    image = hud.draw(view, depth, scene, last_decision or {}, telemetry, enemy_xy)
                writer.append_data(image)
                memory = io.BytesIO()
                Image.fromarray(image).save(memory, format="JPEG", quality=80)
                tmp = live_dir/"frame.jpg.tmp"
                tmp.write_bytes(memory.getvalue())
                tmp.replace(live_dir/"frame.jpg")
                publish(live_dir/"status.json", status)
                next_frame = t+1/REPLAY_FPS
            if action_ctrl.hits >= 10:
                output = "success"
                break
            if data.qpos[2] < .2:
                output = "crash"
                break
        if output is None:
            output = "timeout"
        # Finish the in-flight request while the log is still open. Keep its
        # returned choice as evidence without applying a maneuver after the end.
        worker.close()
        for result in worker.drain():
            result["applied"] = False
            result["reason"] = "episode ended"
            result["applied_sim_time"] = round(t, 3)
            result["age_s"] = round(t-result["sim_time"], 3)
            decision_log.write(json.dumps(result, ensure_ascii=False)+"\n")
        decision_log.flush()
    writer.close()
    renderer.close()
    eye.depth.close()
    eye.seg.close()
    if urban:
        rgb_renderer.close()
        top_renderer.close()
    summary = {"outcome": output, "time_s": round(t, 2), "hits": action_ctrl.hits,
               "shots": action_ctrl.shots, "collisions": collisions, "action_counts": dict(counts),
               "jev_calls": worker.calls, "jev_errors": worker.errors,
               "records": str(records), "events": str(events), "referee": str(referee),
               "seed": seed, "version": "urban" if urban else "classic", "policy_version": POLICY_VERSION,
               "policy_mode": POLICY_MODE,
               "model_endpoint": endpoint,
               "model_name": worker.model, "model_backend": worker.backend,
               "model_input_mode": worker.input_mode, "model_abstentions": worker.abstentions,
               "scene_version": 'urban_observable_cruise_v4' if urban else 'city_box_v1',
               "evader_cruise_mps": [4.6, 5.2], "evader_rain_factor": .68,
               "evader_speed_control": 'independent_nonzero_cruise'}
    summary["replay_retained"] = retain_best_replay(live_dir, video_tmp, summary)
    publish(live_dir/"summary.json", summary)
    status["running"] = False
    status["summary"] = summary
    publish(live_dir/"status.json", status)
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    return summary


def save_jpeg(path, pixels, quality=85):
    memory = io.BytesIO()
    Image.fromarray(pixels).save(memory, format="JPEG", quality=quality)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_bytes(memory.getvalue())
    temporary.replace(path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=120)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--live-dir", type=Path, default=ROOT/"duel_live")
    parser.add_argument("--endpoint", default=os.environ.get("JEV_ENDPOINT", "http://127.0.0.1:8000/v1/systemone"))
    parser.add_argument("--fast", action="store_true")
    parser.add_argument("--urban", action="store_true", help="CC0 waterfront district and full-screen flight HUD")
    args = parser.parse_args()
    run(args.seconds, args.seed, args.live_dir, args.endpoint, not args.fast, args.urban)
