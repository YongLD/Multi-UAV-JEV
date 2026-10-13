"""Multiple physical X2 pursuers sharing only onboard observations and own odometry."""
from __future__ import annotations

from collections import Counter, deque
import io
import json
import math
import time
import threading
import numpy as np
import mujoco
from PIL import Image, ImageDraw

from duel import (ROOT, DuelEye, Evader, ActionController, angle, yaw_of, attitude,
                  publish, save_jpeg,camera_blocked)
from flight import Pilot
from duel_tactics import ACTIONS, QUESTION, POLICY_VERSION, POLICY_MODE, JevWorker, build_state
from onboard_target import aim_at_execution, pursuit_point_at_execution
from fleet_config import read_config
from fleet_tracking import SharedTargetTrack, team_context, separation_velocity, COLORS, ROLES, validate_count
from camera_controls import validate_camera
from urban_hud import UrbanHud
from hud import _depth_rgb, font
from fleet_model import create_model
from city_layout import RAIN_RADIUS_M
from fleet_roles import RoleBook
from fleet_replay import FleetReplayRecorder
from replay_store import HIT_GOAL, REPLAY_FPS
from fleet_visuals import WorldProjector, world_trajectory, camera_overlay, remember_shot, shot_status, prefer_active_shot
from fleet_control import (TeamCoordinator, navigation_velocity, safe_velocity,
                           detour_velocity,reaction_budget, CONTROL_PERIOD, OBSERVATION_PERIOD)


def fleet_decision_state(scene, own):
    """Keep the model's 2048-token state budget for useful team observations."""
    state = build_state(scene, own)
    target = state['target_from_camera']
    # The fused position/velocity already summarizes history. One future point
    # preserves short-horizon pursuit without sending the same track repeatedly.
    trajectory = target.get('trajectory')
    if trajectory:
        target['trajectory'] = {
            'last_observation_age_s': trajectory['last_observation_age_s'],
            'predictions': trajectory['predictions'][:1],
        }
    target.pop('last_seen', None)
    target.pop('estimated_relative_position_m', None)
    target.pop('estimated_current_bearing_deg', None)
    state.pop('relative_target_description', None)
    state.pop('peripheral_free_space_from_camera_m', None)
    # These are redundant with navigation's measured depth grid and per-action
    # predicted contact/clearance; all physical guards still use the full scene.
    for action in state['navigation_from_camera']['actions'].values():
        action.pop('observed_path_fraction', None)
    def without_nulls(value):
        if isinstance(value, dict):
            return {key: without_nulls(item) for key, item in value.items() if item is not None}
        if isinstance(value, list):
            return [without_nulls(item) for item in value]
        return value
    # Preserve nulls inside depth arrays: those cells mean unobserved space.
    return without_nulls(state)


class Drone:
    def __init__(self, model, data, index, count, endpoint, request_lock, roles):
        self.index, self.id = index, f'P-{index+1:02d}'
        self.roles = roles
        self.body = 'x2' if index == 0 else f'x2_{index+1}'
        body_id = model.body(self.body).id
        joint = model.body_jntadr[body_id]
        self.q = int(model.jnt_qposadr[joint])
        self.v = int(model.jnt_dofadr[joint])
        prefix = '' if index == 0 else self.body+'__'
        self.motors = [model.actuator(prefix+f'thrust{i}').id for i in range(1, 5)]
        start=roles.role(index)['start']
        data.qpos[self.q:self.q+3] = start
        data.qpos[self.q+3:self.q+7] = [1., 0., 0., 0.]
        self.eye = DuelEye(model, target_body='enemy', ignore_geoms=('rain_visual',), own_body=self.body)
        self.pilot = Pilot(model, self.body, self.motors)
        self.controller = ActionController()
        self.controller.altitude = start[2]
        questions = roles.questions(QUESTION,index)
        self.worker = JevWorker(endpoint, hz=2. if index < 3 else 1., questions=questions, request_lock=request_lock)
        # Stagger offers so the shared model is not burst with all agents at once.
        self.worker._sent_wall = time.monotonic() - self.worker.interval + index/3.
        self.allowed_actions = tuple(questions['action']['criteria'])
        self.counts = Counter({a: 0 for a in self.allowed_actions})
        self.scene = self.decision = self.navigation = None
        self.local_target = {}
        self.alive = True
        self.collisions = 0
        self.contacts = set()
        self.distance = 0.
        self.previous = np.array(start,dtype=float)
        self.closest = None
        self.guard_active = False
        self.shot = None
        self.last_shot_attempt = None
        self.replay_rgb = self.replay_depth = None
        self.goal = None
        self.observed_at = 0.
        self.control_mode = 'starting'
        self.safety_limited = False
        self.requested_navigation_speed = 0.
        self.control_updates = 0
        self.last_control_time = 0.
        self.control_max_interval = 0.
        self.heading_command = 0.

    def position(self, data):
        return data.qpos[self.q:self.q+3].copy()

    def yaw(self, data):
        return yaw_of(data.qpos[self.q+3:self.q+7])

    def report(self, data):
        return {'id': self.id, 'position': self.position(data), 'yaw': self.yaw(data),
                'velocity': data.qvel[self.v:self.v+3].copy(), 'target': self.local_target if self.alive else {},'alive':self.alive}

    def own(self, data):
        pos, yaw = self.position(data), self.yaw(data)
        velocity = data.qvel[self.v:self.v+3]
        roll, pitch, yaw_rate = attitude(data.qpos[self.q+3:self.q+7], data.qvel[self.v+3:self.v+6])
        ctrl = self.controller
        return {'speed_mps': float(np.linalg.norm(velocity)), 'horizontal_speed_mps': float(np.linalg.norm(velocity[:2])),
                'velocity_body_mps': [float(velocity[0]*math.cos(yaw)+velocity[1]*math.sin(yaw)),
                                      float(-velocity[0]*math.sin(yaw)+velocity[1]*math.cos(yaw)), float(velocity[2])],
                'last_decision_latency_s': (self.decision or {}).get('total_latency_s',(self.decision or {}).get('latency_s', .4)),
                'last_decision_age_s': self.now-self.decision['sim_time'] if self.decision else .3,
                'altitude_m': float(pos[2]), 'roll_deg': roll, 'pitch_deg': pitch,
                'yaw_deg': math.degrees(yaw), 'yaw_rate_deg_s': yaw_rate,
                'heading_setpoint_deg': math.degrees(angle(ctrl.heading)),
                'heading_error_deg': math.degrees(angle(ctrl.heading-yaw)),
                'speed_setpoint_mps': ctrl.speed, 'altitude_setpoint_m': ctrl.altitude,
                'altitude_error_m': ctrl.altitude-float(pos[2]), 'last_action': ctrl.last_action,
                'tilt_deg':math.degrees(math.acos(float(np.clip(data.xmat[self.pilot.body_id].reshape(3,3)[2,2],-1,1)))),
                'front_clearance_m':self.scene['observed_clearance']['level'] if self.scene else None,
                'navigation_distance_m':None if self.goal is None else float(np.linalg.norm(self.goal-pos)),
                'safety_limited':self.safety_limited,'requested_navigation_speed_mps':self.requested_navigation_speed,
                'shot_cooldown_remaining_s': max(0., 2.-(self.now-ctrl.last_shot_t)),
                'turn_in_progress': abs(angle(ctrl.heading-yaw)) > math.radians(3),
                'vertical_in_progress': abs(ctrl.altitude-float(pos[2])) > .3}

    def observe(self, data, t):
        self.now = t
        self.observed_at = t
        self.scene = self.eye.look(data, self.position(data), self.yaw(data), t)
        self.local_target = dict(self.scene['target'])

    def prepare_state(self, data, target, context):
        self.goal = context.pop('navigation_goal_world',None)
        self.scene['target'] = target
        self.scene['team'] = context
        own = self.own(data)
        self.scene['own_velocity_body_mps'] = own['velocity_body_mps']
        flow = math.atan2(own['velocity_body_mps'][1], own['velocity_body_mps'][0]) if own['horizontal_speed_mps'] > .2 else 0.
        self.scene['velocity_path'] = self.eye.safety.corridor(flow)
        self.scene['reaction_budget_s'] = reaction_budget(self.now-self.observed_at)
        # Full per-action trajectories were computed 6x for every camera tick,
        # even while the serial model was busy. The fast final-command guard
        # validates motion; give Jev the current depth grid and motion summary.
        self.navigation = self.eye.safety.describe(own, ())
        self.navigation['reaction_budget_s'] = self.scene['reaction_budget_s']
        self.scene['navigation'] = self.navigation
        target['aim'] = aim_at_execution(target, own['yaw_rate_deg_s'], own['last_decision_age_s'], own['heading_error_deg'])
        target['lead'] = pursuit_point_at_execution(target, own['velocity_body_mps'], own['yaw_rate_deg_s'], own['last_decision_age_s'], own['heading_error_deg'])
        if self.index > 0 and context['formation_goal_relative_m'] is not None:
            goal = np.array(context['formation_goal_relative_m'])
            if np.linalg.norm(goal) > 4.:
                target['lead'] = {'bearing_deg': context['formation_bearing_deg'], 'height_m': context['formation_height_m'],
                                  'horizontal_horizon_s': .5, 'uncertainty_m': target['position_uncertainty_m']}
        state = fleet_decision_state(self.scene, own)
        state['role_version'] = self.roles.version
        state['navigation_goal'] = {'bearing_deg':context['formation_bearing_deg'],'height_m':context['formation_height_m']}
        nearest=self.navigation.get('nearest_obstacle_m')
        if nearest is not None and nearest<.5:
            state['navigation_from_camera']['imminent_contact']=True
        return state


def decorate_map(view, camera, agents, data, enemy, trails,missions=None,trajectory=None):
    image = Image.fromarray(view).convert('RGBA')
    draw = ImageDraw.Draw(image)
    h, w = view.shape[:2]
    forward, up = np.asarray(camera.forward), np.asarray(camera.up)
    right = np.cross(forward, up)
    focal = h/(2*math.tan(math.radians(24)))
    def project(pos):
        delta = np.asarray(pos)-camera.pos
        depth = delta@forward
        return None if depth <= .1 else (w/2+delta@right/depth*focal, h/2-delta@up/depth*focal)
    bodies = [(a.id, a.position(data), a.yaw(data), COLORS[a.index]) for a in agents]
    bodies.append(('E-01', enemy.pos, enemy.yaw, (252, 129, 125)))
    for label, position, yaw, color in bodies:
        line = [project(p) for p in trails[label]]
        line = [p for p in line if p]
        if len(line) > 1:
            draw.line(line, fill=color+(170,), width=2)
        xy = project(position)
        nose = project(position+np.array([5*math.cos(yaw), 5*math.sin(yaw), 0.]))
        if xy:
            draw.ellipse((xy[0]-4, xy[1]-4, xy[0]+4, xy[1]+4), fill=color+(255,))
            draw.text((xy[0]+7, xy[1]-7), label, fill=color+(255,), font=font(10, True))
            if nose:
                draw.line((*xy, *nose), fill=color+(255,), width=2)
    for a in agents:
        mission=(missions or {}).get(a.id,{})
        goal=mission.get('goal_xyz_m')
        if goal is None or mission.get('kind') not in ('front_block','trajectory_intercept'):continue
        xy,start=project(np.asarray(goal)),project(a.position(data))
        if xy is None:continue
        color=COLORS[a.index]+(230,)
        if start is not None:draw.line([start,xy],fill=color,width=2)
        draw.ellipse((xy[0]-6,xy[1]-6,xy[0]+6,xy[1]+6),outline=color,width=2)
        draw.text((xy[0]+8,xy[1]-8),'B2' if mission['kind']=='front_block' else 'I3',fill=color,font=font(12))
    if trajectory and trajectory['current'] is not None:
        previous=project(trajectory['current'])
        for prediction in trajectory['predictions']:
            xy=project(prediction['point'])
            if previous and xy:
                for start in np.arange(0.,1.,.14):
                    end=min(start+.07,1.)
                    draw.line((previous[0]+(xy[0]-previous[0])*start,previous[1]+(xy[1]-previous[1])*start,
                               previous[0]+(xy[0]-previous[0])*end,previous[1]+(xy[1]-previous[1])*end),fill=(105,206,231,230),width=2)
                draw.text((xy[0]+4,xy[1]),f"+{prediction['horizon_s']:.0f}s",fill=(105,206,231,255),font=font(10))
            previous=xy
    return np.asarray(image.convert('RGB'))


def render_follow_view(renderer,model,data,agent,camera):
    """Observer-only third-person chase camera for this physical airframe."""
    pos=agent.position(data)
    camera.lookat[:]=pos;camera.azimuth=math.degrees(agent.yaw(data))
    camera.distance=5.8;camera.elevation=-14.
    model.vis.global_.fovy=50
    renderer.update_scene(data,camera)
    if camera_blocked(pos,renderer._scene.camera[0].pos,model=model,data=data):
        for distance,elevation in ((3.8,-25.),(2.6,-40.),(2.,-65.)):
            camera.distance=distance;camera.elevation=elevation
            renderer.update_scene(data,camera)
            if not camera_blocked(pos,renderer._scene.camera[0].pos,model=model,data=data):break
    return renderer.render()


def replay_instrument_snapshot(agent,data,t,count,rain_xy,overview,overview_at,team_hits):
    """Detached own-drone HUD state; sensor arrays stay immutable until encoded."""
    if agent.replay_rgb is None or agent.replay_depth is None or overview is None:
        raise RuntimeError('Replay requires RGB, depth and overview from the first frame')
    own=agent.own(data)
    decision=agent.decision or {}
    return {'rgb':agent.replay_rgb,'depth':agent.replay_depth,'overview':overview,
            'scene':{'target':{'visible':False,'range_m':None,'relative_height_m':None,**agent.local_target},
                     'sector_range_m':dict(agent.scene['sector_range_m'])},
            'decision':{'action':decision.get('action'),'probabilities':dict(decision.get('probabilities') or {})},
            'telemetry':{'focused_drone_id':agent.id,'fleet_count':count,'display_mode':'FOLLOW',
                         'model_name':agent.worker.model,'time_s':t,'team_hits':team_hits,
                         'altitude_m':own['altitude_m'],'speed_mps':own['speed_mps'],
                         'vertical_speed_mps':float(data.qvel[agent.v+2]),
                         'yaw_deg':own['yaw_deg'],'roll_deg':own['roll_deg'],'pitch_deg':own['pitch_deg'],
                         'rain_factor':.68 if np.linalg.norm(agent.position(data)[:2]-rain_xy)<RAIN_RADIUS_M else 1.,
                         'collisions':agent.collisions,'distance_flown_m':agent.distance,
                         'allowed_actions':tuple(agent.allowed_actions),'counts':dict(agent.counts),
                         'action':getattr(agent.controller,'last_action',decision.get('action')),
                         'calls':agent.worker.calls,'errors':agent.worker.errors,
                         'latency_s':float(decision.get('latency_s') or 0.),
                         'hits':agent.controller.hits,'shots':agent.controller.shots,
                         'sensor_age_s':max(0.,t-agent.observed_at),
                         'overview_age_s':max(0.,t-overview_at)}}


def run_fleet(seconds, seed, live_dir, endpoint, count, realtime=True):
    live_dir.mkdir(parents=True, exist_ok=True)
    model = create_model(count)
    data = mujoco.MjData(model)
    enemy = Evader(seed)
    enemy_id, rain_id = model.body('enemy').mocapid[0], model.body('rain').mocapid[0]
    data.mocap_pos[enemy_id] = enemy.pos
    request_lock = threading.Lock()
    roles = RoleBook()
    agents = [Drone(model, data, i, count, endpoint, request_lock, roles) for i in range(count)]
    coordinator = TeamCoordinator(roles,count)
    mujoco.mj_forward(model, data)
    track = SharedTargetTrack()
    model.vis.quality.offsamples = 4
    renderer = mujoco.Renderer(model, 720, 1280)
    rgb_renderer = mujoco.Renderer(model, 144, 192)
    monitor_renderer = mujoco.Renderer(model,360,640)
    monitor_camera = mujoco.MjvCamera();monitor_camera.type=mujoco.mjtCamera.mjCAMERA_FREE
    top_renderer = mujoco.Renderer(model, 288, 448)
    hud = UrbanHud(1280, 720)
    camera = mujoco.MjvCamera(); camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    top_camera = mujoco.MjvCamera(); top_camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    top_camera.lookat[:] = [0, 0, 0]; top_camera.distance = 250; top_camera.azimuth = 90; top_camera.elevation = -89.9
    trails = {key: deque(maxlen=500) for key in [a.id for a in agents]+['E-01']}
    overview = None
    overview_at = 0.
    next_observe = next_frame = next_top = next_monitors = next_role_check = 0.
    replay_frame = 0
    records, events, referee = (live_dir/name for name in ('decisions.jsonl', 'events.jsonl', 'referee.jsonl'))
    if (live_dir/'replay_reset.request').exists():
        for name in ('replay.mp4', 'replay_summary.json', 'replay_catalog.json', 'replay_reset.request'):
            (live_dir/name).unlink(missing_ok=True)
    (live_dir/'restart.request').unlink(missing_ok=True)
    publish(live_dir/'policy.json', {'policy_version': POLICY_VERSION+'_missions_v3', 'fleet_count': count,
                                   'model_endpoint': endpoint, 'information_sharing': 'camera_only_ideal_broadcast',
                                   'roles': [roles.role(i)['name'] for i in range(count)], 'role_version':roles.version,
                                   'minimum_separation_m': 4., 'control_hz':1/CONTROL_PERIOD,'observation_hz':1/OBSERVATION_PERIOD,
                                   'model_requests':'serialized_latest_observation'})
    fps = 12 if count == 2 else 10
    # Capture real physics states at the same rate as the single-drone replay.
    # JPEG publication and instrument rendering have independent schedules.
    recorder = FleetReplayRecorder(live_dir, count, REPLAY_FPS)
    began = time.monotonic()
    output, status = None, None
    try:
        with records.open('w') as decision_log, events.open('w') as event_log, referee.open('w') as referee_log:
            for i in range(int(seconds/model.opt.timestep)):
                t = i*model.opt.timestep
                # A selection made during MuJoCo startup can race with removal
                # of the old restart marker. The persistent count is authoritative.
                if status is not None and i % 100 == 0 and read_config(live_dir)['count'] != count:
                    output='manual_restart';break
                if i % 10 == 0 and (live_dir/'restart.request').exists():
                    (live_dir/'restart.request').unlink(missing_ok=True)
                    output = 'manual_restart'; break
                if realtime:
                    delay = t-(time.monotonic()-began)
                    if delay > .001:
                        time.sleep(delay)
                rain_xy = np.array([-12+16*math.sin(t/24), 28+18*math.sin(t/31)])
                data.mocap_pos[rain_id] = [*rain_xy, 0.]
                if time.monotonic() >= next_role_check:
                    if roles.reload():
                        for a in agents:
                            a.worker.questions = roles.questions(QUESTION,a.index)
                    next_role_check = time.monotonic()+1.
                if t >= next_observe:
                    mujoco.mj_forward(model, data)
                    for a in agents:
                        if a.alive:
                            a.observe(data, t)
                            # RGB and depth depict the same front camera and
                            # observation time, independently of viewer focus.
                            a.eye._aim(a.position(data),a.yaw(data))
                            rgb_renderer.update_scene(data,a.eye.cam)
                            a.replay_rgb=rgb_renderer.render().copy()
                            a.replay_depth=a.eye.last_depth.copy()
                            a.replay_rgb.setflags(write=False);a.replay_depth.setflags(write=False)
                    reports = [a.report(data) for a in agents]
                    track.update(reports, t)
                    coordinator.update(reports,[a.own(data) for a in agents],track,t)
                    for a, report in zip(agents, reports):
                        if a.alive:
                            context = coordinator.context(a.index,reports,track,t,team_context(report, reports, track, t, a.index))
                            state = a.prepare_state(data, track.relative(report, t), context)
                            if a.index==3:
                                a.worker.interval=.3 if coordinator.assignment else 1.
                            a.worker.offer(state, t)
                    next_observe = t+OBSERVATION_PERIOD
                for a in agents:
                    a.now = t
                    for result in a.worker.drain():
                        age = t-result['sim_time']
                        result.update(drone_id=a.id, role=roles.role(a.index)['name'], applied_sim_time=round(t, 3), age_s=round(age, 3), applied=False)
                        if a.alive and result.get('action') in a.allowed_actions and age <= .8:
                            action = result['action']; shot = None
                            if a.index==3 and coordinator.self_rescue.blocks_support and action!='self_rescue':
                                result['reason']='own recovery has priority'
                                decision_log.write(json.dumps(result,ensure_ascii=False)+'\n');continue
                            if action == 'self_rescue':
                                if a.index!=3 or not coordinator.self_rescue.request(a.report(data),a.own(data),coordinator.health[3],t):
                                    result['reason']='own recovery not needed or attempts exhausted'
                                    decision_log.write(json.dumps(result,ensure_ascii=False)+'\n');continue
                                a.controller.last_action=action
                            elif action == 'rescue':
                                if coordinator.self_rescue.blocks_support:
                                    result['reason']='support aircraft is recovering itself'
                                    decision_log.write(json.dumps(result,ensure_ascii=False)+'\n');continue
                                coordinator.authorize_rescue(a.report(data)); a.controller.last_action = action
                            elif action == 'fire':
                                # Several queued drone decisions may execute in one
                                # physics step. Stop further fire as soon as the
                                # team's thirtieth hit has been counted.
                                if sum(member.controller.hits for member in agents) >= HIT_GOAL:
                                    result['reason']='episode hit goal already reached'
                                    decision_log.write(json.dumps(result,ensure_ascii=False)+'\n');continue
                                local = a.local_target
                                aim = a.scene['target'].get('aim') or {}
                                if coordinator.health[a.index]!='healthy' or not local.get('visible') or t-a.observed_at>.3 or not aim or aim.get('range_m',100)>25 or abs(aim.get('bearing_deg',180))>40 or abs(aim.get('height_m',100))>1:
                                    a.last_shot_attempt={'time_s':t,'number':a.controller.shots,'hit':False,'cooldown_ok':False,'blocked':True}
                                    a.shot=prefer_active_shot(a.shot,a.last_shot_attempt,t)
                                    result['reason'] = 'fire safety precondition changed'; decision_log.write(json.dumps(result,ensure_ascii=False)+'\n'); continue
                                shot = a.controller.apply(action, a.yaw(data), t, a.position(data), enemy.pos)
                            else:
                                a.controller.apply(action, a.yaw(data), t, a.position(data), enemy.pos)
                            result.update(applied=True, shot=shot, executed_action=a.controller.last_action)
                            a.counts[result['action']] += 1; a.decision = result
                            if shot:
                                a.last_shot_attempt=remember_shot(shot,a.position(data),a.yaw(data),enemy.pos,t,a.controller.shots)
                                a.shot=prefer_active_shot(a.shot,a.last_shot_attempt,t)
                                event_log.write(json.dumps({'time_s': t, 'drone_id': a.id, 'type': 'shot', **shot})+'\n'); event_log.flush()
                        else:
                            result['reason'] = 'error' if 'error' in result else 'inactive or stale observation'
                        decision_log.write(json.dumps(result, ensure_ascii=False)+'\n'); decision_log.flush()
                if i % 10 == 0:
                    reports = [a.report(data) for a in agents]
                    coordinator.update(reports,[a.own(data) for a in agents],track,t)
                    for event in coordinator.self_rescue.take_events():
                        if event['type']=='self_rescue_started':
                            agents[3].controller.last_action='self_rescue'
                            if event['source']!='model':agents[3].counts['self_rescue']+=1
                        elif event['type']=='self_rescue_recovered':agents[3].controller.last_action='forward'
                        event_log.write(json.dumps(event)+'\n');event_log.flush()
                    if count==4 and coordinator.assignment and not coordinator.authorized and coordinator.health[3]=='healthy' and roles.config['rescue']['auto_authorize']:
                        coordinator.authorize_rescue(reports[3])
                        agents[3].controller.last_action='rescue';agents[3].counts['rescue']+=1
                        event_log.write(json.dumps({'time_s':round(t,3),'drone_id':'P-04','type':'rescue_authorized',
                            'source':'role_rule','role_version':roles.version,'target':coordinator.assignment})+'\n');event_log.flush()
                    for a, report in zip(agents, reports):
                        if not a.alive:
                            data.ctrl[a.motors] = 0.; continue
                        factor = .68 if np.linalg.norm(report['position'][:2]-rain_xy) < RAIN_RADIUS_M else 1.
                        previous_heading = a.heading_command
                        a.goal = coordinator.goal(a.index,reports,track,t)
                        assisted = count<4 or (coordinator.authorized and coordinator.assignment==a.id and np.linalg.norm(reports[3]['position']-report['position'])<=roles.config['rescue']['command_range_m'])
                        if count==4 and assisted and coordinator.assignment==a.id and (coordinator.health[a.index] in ('unstable','stuck') or a.own(data)['tilt_deg']>25):
                            coordinator.delivered=True
                        recovering_tilt=assisted and a.own(data)['tilt_deg']>25
                        if a.index==3 and coordinator.self_rescue.blocks_support:
                            a.velocity_command,a.heading_command,a.control_mode,recover=coordinator.self_rescue.command(
                                report,a.eye.safety_views,[r for r in reports if r['id']!=a.id],t)
                            a.pilot.recovery_requested=recover
                        else:
                            a.pilot.recovery_requested = coordinator.health[a.index]=='unstable' or recovering_tilt
                            a.velocity_command, a.heading_command, a.control_mode = navigation_velocity(report,a.goal,track,t,
                                a.controller.last_action,'unstable' if recovering_tilt else coordinator.health[a.index],assisted,a.eye.safety_views,coordinator.motion(a.index))
                        a.velocity_command *= factor
                        correction, a.closest = separation_velocity(report['position'], report['velocity'], [r for r in reports if r['id'] != a.id])
                        a.guard_active = bool(np.linalg.norm(correction) > .1)
                        a.velocity_command += correction
                        requested_speed=float(np.linalg.norm(a.velocity_command))
                        a.requested_navigation_speed=requested_speed
                        a.velocity_command,a.safety_limited = safe_velocity(a.velocity_command,report,a.eye.safety_views,[r for r in reports if r['id']!=a.id],t)
                        if requested_speed>.5 and np.linalg.norm(a.velocity_command)<.3 and a.control_mode in ('role_navigation','tracking','tracking_relay','block_transit','block_arrive_late','intercept_transit','intercept_unreachable','communication_support','rescue_approach'):
                            detour,heading=detour_velocity(report,a.goal,a.eye.safety_views,[r for r in reports if r['id']!=a.id],t,a.controller.last_action)
                            if np.linalg.norm(detour)>.3:
                                a.velocity_command,a.heading_command,a.control_mode=detour,heading,'clear_route_detour'
                        # A raw 180deg yaw setpoint saturates the torque-priority
                        # mixer and adds excessive collective thrust. Slew it.
                        heading_step = math.radians(60)*CONTROL_PERIOD
                        a.heading_command = previous_heading+float(np.clip(angle(a.heading_command-previous_heading),-heading_step,heading_step))
                        a.controller.heading = a.heading_command
                        a.controller.altitude = float(report['position'][2])+float(a.velocity_command[2])/1.1
                        a.controller.speed = float(np.linalg.norm(a.velocity_command[:2]))
                        a.control_max_interval = max(a.control_max_interval,t-a.last_control_time)
                        a.last_control_time = t; a.control_updates += 1
                for a in agents:
                    if a.alive:
                        data.ctrl[a.motors] = a.pilot(data, a.velocity_command, a.heading_command, model.opt.timestep)
                mujoco.mj_step(model, data)
                enemy.step(t, model.opt.timestep, rain_xy)
                data.mocap_pos[enemy_id] = enemy.pos
                data.mocap_quat[enemy_id] = [math.cos(enemy.yaw/2), 0., 0., math.sin(enemy.yaw/2)]
                if i % 250 == 0:
                    referee_log.write(json.dumps({'time_s': round(t, 3), 'drones': [{'id': a.id, 'xyz_m': a.position(data).tolist()} for a in agents],
                                                  'enemy_xyz_m': enemy.pos.tolist(), 'rain_xy_m': rain_xy.tolist()})+'\n'); referee_log.flush()
                for c in range(data.ncon):
                    contact = data.contact[c]; b1, b2 = model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2]
                    for a in agents:
                        if (b1 == model.body(a.body).id) != (b2 == model.body(a.body).id):
                            pair = tuple(sorted((int(contact.geom1), int(contact.geom2))))
                            if pair not in a.contacts:
                                a.contacts.add(pair); a.collisions += 1
                for a in agents:
                    if a.alive and a.position(data)[2] < -.5:
                        a.alive = False; a.worker.close(); a.local_target = {}
                if t >= next_top:
                    mujoco.mj_forward(model,data)
                    for a in agents:trails[a.id].append(a.position(data))
                    trails['E-01'].append(enemy.pos.copy())
                    model.vis.global_.fovy=48;top_renderer.update_scene(data,top_camera)
                    overview=decorate_map(top_renderer.render(),top_renderer._scene.camera[0],agents,data,enemy,trails,
                                          {a.id:coordinator.mission(a.index,t) for a in agents},
                                          world_trajectory(track,coordinator.missions.predictor,t))
                    overview.setflags(write=False)
                    save_jpeg(live_dir/'overview.jpg',overview,84)
                    overview_at=t;next_top=t+.3
                if t+1e-9 >= replay_frame/REPLAY_FPS:
                    mujoco.mj_forward(model, data)
                    trajectory=world_trajectory(track,coordinator.missions.predictor,t)
                    record_views=[];instruments=[]
                    publish_monitors=t+1e-9>=next_monitors
                    team_hits=sum(member.controller.hits for member in agents)
                    for a in agents:
                        raw=render_follow_view(monitor_renderer,model,data,a,monitor_camera)
                        record_views.append(raw.copy())
                        project=WorldProjector(monitor_renderer._scene.camera[0],1280,720)
                        info=replay_instrument_snapshot(a,data,t,count,rain_xy,overview,overview_at,team_hits)
                        info['telemetry'].update(camera_overlay(project,trajectory,a.shot,t))
                        info['enemy_xy']=project(trajectory['current']) if a.local_target.get('visible') and trajectory['current'] is not None else None
                        instruments.append(info)
                        if a.index==3:info['telemetry']['self_rescue']=coordinator.self_rescue.summary(t)
                    recorder.append(record_views,sim_time=t,instruments=instruments,publish_live=publish_monitors)
                    replay_frame += 1
                    if publish_monitors:
                        next_monitors += .2
                if t >= next_frame:
                    mujoco.mj_forward(model, data)
                    focus = min(read_config(live_dir)['focus'], count-1)
                    focused = agents[focus]
                    own, pos, yaw = focused.own(data), focused.position(data), focused.yaw(data)
                    for a in agents:
                        a.distance += float(np.linalg.norm(a.position(data)-a.previous)); a.previous = a.position(data)
                    try:
                        settings = validate_camera(json.loads((live_dir/'camera.json').read_text()))
                    except (OSError, ValueError, TypeError):
                        settings = {'mode': 'follow', 'azimuth': 0., 'elevation': -14., 'distance': 5.8}
                    camera.lookat[:] = pos; camera.distance = settings['distance']
                    camera.elevation = -89. if settings['mode'] == 'top' else settings['elevation']
                    camera.azimuth = math.degrees(yaw)+settings['azimuth'] if settings['mode'] == 'follow' else settings['azimuth']
                    model.vis.global_.fovy = 50; renderer.update_scene(data, camera)
                    view = renderer.render()
                    cam = renderer._scene.camera[0]
                    trajectory=world_trajectory(track,coordinator.missions.predictor,t)
                    focused_overlay=camera_overlay(WorldProjector(cam,1280,720),trajectory,focused.shot,t)
                    forward, up = np.asarray(cam.forward), np.asarray(cam.up); right = np.cross(forward, up)
                    relative = enemy.pos-cam.pos; distance = float(relative@forward)
                    enemy_xy = None if distance <= .1 or not focused.local_target.get('visible') else (640+relative@right/distance*720/(2*math.tan(math.radians(25))), 360-relative@up/distance*720/(2*math.tan(math.radians(25))))
                    rgb = focused.replay_rgb
                    depth = focused.replay_depth
                    drones = []
                    for a in agents:
                        info = a.own(data)
                        target = a.scene['target']
                        drones.append({'id': a.id, 'role': roles.role(a.index)['name'], 'alive': a.alive,
                                       'color': '#'+''.join(f'{c:02x}' for c in COLORS[a.index]),
                                       'altitude_m': round(info['altitude_m'], 1), 'speed_mps': round(info['speed_mps'], 1),
                                       'action': a.controller.last_action, 'target_visible': bool(a.local_target.get('visible')),
                                       'information_source': target.get('information_source'), 'range_m': target.get('range_m'),
                                       'calls': a.worker.calls, 'errors': a.worker.errors, 'hits': a.controller.hits,
                                       'shot_status':shot_status(a.shot,t,a.last_shot_attempt),
                                       'collisions': a.collisions, 'closest_teammate_m': a.closest, 'separation_guard': a.guard_active,
                                       'health':coordinator.health[a.index],'control_mode':a.control_mode,'safety_limited':a.safety_limited,
                                       'mission':coordinator.mission(a.index,t),
                                       **({'self_rescue':coordinator.self_rescue.summary(t)} if a.index==3 else {}),
                                       'allowed_actions':list(a.allowed_actions),'control_updates':a.control_updates,
                                       'control_max_interval_s':round(a.control_max_interval,3),'roll_deg':info['roll_deg'],'pitch_deg':info['pitch_deg']})
                    status = {'running': True, 'time_s': round(t, 2), 'seed': seed, 'version': 'urban', 'hit_goal':HIT_GOAL,
                              'episode_limit_s':seconds,'replay_fps':REPLAY_FPS,
                              'fleet_count': count, 'focused_drone_id': focused.id, 'focus': focus, 'drones': drones,
                              'shared_track': track.summary(t), 'fleet_calls': sum(a.worker.calls for a in agents),
                              'role_version':roles.version,'role_error':roles.error,'effective_tracker':coordinator.effective_tracker,
                              'primary_tracking_agent':coordinator.primary_tracking_agent,
                              'rescue':{'assigned_to':coordinator.assignment,'authorized':coordinator.authorized,
                                        'command_delivered':coordinator.delivered,
                                        'completed':coordinator.rescues_completed,'failed':coordinator.rescues_failed,'last_result':coordinator.outcome},
                              'control_hz':1/CONTROL_PERIOD,'observation_hz':1/OBSERVATION_PERIOD,
                              'sim_to_wall_ratio':round(t/max(.001,time.monotonic()-began),3),
                              'allowed_actions':list(focused.allowed_actions),
                              'monitor_view':'third_person_follow_full_hud',
                              'hits': sum(a.controller.hits for a in agents), 'shots': sum(a.controller.shots for a in agents),
                              'shot_status':shot_status(focused.shot,t,focused.last_shot_attempt),
                              'collisions': sum(a.collisions for a in agents), 'distance_flown_m': round(sum(a.distance for a in agents), 1),
                              'action': focused.controller.last_action, 'counts': dict(focused.counts),
                              'calls': focused.worker.calls, 'errors': focused.worker.errors,
                              'fleet_errors': sum(a.worker.errors for a in agents), 'last_error': focused.worker.last_error,
                              'target': focused.scene['target'], 'speed_mps': round(own['speed_mps'], 2),
                              'altitude_m': round(own['altitude_m'], 2), 'vertical_speed_mps': round(float(data.qvel[focused.v+2]), 2),
                              'roll_deg': own['roll_deg'], 'pitch_deg': own['pitch_deg'], 'yaw_deg': own['yaw_deg'],
                              'rain_factor': .68 if np.linalg.norm(pos[:2]-rain_xy) < RAIN_RADIUS_M else 1.,
                              'enemy_speed_mps': round(enemy.speed, 2), 'navigation': focused.navigation,
                              'free_space': {'horizontal': focused.scene['sector_range_m'], **focused.scene['observed_clearance']},
                              'model_endpoint': endpoint, 'model_name': focused.worker.model, 'model_backend': focused.worker.backend,
                              'model_input_mode': focused.worker.input_mode, 'policy_mode': POLICY_MODE,
                              'policy_version': POLICY_VERSION+'_missions_v3', 'scene_version': 'urban_role_missions_v3',
                              'viewer_camera': {'mode': settings['mode'], 'azimuth': camera.azimuth, 'distance': camera.distance},
                              'last_decision': {k: focused.decision.get(k) for k in ('action', 'probabilities', 'latency_s','queue_wait_s','total_latency_s','age_s')} if focused.decision else None}
                    telemetry = {**status, **focused_overlay, 'latency_s': (focused.decision or {}).get('latency_s', 0.)}
                    clean = hud.decorate(view, telemetry, enemy_xy).convert('RGB')
                    save_jpeg(live_dir/'scene.jpg', np.asarray(clean), 85)
                    save_jpeg(live_dir/'rgb.jpg', rgb, 90); save_jpeg(live_dir/'depth.jpg', _depth_rgb(depth, 35.), 92)
                    image = hud.draw(view, depth, focused.scene, focused.decision or {}, telemetry, enemy_xy, rgb=rgb, overview=overview)
                    save_jpeg(live_dir/'frame.jpg', image, 80)
                    publish(live_dir/'status.json', status)
                    next_frame = t+1/fps
                if sum(a.controller.hits for a in agents) >= HIT_GOAL:
                    output = 'success'; break
                if not any(a.alive for a in agents):
                    output = 'crash'; break
        output = output or 'timeout'
    finally:
        recorder.close()
        for a in agents:
            a.worker.close(); a.eye.depth.close(); a.eye.seg.close()
        renderer.close(); rgb_renderer.close(); monitor_renderer.close(); top_renderer.close()
    summary = {'outcome': output, 'time_s': round(t, 2), 'hits': sum(a.controller.hits for a in agents), 'hit_goal':HIT_GOAL,
               'shots': sum(a.controller.shots for a in agents), 'collisions': sum(a.collisions for a in agents),
               'episode_limit_s':seconds,
               'seed': seed, 'fleet_count': count, 'drones': drones, 'model_endpoint': endpoint,
               'model_name': agents[0].worker.model, 'policy_version': POLICY_VERSION+'_missions_v3',
               'scene_version': 'urban_role_missions_v3','rescue':status.get('rescue') if status else None, 'shared_track': track.summary(t)}
    summary['replay_retained'] = recorder.retain(live_dir, summary)
    publish(live_dir/'summary.json', summary)
    if status:
        status.update(running=False, summary=summary, hits=summary['hits'], time_s=summary['time_s']); publish(live_dir/'status.json', status)
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    return summary
