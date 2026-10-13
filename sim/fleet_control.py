"""Continuous role navigation, broadcast health/rescue, and final camera/peer safety."""
import math
import numpy as np
from onboard_target import rotate, angles
from onboard_safety import BRAKE_ACCEL, MARGIN, SAMPLE_OFFSETS
from fleet_missions import MissionPlanner
from fleet_self_rescue import SupportSelfRescue

CONTROL_PERIOD = .02
OBSERVATION_PERIOD = .15


def rotate_many(vectors, yaw):
    c,s = math.cos(yaw),math.sin(yaw)
    return np.asarray(vectors) @ np.array([[c,s,0],[-s,c,0],[0,0,1]])


def reaction_budget(observation_age, control_period=CONTROL_PERIOD):
    # The continuous controller does not wait for Jev. Account for the age of
    # the camera geometry, the next control tick, and motor response.
    return max(0., observation_age)+control_period+.25


def safe_velocity(desired, report, views, peers, now):
    """Project the FINAL command (including separation) into measured free space.

    Each view records own odometry at capture; transform candidate samples back
    into that camera frame. No building coordinates or target truth are used.
    """
    candidate = np.asarray(desired, dtype=float).copy()
    if not np.all(np.isfinite(candidate)):
        return np.zeros(3), True
    horizontal = np.linalg.norm(candidate[:2])
    if horizontal > 6.3:
        candidate[:2] *= 6.3/horizontal
    candidate[2] = np.clip(candidate[2], -1.5, 1.5)
    fresh = [v for v in views if now-v['time'] <= .45]
    if not fresh:
        return np.zeros(3), True
    age = max(now-v['time'] for v in fresh)
    speed = float(np.linalg.norm(candidate))
    if speed < .01:
        return candidate, False
    direction = candidate/speed
    # Search only along the requested vector. Unknown cells remain unavailable.
    # Camera sees from .15m forward; skip the already occupied airframe envelope.
    distances = np.arange(1.0, 12.1, .5)
    path = np.asarray(report['position'])+distances[:,None]*direction
    samples = path[:,None,:]+rotate_many(SAMPLE_OFFSETS, report['yaw'])[None,:,:]
    covered = np.zeros(samples.shape[:2], dtype=bool)
    blocked = np.zeros_like(covered)
    for view in fresh:
        local = rotate_many(samples-view['position'], -view['yaw'])
        seen, hit = view['safety'].query(local.reshape(-1,3))
        covered |= seen.reshape(covered.shape)
        blocked |= hit.reshape(blocked.shape)
    danger = np.flatnonzero(blocked.any(axis=1) | (covered.mean(axis=1) < .75))
    clear = distances[danger[0]]-.5 if danger.size else distances[-1]
    available = max(0., clear-MARGIN)
    reaction = reaction_budget(age)
    limit = math.sqrt((BRAKE_ACCEL*reaction)**2+2*BRAKE_ACCEL*available)-BRAKE_ACCEL*reaction
    scale = min(1.,limit/speed)
    # Do not let any command approach an already-close teammate, including
    # rescue recipients. Check actual velocity stopping distance as well.
    for peer in peers:
        delta = np.asarray(peer['position'])-report['position']
        distance = float(np.linalg.norm(delta))
        if distance < .01:
            scale = 0.; continue
        toward = delta/distance
        other_v = np.asarray(peer['velocity'])
        own_closing = max(0., float((np.asarray(report['velocity'])-other_v)@toward))
        buffer = 4.+own_closing*reaction+own_closing**2/(2*BRAKE_ACCEL)
        allowance = max(0., (distance-buffer)/.5)
        approach = float(candidate@toward)
        if approach > .01:
            scale = min(scale,max(0.,allowance+float(other_v@toward))/approach)
    # Scaling preserves the final camera-checked ray; no guard can rotate it
    # into an unchecked building after this point.
    result = candidate*scale
    return result, bool(np.linalg.norm(result-candidate) > .05)


def escape_velocity(report, views, now):
    best = (0., np.zeros(3))
    for view in views:
        if now-view['time'] > .45:
            continue
        for bearing in (-40, 0, 40):
            for dz in (0., 2.):
                corridor = view['safety'].corridor(math.radians(bearing), dz)
                clear = corridor['clearance_m'] or 0.
                if corridor['status'] == 'unknown' or clear < 3.:
                    continue
                direction = rotate([math.cos(math.radians(bearing)), math.sin(math.radians(bearing)), dz/4], view['yaw'])
                if clear > best[0]:
                    best = (clear, direction/max(np.linalg.norm(direction),1e-6))
    return best[1]


def detour_velocity(report, goal, views, peers, now, action):
    """Choose a measured clear local alternative when the direct slot is blocked."""
    if goal is None:return np.zeros(3),report['yaw']
    delta=np.asarray(goal)-report['position'];heading=math.atan2(delta[1],delta[0])
    best=(0.,np.zeros(3),heading)
    preference=1 if action=='turn_left' else -1 if action=='turn_right' else 0
    for offset in (-60,-40,40,60):
        yaw=report['yaw']+math.radians(offset)
        desired=np.array([2.*math.cos(yaw),2.*math.sin(yaw),np.clip(delta[2]*.3,-.5,.5)])
        velocity,_=safe_velocity(desired,report,views,peers,now)
        score=np.linalg.norm(velocity[:2])*(.5+.5*math.cos(yaw-heading))+.02*preference*np.sign(offset)
        if score>best[0] and np.linalg.norm(velocity[:2])>.3:best=(score,velocity,yaw)
    return best[1],best[2]


class TeamCoordinator:
    def __init__(self, roles, count):
        self.roles, self.count = roles, count
        self.stuck_since = [None]*count
        self.stable_since = [None]*count
        self.health = ['healthy']*count
        self.assignment = None
        self.authorized = False
        self.delivered = False
        self.started = None
        self.cooldowns = {}
        self.effective_tracker = 'P-01'
        self.handoff_at = -100.
        self.rescues_completed = self.rescues_failed = 0
        self.outcome = None
        self.heading = 0.
        self.broadcast_health = []
        self.primary_tracking_agent = 'P-01'
        self.missions = MissionPlanner(roles,count)
        self.self_rescue = SupportSelfRescue(roles)

    def update(self, reports, own, track, now):
        config = self.roles.config['rescue']
        for i,(r,o) in enumerate(zip(reports,own)):
            tilt = o.get('tilt_deg', max(abs(o['roll_deg']),abs(o['pitch_deg'])))
            distance = o.get('navigation_distance_m')
            stuck = o.get('front_clearance_m') is not None and o['front_clearance_m'] < 3. and o['speed_mps'] < .8 and (distance is None or distance > 5.)
            if stuck:
                if self.stuck_since[i] is None:self.stuck_since[i] = now
            else:self.stuck_since[i] = None
            self.health[i] = ('unavailable' if not r.get('alive',True) else 'unstable' if tilt >= config['tilt_degrees'] else
                              'stuck' if self.stuck_since[i] is not None and now-self.stuck_since[i] >= config['stuck_seconds'] else 'healthy')
        if self.count==4:
            self.self_rescue.update(reports[3],own[3],self.health[3],now)
            if self.self_rescue.blocks_support and self.assignment:
                self.outcome={'target':self.assignment,'result':'support_self_rescue','time_s':round(now,2)}
                self.assignment=None;self.authorized=self.delivered=False
        self.broadcast_health=[{'id':r['id'],'state':self.health[i],
            'tilt_deg':round(o.get('tilt_deg',0),1),'speed_mps':round(o['speed_mps'],2),
            'altitude_m':round(float(r['position'][2]),1),
            'stuck_for_s':round(now-self.stuck_since[i],2) if self.stuck_since[i] is not None else 0.}
            for i,(r,o) in enumerate(zip(reports,own))]
        if self.assignment:
            i = int(self.assignment[-2:])-1
            stable = self.health[i] == 'healthy' and own[i].get('tilt_deg',90) < 25
            if stable:
                if self.stable_since[i] is None:self.stable_since[i] = now
            else:self.stable_since[i] = None
            if self.authorized and self.delivered and self.stable_since[i] is not None and now-self.stable_since[i] >= config['stable_seconds']:
                self.rescues_completed += 1; self.outcome = {'target':self.assignment,'result':'recovered','time_s':round(now,2)}
                self.assignment = None; self.authorized = self.delivered = False
            elif not self.delivered and self.stable_since[i] is not None and now-self.stable_since[i] >= config['stable_seconds']:
                self.outcome={'target':self.assignment,'result':'self_recovered','time_s':round(now,2)}
                self.assignment=None;self.authorized=False
            elif now-self.started > config['timeout_seconds']:
                self.rescues_failed += 1; self.cooldowns[self.assignment] = now+5.
                self.outcome = {'target':self.assignment,'result':'timeout','time_s':round(now,2)}
                self.assignment = None; self.authorized = self.delivered = False
        if self.count == 4 and not self.assignment and self.health[3] == 'healthy' and not self.self_rescue.blocks_support:
            candidates = [i for i in range(3) if self.health[i] in ('unstable','stuck') and now >= self.cooldowns.get(reports[i]['id'],-1)]
            if candidates:
                i = min(candidates,key=lambda i:(0 if self.health[i]=='unstable' else 1,np.linalg.norm(reports[i]['position']-reports[3]['position'])))
                self.assignment = reports[i]['id']; self.started = now; self.stable_since[i] = None; self.delivered=False
        healthy = [i for i in range(min(3,self.count)) if self.health[i] == 'healthy']
        visible = [i for i in healthy if reports[i]['target'].get('visible')]
        candidates = visible or healthy
        if candidates:
            best = max(candidates,key=lambda i:reports[i]['target'].get('track_confidence',0)-.002*(reports[i]['target'].get('range_m') or 100))
            current = int(self.effective_tracker[-2:])-1
            if current not in candidates or (now-self.handoff_at > 3. and reports[best]['target'].get('track_confidence',0) > reports[current]['target'].get('track_confidence',0)+.15):
                self.effective_tracker = reports[best]['id']; self.handoff_at = now
        if np.linalg.norm(track.velocity[:2]) > .5:
            self.heading = math.atan2(track.velocity[1],track.velocity[0])
        self.primary_tracking_agent = 'P-01' if self.health[0]=='healthy' else self.effective_tracker
        self.missions.update(reports,track,now,self.primary_tracking_agent)

    def goal(self, index, reports, track, now):
        role = self.roles.role(index)
        if index == 3:
            positions = [r['position'] for i,r in enumerate(reports[:3]) if self.health[i]!='unavailable']
            center = np.mean(positions,axis=0) if positions else reports[index]['position'].copy()
            if self.assignment:
                patient = reports[int(self.assignment[-2:])-1]['position']
                delta = reports[index]['position']-patient; delta[2] = 0.
                if np.linalg.norm(delta) < .1:delta = np.array([-1.,0.,0.])
                return patient+delta/np.linalg.norm(delta)*self.roles.config['rescue']['stand_off_m']+np.array([0,0,2.])
            return center+rotate(role['offset'],self.heading)
        return self.missions.plans.get(index,{}).get('point')

    def mission(self,index,now):
        if index==3:
            if self.self_rescue.blocks_support:
                return {'kind':'support_rescue','status':'self_rescue_'+self.self_rescue.state,
                        'self_rescue':self.self_rescue.summary(now)}
            return {'kind':'support_rescue','status':'rescue_approach' if self.assignment else 'communication_support'}
        return self.missions.details(index,now)

    def motion(self,index):
        if index==3:return {'feedforward':np.zeros(3),'face':None,'mode':self.mission(index,0)['status']}
        return self.missions.motion(index)

    def context(self, index, reports, track, now, context):
        r = reports[index]; goal = self.goal(index,reports,track,now)
        relative = None if goal is None else rotate(goal-r['position'],-r['yaw'])
        motion=self.motion(index)
        face=motion['face']
        facing=relative if face is None else rotate(face-r['position'],-r['yaw'])
        status = self.health[index]
        return {**context,'role':self.roles.role(index)['name'],'effective_tracker':self.effective_tracker,
                'primary_tracking_agent':self.primary_tracking_agent,'mission':self.mission(index,now),
                'health':status,'peers_health':[entry if index==3 else {'id':entry['id'],'state':entry['state']} for i,entry in enumerate(self.broadcast_health) if i!=index],
                'rescue_assignment':self.assignment if index==3 else None,'rescue_authorized':self.authorized,
                **({'self_rescue':self.self_rescue.summary(now)} if index==3 else {}),
                'formation_goal_relative_m':None if relative is None else np.round(relative,2).tolist(),
                'formation_bearing_deg':None if facing is None else angles(facing)['bearing_deg'],
                'formation_height_m':None if relative is None else round(float(relative[2]),2),
                'navigation_goal_world':goal}

    def authorize_rescue(self, report):
        if report['id'] == 'P-04' and self.assignment and not self.self_rescue.blocks_support:
            self.authorized = True
            return True
        return False


def navigation_velocity(report, goal, track, now, action, health, assisted, views, motion=None):
    if health == 'unavailable':return np.zeros(3),report['yaw'],'unavailable'
    if health == 'unstable':return np.zeros(3),report['yaw'],'attitude_recovery'
    if assisted and health == 'stuck':
        escape = escape_velocity(report,views,now)
        return escape, math.atan2(escape[1],escape[0]) if np.linalg.norm(escape[:2])>.1 else report['yaw'], 'rescue_escape'
    if health == 'stuck':return np.zeros(3),report['yaw'],'awaiting_rescue'
    if goal is None:
        search = {'P-01':0.,'P-02':math.pi/2,'P-03':-math.pi/2,'P-04':math.pi}[report['id']]
        return np.zeros(3), (search+now*.2)%(2*math.pi), 'search'
    error = np.asarray(goal)-report['position']
    feedforward = motion['feedforward'] if motion is not None else track.velocity if report['id']!='P-04' and track.summary(now)['available'] else np.zeros(3)
    velocity = .65*error+feedforward
    # Stand-off cameras need to turn before translating into the desired route.
    heading = math.atan2(error[1],error[0]) if np.linalg.norm(error[:2])>2. else report['yaw']
    if motion is not None and motion.get('face') is not None:
        face=np.asarray(motion['face'])-report['position']
        if np.linalg.norm(face[:2])>.1:heading=math.atan2(face[1],face[0])
    speed = np.linalg.norm(velocity[:2])
    limit=motion.get('max_speed_mps',6.3) if motion is not None else 6.3
    if speed > limit:velocity[:2] *= limit/speed
    velocity[2] = np.clip(velocity[2],-1.5,1.5)
    return velocity,heading,motion.get('mode','role_navigation') if motion is not None else 'role_navigation'
