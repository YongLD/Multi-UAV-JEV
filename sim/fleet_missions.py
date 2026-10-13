"""Distinct pursuit, forward blockade and turn-aware interception from shared observations.

No enemy truth or city map is accepted. ETA is an obstacle-free lower bound;
the existing camera safety controller must validate every actual movement.
"""
import math
import numpy as np
from onboard_target import rotate


def wrap(x):
    return (x+math.pi)%(2*math.pi)-math.pi


def arrival_time(report, point, speed, acceleration):
    delta=np.asarray(point)-report['position']
    distance=float(np.linalg.norm(delta[:2]))
    if distance<.1:return abs(float(delta[2]))/1.5
    direction=delta[:2]/distance
    initial=float(np.clip(np.asarray(report['velocity'])[:2]@direction,0.,speed))
    ramp=(speed-initial)/acceleration
    ramp_distance=(initial+speed)*ramp/2
    horizontal=(-initial+math.sqrt(initial**2+2*acceleration*distance))/acceleration if distance<=ramp_distance else ramp+(distance-ramp_distance)/speed
    turn=abs(wrap(math.atan2(delta[1],delta[0])-report['yaw']))/math.radians(60)
    # The velocity and yaw loops run concurrently; summing their times would
    # incorrectly reject reachable meeting points as if rotation preceded flight.
    return max(horizontal,abs(float(delta[2]))/1.5,turn)+.25


class TrajectoryPredictor:
    def __init__(self):
        self.stamp=None
        self.position=None
        self.velocity=np.zeros(3)
        self.turn_rate=0.
        self.model='constant_velocity'
        self.vertical_horizon=2.

    def update(self, track, config):
        stamp=(track.observed_at,config['turn_window_s'],config['max_turn_rate_deg_s'],config['vertical_horizon_s'])
        if stamp==self.stamp:return
        self.stamp=stamp
        self.vertical_horizon=config['vertical_horizon_s']
        self.position=None if track.position is None else np.asarray(track.position).copy()
        self.at=track.observed_at
        self.velocity=np.asarray(track.velocity).copy()
        self.turn_rate=0.;self.model='constant_velocity'
        if self.at is None:return
        history=[(t,p) for t,p in track.history if self.at-t<=config['turn_window_s']]
        if len(history)<6 or history[-1][0]-history[0][0]<.6:return
        times=np.array([t-self.at for t,p in history])
        positions=np.array([p for t,p in history])
        basis=np.column_stack((np.ones_like(times),times,.5*times**2))
        coefficients=np.linalg.lstsq(basis,positions,rcond=None)[0]
        residual=float(np.sqrt(np.mean((basis@coefficients-positions)**2)))
        velocity,accel=coefficients[1],coefficients[2]
        speed=float(np.linalg.norm(velocity[:2]))
        if residual>max(1.,2*track.uncertainty) or speed<.5 or speed>15:return
        self.velocity=.7*velocity+.3*self.velocity
        self.velocity[2]=np.clip(self.velocity[2],-1.5,1.5)
        rate=float((velocity[0]*accel[1]-velocity[1]*accel[0])/max(speed**2,.25))
        limit=math.radians(config['max_turn_rate_deg_s'])
        self.turn_rate=float(np.clip(rate,-limit,limit))
        if abs(self.turn_rate)>math.radians(1.):self.model='constant_turn_rate'

    def predict(self, now, horizon, curved=True):
        if self.position is None:return None
        elapsed=max(0.,now-self.at)+horizon
        w=self.turn_rate if curved else 0.
        point=self.position.copy()
        if abs(w)<1e-5:
            point[:2]+=self.velocity[:2]*elapsed
        else:
            vx,vy=self.velocity[:2];s,c=math.sin(w*elapsed),math.cos(w*elapsed)
            point[:2]+=np.array([(vx*s+vy*(c-1))/w,(vx*(1-c)+vy*s)/w])
        point[2]+=self.velocity[2]*min(elapsed,self.vertical_horizon)
        return point

    def direction(self, now, horizon=0., curved=True):
        velocity=rotate(self.velocity,(self.turn_rate if curved else 0.)*(max(0.,now-self.at)+horizon))
        speed=np.linalg.norm(velocity[:2])
        return velocity[:2]/speed if speed>.5 else None


class MissionPlanner:
    def __init__(self, roles, count):
        self.roles,self.count=roles,count
        self.predictor=TrajectoryPredictor()
        self.plans={}
        self.last_update=-100.
        self.version=None

    def update(self,reports,track,now,primary):
        changed=self.version!=self.roles.version
        if not changed and now-self.last_update<.25:return
        self.last_update,self.version=now,self.roles.version
        if changed:self.plans.clear()
        config=self.roles.config['tactics']
        self.predictor.update(track,config)
        shared=track.summary(now)
        reliable=shared['available'] and shared['confidence']>=config['min_confidence'] and shared['age_s']<=config['max_track_age_s']
        for index,r in enumerate(reports[:3]):
            task=self.roles.role(index)['task']
            # Observation leadership is independent of mission assignment.
            # Temporary pursuit is used only when the primary aircraft is unhealthy.
            if r['id']==primary and index!=0:task='tracking_relay'
            if task in ('tracking','tracking_relay'):
                goal=None if not shared['available'] else track.position+track.velocity*(now-track.observed_at+.5)+rotate(self.roles.role(0)['offset'],math.atan2(track.velocity[1],track.velocity[0]))
                self.plans[index]={'kind':task,'status':'tracking' if goal is not None else 'waiting_observation','point':goal,'feedforward':track.velocity.copy() if goal is not None else np.zeros(3),'face':None}
            elif not reliable:
                self.plans[index]={'kind':task,'status':'waiting_observation','point':None,'feedforward':np.zeros(3),'face':None,'reason':'shared_track_stale_or_uncertain'}
            elif task=='front_block':
                self._block(index,r,now,config)
            elif task=='trajectory_intercept':
                self._intercept(index,r,now,config)

    def _samples(self,r,now,config,horizons,curved,lateral=0.,front=0.):
        candidates=[]
        for horizon in np.arange(horizons[0],horizons[1]+.01,.25):
            predicted=self.predictor.predict(now,float(horizon),curved)
            direction=self.predictor.direction(now,float(horizon),curved)
            if direction is None:continue
            point=predicted.copy()
            point[:2]+=front*direction+lateral*np.array([-direction[1],direction[0]])
            eta=arrival_time(r,point,config['max_speed_mps'],config['acceleration_mps2'])
            speed=max(.5,float(np.linalg.norm(self.predictor.velocity[:2])))
            target_eta=float(horizon)+front/speed
            candidates.append({'point':point,'predicted':predicted,'direction':direction,'horizon_s':float(horizon),
                'own_eta_s':eta,'target_eta_s':target_eta,'arrival_margin_s':target_eta-eta})
        return candidates

    def _block(self,index,r,now,config):
        plan=self.plans.get(index)
        current=self.predictor.predict(now,0.,False)
        keep=False
        if plan and plan.get('point') is not None and plan['kind']=='front_block':
            passed=float((current[:2]-plan['point'][:2])@plan['direction'])>config['block_passed_m']
            remaining=max(0.,plan['deadline']-now)
            deviation=float(np.linalg.norm(self.predictor.predict(now,remaining,False)[:2]-plan['point'][:2]))
            direction=self.predictor.direction(now,0.,False)
            turn=180. if direction is None else math.degrees(math.acos(float(np.clip(direction@plan['direction'],-1,1))))
            keep=not passed and turn<=config['block_replan_heading_deg'] and (now-plan['planned_at']<config['block_commit_s'] or deviation<config['block_replan_deviation_m']) and remaining>-.1
        if not keep:
            choices=self._samples(r,now,config,config['block_horizon_s'],False,config['block_side_m'],config['block_front_m'])
            if not choices:
                self.plans[index]={'kind':'front_block','status':'waiting_observation','point':None,'feedforward':np.zeros(3),'face':None,'reason':'target_direction_unavailable'};return
            feasible=[c for c in choices if c['arrival_margin_s']>=config['block_reserve_s']]
            selected=min(feasible,key=lambda c:c['target_eta_s']) if feasible else max(choices,key=lambda c:c['arrival_margin_s'])
            plan={**selected,'kind':'front_block','planned_at':now,'deadline':now+selected['target_eta_s'],'prediction_model':'constant_velocity',
                  'feedforward':np.zeros(3),'face':None}
            self.plans[index]=plan
        plan['own_eta_s']=arrival_time(r,plan['point'],config['max_speed_mps'],config['acceleration_mps2'])
        plan['target_eta_s']=max(0.,plan['deadline']-now)
        plan['arrival_margin_s']=plan['target_eta_s']-plan['own_eta_s']
        plan['feasible']=plan['arrival_margin_s']>=0
        on_station=np.linalg.norm(r['position']-plan['point'])<=config['hold_radius_m']
        plan['status']='block_hold' if on_station else 'block_transit' if plan['feasible'] else 'block_arrive_late'
        plan['point'][2]=current[2] if on_station else self.predictor.predict(now,min(plan['target_eta_s'],self.predictor.vertical_horizon),False)[2]
        plan['face']=current if on_station else None

    def _intercept(self,index,r,now,config):
        choices=self._samples(r,now,config,config['intercept_horizon_s'],True,config['intercept_side_m'])
        if not choices:
            self.plans[index]={'kind':'trajectory_intercept','status':'waiting_observation','point':None,'feedforward':np.zeros(3),'face':None,'reason':'target_direction_unavailable'};return
        feasible=[c for c in choices if c['arrival_margin_s']>=0]
        selected=min(feasible,key=lambda c:c['target_eta_s']) if feasible else max(choices,key=lambda c:c['arrival_margin_s'])
        current=self.predictor.predict(now,0.,True)
        in_window=np.linalg.norm(r['position']-current)<config['intercept_window_m']
        if in_window:
            direction=self.predictor.direction(now,0.,True)
            near=current+self.predictor.velocity*.3
            near[:2]+=config['intercept_side_m']*np.array([-direction[1],direction[0]])
            selected={**selected,'point':near,'horizon_s':.3,'target_eta_s':.3,
                      'own_eta_s':arrival_time(r,near,config['max_speed_mps'],config['acceleration_mps2'])}
            selected['arrival_margin_s']=selected['target_eta_s']-selected['own_eta_s']
        self.plans[index]={**selected,'kind':'trajectory_intercept','planned_at':now,'deadline':now+selected['target_eta_s'],
            'feasible':bool(feasible),'prediction_model':self.predictor.model,
            'status':'intercept_window' if in_window else 'intercept_transit' if feasible else 'intercept_unreachable',
            'face':current if in_window else None,
            'feedforward':self.predictor.velocity.copy() if in_window else np.zeros(3)}

    def details(self,index,now):
        plan=self.plans.get(index,{})
        result={key:plan.get(key) for key in ('kind','status','feasible','prediction_model','horizon_s','own_eta_s','target_eta_s','arrival_margin_s','reason') if key in plan}
        for key,value in list(result.items()):
            if isinstance(value,(float,np.floating)):result[key]=round(float(value),2)
        point=plan.get('point')
        result['goal_xyz_m']=None if point is None else np.round(point,2).tolist()
        result['planning_age_s']=round(max(0.,now-plan.get('planned_at',now)),2)
        return result

    def motion(self,index):
        plan=self.plans.get(index,{})
        return {'feedforward':plan.get('feedforward',np.zeros(3)),'face':plan.get('face'),
                'max_speed_mps':self.roles.config['tactics']['max_speed_mps'],
                'mode':plan.get('status','role_navigation')}
