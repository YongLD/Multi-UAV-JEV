"""P-04 motor-only recovery with measured escape, scanning and bounded retries."""
import math
import numpy as np

DEFAULTS={'blocked_seconds':2.,'timeout_seconds':18.,'stable_seconds':1.5,
          'escape_distance_m':2.,'escape_speed_mps':1.2,'scan_rate_deg_s':35.,
          'retry_seconds':2.,'max_attempts':3}


class SupportSelfRescue:
    def __init__(self,roles):
        self.roles=roles
        self.state='idle';self.attempt=0;self.completed=0;self.failed=0
        self.started=self.stable_since=self.blocked_since=None
        self.origin=None;self.scan_yaw=0.;self.scan_started=0.
        self.route=None;self.route_until=0.;self.retry_at=0.
        self.last_result=None;self.events=[];self.tilt=0.

    @property
    def config(self):return {**DEFAULTS,**self.roles.config.get('self_rescue',{})}

    @property
    def active(self):return self.state in ('righting','scan','escape','cooldown')

    @property
    def blocks_support(self):return self.active or self.state=='failed'

    def _event(self,now,result,**extra):
        self.events.append({'time_s':round(now,3),'drone_id':'P-04','type':'self_rescue_'+result,
                            'attempt':self.attempt,**extra})

    def _begin(self,report,own,now,source):
        self.attempt+=1;self.started=now;self.origin=np.array(report['position'],copy=True)
        self.stable_since=None;self.route=None
        self.tilt=float(own.get('tilt_deg',90.))
        self._scan(report,now)
        if self.tilt>=25:self.state='righting'
        self._event(now,'started',source=source)

    def _scan(self,report,now):
        self.state='scan';self.scan_yaw=report['yaw'];self.scan_started=now;self.route=None

    def update(self,report,own,health,now):
        c=self.config;self.tilt=float(own.get('tilt_deg',max(abs(own['roll_deg']),abs(own['pitch_deg']))))
        blocked=(own.get('navigation_distance_m',0) is not None and own.get('navigation_distance_m',0)>5
                 and own['speed_mps']<.3 and own.get('safety_limited',False)
                 and own.get('requested_navigation_speed_mps',0)>.5)
        if blocked:
            if self.blocked_since is None:self.blocked_since=now
        else:self.blocked_since=None
        if not report.get('alive',True):
            if self.active:
                self.failed+=1;self.last_result={'result':'unavailable','time_s':round(now,3)}
                self._event(now,'failed',reason='unavailable')
            self.state='failed';return
        moved=0. if self.origin is None else float(np.linalg.norm(report['position']-self.origin))
        stable=(self.tilt<25 and health=='healthy' and moved>=c['escape_distance_m'])
        if (self.active or self.state=='failed') and stable:
            if self.stable_since is None:self.stable_since=now
            if now-self.stable_since>=c['stable_seconds']:
                self.completed+=1;self.state='recovered'
                self.last_result={'result':'recovered','time_s':round(now,3),'moved_m':round(moved,2)}
                self._event(now,'recovered',moved_m=round(moved,2));return
        else:self.stable_since=None
        if self.active:
            if self.state=='cooldown':
                if now>=self.retry_at:self._begin(report,own,now,'retry')
            elif now-self.started>=c['timeout_seconds']:
                self.failed+=1;self.last_result={'result':'timeout','time_s':round(now,3),'state':self.state}
                self._event(now,'failed',reason='timeout',phase=self.state)
                if self.attempt<c['max_attempts']:
                    self.state='cooldown';self.retry_at=now+c['retry_seconds']
                else:self.state='failed'
            elif self.tilt>=25:
                self.state='righting';self.route=None
            elif self.state=='righting':self._scan(report,now)
            return
        risk=health in ('stuck','unstable') or (self.blocked_since is not None and now-self.blocked_since>=c['blocked_seconds'])
        if self.state!='failed' and risk:
            self.attempt=0;self._begin(report,own,now,'health_guard')

    def request(self,report,own,health,now):
        if self.active:return True
        risk=health in ('stuck','unstable') or (
            own.get('front_clearance_m') is not None and own['front_clearance_m']<3
            and own['speed_mps']<.8 and (own.get('navigation_distance_m') or 0)>5)
        if report.get('alive',True) and self.state!='failed' and risk:
            self.attempt=0;self._begin(report,own,now,'model');return True
        return False

    def command(self,report,views,peers,now):
        from fleet_control import safe_velocity
        if self.state in ('righting','cooldown','failed'):
            return np.zeros(3),report['yaw'],'self_rescue_'+self.state,self.tilt>=25
        if self.state=='escape' and self.route is not None and now<self.route_until:
            velocity,_=safe_velocity(self.route,report,views,peers,now)
            if np.linalg.norm(velocity)>.3:
                return velocity,math.atan2(velocity[1],velocity[0]),'self_rescue_escape',False
            self._scan(report,now)
        best=(0.,np.zeros(3))
        for view in views:
            if now-view['time']>.45:continue
            for bearing in (-40,0,40):
                for dz in (0.,2.):
                    corridor=view['safety'].corridor(math.radians(bearing),dz)
                    clear=corridor['clearance_m'] or 0.
                    if corridor['status']=='unknown' or clear<3:continue
                    yaw=view['yaw']+math.radians(bearing)
                    direction=np.array([math.cos(yaw),math.sin(yaw),dz/4])
                    requested=direction/np.linalg.norm(direction)*self.config['escape_speed_mps']
                    velocity,_=safe_velocity(requested,report,views,peers,now)
                    score=min(clear,12.)*np.linalg.norm(velocity)
                    if np.linalg.norm(velocity)>.3 and score>best[0]:best=(score,velocity)
        if best[0]>0:
            self.state='escape';self.route=best[1];self.route_until=now+1.
            return self.route,math.atan2(self.route[1],self.route[0]),'self_rescue_escape',False
        if self.state!='scan':self._scan(report,now)
        heading=self.scan_yaw+math.radians(self.config['scan_rate_deg_s'])*(now-self.scan_started)
        return np.zeros(3),heading,'self_rescue_scan',False

    def summary(self,now):
        return {'active':self.active,'state':self.state,'attempt':self.attempt,
                'elapsed_s':round(max(0.,now-self.started),2) if self.active and self.started is not None else 0.,
                'completed':self.completed,'failed_attempts':self.failed,'last_result':self.last_result}

    def take_events(self):
        result=self.events;self.events=[];return result
