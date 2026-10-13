"""Observer overlays. Estimated trajectories never enter the perception images."""
import math
import numpy as np

EFFECT_SECONDS = 1.2


def prefer_active_shot(previous,candidate,now):
    # A rapid cooldown/rejected request must not erase a just-fired beam or hit.
    if previous and previous['cooldown_ok'] and not candidate['cooldown_ok'] and now-previous['time_s']<=EFFECT_SECONDS:
        return previous
    return candidate


def shot_status(shot, now, latest=None):
    if latest and (not shot or now-shot['time_s']>EFFECT_SECONDS):
        shot=latest
    if not shot:
        return None
    age=max(0.,now-shot['time_s'])
    result={'time_s':shot['time_s'],'age_s':round(age,3),'number':shot['number'],
            'state':'blocked' if shot.get('blocked') else 'cooldown' if not shot['cooldown_ok'] else 'hit' if shot['hit'] else 'miss'}
    if latest and latest['time_s']>shot['time_s']:
        result['attempt_state']='blocked' if latest.get('blocked') else 'cooldown' if not latest['cooldown_ok'] else 'hit' if latest['hit'] else 'miss'
    return result


def remember_shot(result, position, yaw, target, now, number, shot_range=25.):
    start=np.asarray(position).copy()+.7*np.array([math.cos(yaw),math.sin(yaw),0.])
    end=np.asarray(target).copy() if result['hit'] else start+shot_range*np.array([math.cos(yaw),math.sin(yaw),0.])
    return {**result,'time_s':now,'number':number,'start':start,'end':end}


class WorldProjector:
    def __init__(self,camera,width,height,fovy=50.):
        # Snapshot camera arrays: another renderer update may reuse MuJoCo memory.
        self.pos=np.asarray(camera.pos).copy()
        self.forward=np.asarray(camera.forward).copy()
        self.up=np.asarray(camera.up).copy()
        self.right=np.cross(self.forward,self.up)
        self.width,self.height=width,height
        self.focal=height/(2*math.tan(math.radians(fovy/2)))

    def __call__(self,point):
        delta=np.asarray(point)-self.pos
        depth=float(delta@self.forward)
        if depth<=.1:return None
        xy=(self.width/2+float(delta@self.right)/depth*self.focal,
            self.height/2-float(delta@self.up)/depth*self.focal)
        if not np.all(np.isfinite(xy)) or max(abs(xy[0])/self.width,abs(xy[1])/self.height)>8:return None
        return xy


def world_trajectory(track,predictor,now):
    shared=track.summary(now)
    if not shared['available'] or predictor.position is None:
        return {'history':[],'predictions':[],'current':None}
    current=predictor.predict(now,0.)
    history=[np.asarray(p).copy() for at,p in track.history if now-at<=3.][-20:]
    history.append(current)
    return {'history':history,'current':current,
            'predictions':[{'point':predictor.predict(now,h),'horizon_s':h,
                            'confidence':shared['confidence']*math.exp(-h/3)} for h in (1.,2.,3.)]}


def camera_overlay(project,trajectory,shot,now):
    overlay={'trajectory_overlay':{
        'history_xy':[project(p) for p in trajectory['history']],
        'predicted_xy':[{'xy':project(p['point']),'horizon_s':p['horizon_s'],'confidence':p['confidence']}
                        for p in trajectory['predictions']]}}
    if shot and 0<=now-shot['time_s']<=EFFECT_SECONDS:
        overlay['shot_effect']={**shot,'age_s':now-shot['time_s'],
            'display_duration_s':EFFECT_SECONDS,'trace_duration_s':.45,
            'start_xy':project(shot['start']) if shot.get('start') is not None else None,
            'end_xy':project(shot['end']) if shot.get('end') is not None else None}
    return overlay
