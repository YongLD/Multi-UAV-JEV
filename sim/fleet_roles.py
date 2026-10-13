"""Validated, reloadable Markdown roles; only the current role enters a model request."""
import copy
import hashlib
import json
from pathlib import Path
import re

FLIGHT_ACTIONS = ('turn_left', 'turn_right', 'climb', 'descend', 'forward')


class RoleBook:
    def __init__(self, path=None):
        self.path = Path(path or Path(__file__).with_name('role.md'))
        self.version = self.error = self.mtime = None
        self.reload()
        if self.version is None:
            raise ValueError('Cannot load role.md: '+str(self.error))

    def reload(self):
        try:
            stamp = self.path.stat().st_mtime_ns
            if stamp == self.mtime:
                return False
            text = self.path.read_text(encoding='utf-8')
            config = json.loads(re.search(r'```json\s*([\s\S]*?)```', text).group(1))
            sections = dict(re.findall(r'### (P-0[1-4])\s*\n([\s\S]*?)(?=\n### |\Z)', text))
            for key in ('P-01', 'P-02', 'P-03', 'P-04'):
                role = config['roles'][key]
                assert isinstance(role['name'], str) and 0 < len(role['name']) <= 40
                assert len(role['offset']) == 3 and all(type(v) in (int,float) and abs(v) <= 100 for v in role['offset'])
                assert len(role['start'])==3 and all(type(v) in (int,float) and abs(v)<=120 for v in role['start']) and 3<=role['start'][2]<=36
                assert role['task'] in ('tracking','front_block','trajectory_intercept','support_rescue')
                assert (role['task']=='support_rescue') == (key=='P-04')
                assert 20 < len(sections[key]) <= 2600
            rescue = config['rescue']
            assert type(rescue['auto_authorize']) is bool
            limits = {'stuck_seconds':(.5,10), 'tilt_degrees':(30,85), 'stand_off_m':(5,15),
                      'command_range_m':(10,40), 'stable_seconds':(.5,5), 'timeout_seconds':(3,30)}
            for key, (lo,hi) in limits.items():
                assert type(rescue[key]) in (int,float) and lo <= rescue[key] <= hi
            self_rescue=config.get('self_rescue',{})
            limits={'blocked_seconds':(.5,10),'timeout_seconds':(5,30),'stable_seconds':(.5,5),
                    'escape_distance_m':(.5,5),'escape_speed_mps':(.3,2),'scan_rate_deg_s':(10,45),
                    'retry_seconds':(.5,5),'max_attempts':(1,5)}
            for key,value in self_rescue.items():
                assert key in limits and type(value) in (int,float) and limits[key][0]<=value<=limits[key][1]
                if key=='max_attempts':assert type(value) is int
            tactics=config['tactics']
            limits={'max_speed_mps':(1,6.3),'acceleration_mps2':(.5,3),'min_confidence':(.1,.9),
                'max_track_age_s':(.2,3),'turn_window_s':(.6,5),'max_turn_rate_deg_s':(1,45),
                'vertical_horizon_s':(.5,3),
                'block_front_m':(4,20),'block_side_m':(-12,12),'block_reserve_s':(0,4),'block_commit_s':(.5,8),
                'block_passed_m':(1,10),'block_replan_deviation_m':(5,30),'block_replan_heading_deg':(10,60),'hold_radius_m':(1,5),
                'intercept_side_m':(-12,12),'intercept_window_m':(5,20)}
            for key,(lo,hi) in limits.items():assert type(tactics[key]) in (int,float) and lo<=tactics[key]<=hi
            for key in ('block_horizon_s','intercept_horizon_s'):
                pair=tactics[key];assert len(pair)==2 and all(type(v) in (int,float) for v in pair) and .5<=pair[0]<pair[1]<=15
            self.config, self.sections = config, sections
            self.version = hashlib.sha256(text.encode()).hexdigest()[:12]
            self.mtime, self.error = stamp, None
            return True
        except (OSError, ValueError, AttributeError, KeyError, AssertionError, TypeError) as exc:
            self.error = type(exc).__name__+': '+str(exc)
            return False

    def role(self, index):
        return self.config['roles'][f'P-{index+1:02d}']

    def questions(self, base, index):
        question = copy.deepcopy(base)
        item = question['action']
        goals={'tracking':'Maintain continuous target tracking and supply observations to the two mission aircraft.',
               'front_block':'Reach and hold a forward blockade position BEFORE the target arrives.',
               'trajectory_intercept':'Predict the target trajectory and reach the earliest feasible future meeting point.',
               'support_rescue':'Protect team communication and restore unhealthy teammates safely.'}
        item['instructions']['goal'] = goals[self.role(index)['task']]
        item['instructions']['rules'] = (
            'Choose ONE action. Avoid contact and unknown space. Positive bearing is LEFT and height is UP. '
            'Navigation heading and altitude use navigation_goal; firing uses actual target aim only. '
            'track_status=shared means a teammate sees the target; it is not local visibility. '
            'Continuous navigation and safety are controller-owned. Follow your role below.\n'+self.sections[f'P-{index+1:02d}'].strip())
        item['criteria'] = {
            'turn_left': 'Navigation goal bearing >12deg, or assigned left search/clear escape.',
            'turn_right': 'Navigation goal bearing <-12deg, or assigned right search/clear escape.',
            'climb': 'Navigation goal height >1m and upper route clear; or clear upward recovery.',
            'descend': 'Navigation goal height <-1m and lower route clear. Never descend blindly.',
            'forward': 'Navigation goal heading and height aligned, route clear; continue current role.',
        }
        if index == 3:
            item['criteria']={key:'ONLY when self_rescue is inactive and rescue_assignment is absent. '+value for key,value in item['criteria'].items()}
            item['criteria']['rescue'] = 'When self_rescue is inactive and rescue_assignment exists: authorize teammate assistance. Controller owns approach and recipient motor recovery. Never fire.'
            item['criteria']['self_rescue'] = 'HIGHEST PRIORITY when your health is stuck/unstable or self_rescue.active: recover YOUR OWN aircraft. Controller brakes, rights attitude, scans fresh depth and escapes through measured clear air. Teammate rescue waits. Never reset pose or bypass safety.'
            item['instructions']['effects'] = 'Turns change navigation heading; climb/descend change navigation altitude; forward continues navigation; rescue assists teammates; self_rescue authorizes own motor-only recovery, observed escape and bounded retries.'
        else:
            item['criteria']['fire'] = 'Local target visible, actual aim range<=25m, abs(bearing)<=40deg, abs(height)<=1m, cooldown=0; unhealthy aircraft cannot fire.'
        return question
