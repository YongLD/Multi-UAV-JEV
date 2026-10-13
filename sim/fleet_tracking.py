"""Communication of camera observations and own odometry, with no enemy truth input."""
from collections import deque
import math
import numpy as np
from onboard_target import rotate, angles

COUNTS = (1, 2, 4)
COLORS = ((255, 181, 82), (105, 206, 231), (180, 166, 241), (131, 223, 180))
ROLES = ('持续跟踪', '前方堵截', '轨迹预测拦截', '通信保障与救助')
MAX_TRACK_AGE = 3.0


def validate_count(value):
    if type(value) is not int or value not in COUNTS:
        raise ValueError('count must be 1, 2 or 4')
    return value


class SharedTargetTrack:
    """Fuse synchronized detector measurements; never re-broadcast predictions as measurements."""
    def __init__(self):
        self.position = None
        self.velocity = np.zeros(3)
        self.observed_at = None
        self.uncertainty = 0.0
        self.confidence = 0.0
        self.sources = []
        self.visible_sources = []
        self.history = deque(maxlen=60)
        self.rejected = 0

    def update(self, reports, t):
        candidates = []
        for report in reports:
            target = report['target']
            if not target.get('visible') or target.get('relative_position_m') is None:
                continue
            position = np.asarray(report['position']) + rotate(target['relative_position_m'], report['yaw'])
            uncertainty = max(.08, float(target.get('position_uncertainty_m') or 1.))
            confidence = float(target.get('track_confidence', 0.))
            if confidence < .05 or not np.all(np.isfinite(position)):
                continue
            relative_velocity = target.get('relative_velocity_mps')
            velocity = (np.asarray(report['velocity']) + rotate(relative_velocity, report['yaw'])
                        if relative_velocity is not None else None)
            candidates.append((report['id'], position, uncertainty, confidence, velocity))
        self.visible_sources = []
        if candidates:
            anchor = max(candidates, key=lambda p: p[3]/p[2])
            accepted = [p for p in candidates if np.linalg.norm(p[1]-anchor[1]) <= max(5., 3*(p[2]+anchor[2]))]
            self.rejected += len(candidates)-len(accepted)
            weights = np.array([p[3]/p[2]**2 for p in accepted])
            position = np.average([p[1] for p in accepted], axis=0, weights=weights)
            velocities = [(p[4], weights[i]) for i, p in enumerate(accepted) if p[4] is not None]
            if velocities:
                self.velocity = np.average([v for v, _ in velocities], axis=0, weights=[w for _, w in velocities])
            elif self.position is not None and .02 <= t-self.observed_at <= .5:
                self.velocity = .7*self.velocity+.3*(position-self.position)/(t-self.observed_at)
            speed = np.linalg.norm(self.velocity)
            if speed > 15:
                self.velocity *= 15/speed
            spread = max(float(np.linalg.norm(p[1]-position)) for p in accepted)
            self.uncertainty = max(.08, float(math.sqrt(1/sum(weights))) + spread)
            self.confidence = max(p[3] for p in accepted)
            self.position, self.observed_at = position, t
            self.sources = [p[0] for p in accepted]
            self.visible_sources = self.sources.copy()
            self.history.append((t, position.copy()))
        return self.summary(t)

    def summary(self, t):
        age = None if self.observed_at is None else max(0., t-self.observed_at)
        available = age is not None and age <= MAX_TRACK_AGE
        return {'available': available, 'visible_sources': self.visible_sources,
                'sources': self.sources if available else [],
                'age_s': None if age is None else round(age, 2),
                'confidence': round(self.confidence*math.exp(-age/1.5), 2) if available else 0.,
                'uncertainty_m': round(self.uncertainty + age*.8, 2) if available else None,
                'mode': 'observed' if self.visible_sources else 'predicted' if available else 'search',
                'rejected_observations': self.rejected,
                'source': 'onboard_camera_measurements_shared_via_own_odometry'}

    def relative(self, report, t):
        local = dict(report['target'])
        shared = self.summary(t)
        if not shared['available']:
            return {**local, 'shared_available': False, 'information_source': 'local' if local.get('visible') else 'search'}
        age = t-self.observed_at
        position = self.position + self.velocity*age
        point = rotate(position-np.asarray(report['position']), -report['yaw'])
        relative_velocity = rotate(self.velocity-np.asarray(report['velocity']), -report['yaw'])
        confidence = shared['confidence']
        past = [{'age_s': round(t-at, 2), 'relative_xyz_m': np.round(rotate(p-report['position'], -report['yaw']), 2).tolist()}
                for at, p in list(self.history)[-5:]]
        predictions = [{'horizon_s': h, 'relative_xyz_m': np.round(point+rotate(self.velocity, -report['yaw'])*h, 2).tolist(),
                        'uncertainty_m': round(shared['uncertainty_m']+h*1.5, 2),
                        'confidence': round(confidence*math.exp(-h/3), 2)} for h in (1., 2., 3.)]
        return {**local, **angles(point), 'relative_position_m': np.round(point, 2).tolist(),
                'estimated_relative_position_m': np.round(point, 2).tolist(),
                'relative_velocity_mps': np.round(relative_velocity, 2).tolist(),
                'range_rate_mps': round(float(point@relative_velocity/max(np.linalg.norm(point), .01)), 2),
                'estimated_altitude_m': round(float(position[2]), 2),
                'track_confidence': confidence, 'position_uncertainty_m': shared['uncertainty_m'],
                'shared_available': bool(shared['visible_sources']), 'observation_sources': shared['sources'],
                'observation_age_s': shared['age_s'],
                'information_source': 'local+shared' if local.get('visible') else 'shared' if self.visible_sources else 'prediction',
                'trajectory': {'history': past, 'predictions': predictions, 'last_observation_age_s': age,
                               'trend': {'turn_rate_deg_s': 0., 'speed_acceleration_mps2': 0.,
                                         'target_velocity_mps': np.round(rotate(self.velocity, -report['yaw']), 2).tolist()}}}


def team_context(report, reports, track, t, index):
    peers = []
    for other in reports:
        if other['id'] == report['id']:
            continue
        relative = rotate(np.asarray(other['position'])-report['position'], -report['yaw'])
        peers.append({'id': other['id'], 'relative_xyz_m': np.round(relative, 2).tolist(),
                      'distance_m': round(float(np.linalg.norm(relative)), 2),
                      'relative_velocity_mps': np.round(rotate(np.asarray(other['velocity'])-report['velocity'], -report['yaw']), 2).tolist()})
    goal = None
    if track.summary(t)['available']:
        heading = math.atan2(track.velocity[1], track.velocity[0]) if np.linalg.norm(track.velocity[:2]) > .5 else report['yaw']
        offsets = ((-15., 0., 0.), (-12., 8., 2.), (-12., -8., -2.), (-23., 0., 4.))
        goal = rotate(track.position+track.velocity*(t-track.observed_at+.5)+rotate(offsets[index], heading)-report['position'], -report['yaw'])
    return {'drone_id': report['id'], 'role': ROLES[index], 'fleet_count': len(reports),
            'link': 'ideal_zero_latency_simulated_broadcast', 'peers': peers,
            'minimum_separation_m': 4., 'formation_goal_relative_m': None if goal is None else np.round(goal, 2).tolist(),
            'formation_bearing_deg': None if goal is None else angles(goal)['bearing_deg'],
            'formation_height_m': None if goal is None else round(float(goal[2]), 2)}


def separation_velocity(position, velocity, peers):
    """Fast local collision guard from communicated own poses and velocities."""
    correction = np.zeros(3)
    closest = None
    for peer in peers:
        delta = np.asarray(position)-peer['position']
        distance = float(np.linalg.norm(delta))
        closest = distance if closest is None else min(distance, closest)
        predicted = delta+.5*(np.asarray(velocity)-peer['velocity'])
        predicted_distance = float(np.linalg.norm(predicted))
        if distance < 6. or predicted_distance < 4.:
            # Never reverse the repulsion direction when projected paths cross.
            direction = delta/max(distance, .01)
            correction += direction*min(3., max(0., 6.-min(distance, predicted_distance)))
    norm = float(np.linalg.norm(correction))
    if norm > 3.:
        correction *= 3./norm
    return correction, closest
