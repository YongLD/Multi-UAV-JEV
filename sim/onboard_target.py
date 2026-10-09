"""Relative target tracking from camera pixels and the chaser's own odometry.

No evader pose, map or scripted trajectory is accepted by this interface.
The current simulator's segmentation detector is ideal; depth and geometry
measurements still come from the actual onboard camera render buffers.
"""
from __future__ import annotations

import math
from collections import deque
import numpy as np


def rotate(vector, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c*vector[0]-s*vector[1], s*vector[0]+c*vector[1], vector[2]])


def angles(point):
    distance = float(np.linalg.norm(point))
    return {'bearing_deg': round(math.degrees(math.atan2(point[1], point[0])), 2),
            'elevation_deg': round(math.degrees(math.atan2(point[2], np.linalg.norm(point[:2]))), 2),
            'relative_height_m': round(float(point[2]), 2), 'range_m': round(distance, 2)}


class CameraTargetTrack:
    def __init__(self, fovy=110., tilt=0., nose=.28, marker_radius=1.3, max_range=70.):
        self.tan_v = math.tan(math.radians(fovy/2))
        self.tilt = math.radians(tilt)
        self.nose = nose
        self.radius = marker_radius
        self.max_range = max_range
        self.last = None
        self.target_velocity = None
        self.previous_own = None
        self.own_velocity = np.zeros(3)
        self.yaw_rate = 0.
        self.observations = deque(maxlen=100)

    def trajectory(self, own_position, yaw, t, confidence):
        """Observed history and CTRA extrapolation in ONE current own frame."""
        # Retain the last observed four-second segment during long occlusion.
        # Its growing ages make it a stale search memory, not a current track.
        latest_observed = self.observations[-1]['t'] if self.observations else t
        while self.observations and latest_observed-self.observations[0]['t'] > 4.:
            self.observations.popleft()
        if not self.observations:
            return None
        history = list(self.observations)
        indices = np.unique(np.linspace(0, len(history)-1, min(5, len(history))).astype(int))
        past = []
        for i in indices:
            observation = history[i]
            relative = rotate(observation['estimated_position']-own_position, -yaw)
            past.append({'age_s': round(t-observation['t'], 2), 'relative_xyz_m': np.round(relative, 2).tolist()})
        recent = [p for p in history if history[-1]['t']-p['t'] <= 1.2]
        velocity = self.target_velocity.copy() if self.target_velocity is not None else np.zeros(3)
        acceleration = np.zeros(3)
        residual = .2
        motion_span = recent[-1]['t']-recent[0]['t'] if len(recent)>1 else 0.
        if len(recent) >= 4 and motion_span >= .25:
            times = np.array([p['t']-history[-1]['t'] for p in recent])
            positions = np.array([p['estimated_position'] for p in recent])
            degree = 2 if len(recent) >= 7 and motion_span >= .6 else 1
            fit = np.polynomial.polynomial.polyfit(times, positions, degree)
            fitted = np.polynomial.polynomial.polyval(times, fit).T
            residual = float(np.sqrt(np.mean((positions-fitted)**2)))
            if residual < .4 and np.linalg.norm(fit[1]) < 15:
                velocity = fit[1]
                if degree == 2:
                    acceleration = np.clip(2*fit[2], -3., 3.)
            if degree == 2 and np.linalg.norm(velocity[:2]) > .7:
                # A Cartesian quadratic creates spurious acceleration on a
                # constant-speed bend. Fit a well-observed arc when available.
                center = positions[:, :2].mean(axis=0)
                xy = positions[:, :2]-center
                design = np.column_stack([2*xy, np.ones(len(xy))])
                if np.linalg.cond(design) < 3000:
                    circle, *_ = np.linalg.lstsq(design, np.sum(xy**2, axis=1), rcond=None)
                    origin_xy = center+circle[:2]
                    radius_squared = circle[2]+circle[:2]@circle[:2]
                    if radius_squared > 1:
                        radius = math.sqrt(radius_squared)
                        radii = np.linalg.norm(positions[:, :2]-origin_xy, axis=1)
                        phase = np.unwrap(np.arctan2(positions[:, 1]-origin_xy[1], positions[:, 0]-origin_xy[0]))
                        if radius < 250 and np.ptp(phase) > .2 and np.std(radii) < .06:
                            phase_fit = np.polynomial.polynomial.polyfit(times, phase, 2)
                            angular_speed = float(phase_fit[1])
                            arc_speed = abs(angular_speed)*radius
                            if .7 < arc_speed < 15 and abs(angular_speed) < math.radians(90):
                                direction = phase_fit[0]+math.copysign(math.pi/2, angular_speed)
                                velocity[:2] = arc_speed*np.array([math.cos(direction), math.sin(direction)])
                                along_accel = float(np.clip(2*phase_fit[2]*radius*np.sign(angular_speed), -3, 3))
                                acceleration[:2] = (along_accel*velocity[:2]/arc_speed
                                                    +angular_speed*np.array([-velocity[1], velocity[0]]))
                                residual = max(residual, float(np.std(radii)))
        speed = float(np.linalg.norm(velocity[:2]))
        heading = math.atan2(velocity[1], velocity[0])
        turn_rate = ((velocity[0]*acceleration[1]-velocity[1]*acceleration[0])/max(speed*speed, .5)
                     if speed > .7 else 0.)
        turn_rate = float(np.clip(turn_rate, -math.radians(90), math.radians(90)))
        tangential_accel = float(np.clip(velocity[:2]@acceleration[:2]/max(speed, .5), -3., 3.))
        dt = .1
        displacement = np.zeros(3)
        origin = history[-1]['estimated_position'].copy()
        # A short occlusion continues the camera-derived motion estimate, with
        # confidence falling. Beyond 3s only the history is returned.
        gap = t-history[-1]['t']
        predictions = []
        if gap <= 3.:
            horizons = (1., 2., 3.)
            checkpoints = [gap+h for h in horizons]
            clock = 0.
            for checkpoint, horizon in zip(checkpoints, horizons):
                while clock < checkpoint-1e-7:
                    step = min(dt, checkpoint-clock)
                    mid = clock+step/2
                    vxy = float(np.clip(speed+tangential_accel*mid, 0., 15.))
                    direction = heading+turn_rate*mid
                    vz = float(np.clip(velocity[2]+acceleration[2]*mid, -3., 3.))
                    displacement += step*np.array([vxy*math.cos(direction), vxy*math.sin(direction), vz])
                    clock += step
                relative = rotate(origin+displacement-own_position, -yaw)
                # Future route changes are unobserved. Increase the uncertainty
                # budget for longer extrapolation, especially on observed bends.
                uncertainty = (history[-1]['uncertainty']+residual*checkpoint
                               +(.9+.1*speed*abs(turn_rate))*checkpoint*checkpoint)
                predictions.append({'horizon_s': horizon, 'relative_xyz_m': np.round(relative, 2).tolist(),
                                    'uncertainty_m': round(uncertainty, 2),
                                    'confidence': round(confidence*min(1., max(motion_span, .15)/.8)*math.exp(-horizon/2.5), 2)})
        return {'frame': 'current_own_heading_frame', 'source': 'camera_history_and_own_odometry',
                'last_observation_age_s': round(gap, 2),
                'history': past, 'predictions': predictions,
                'trend': {'target_velocity_mps': np.round(rotate(velocity, -yaw), 2).tolist(),
                          'turn_rate_deg_s': round(math.degrees(turn_rate), 1),
                          'speed_acceleration_mps2': round(tangential_accel, 2),
                          'vertical_acceleration_mps2': round(float(acceleration[2]), 2)}}

    def measure(self, depth, mask):
        rows, cols = np.nonzero(mask & np.isfinite(depth) & (depth > .1) & (depth < self.max_range))
        if not len(rows):
            return None
        h, w = depth.shape
        u = (2*cols/(w-1)-1)*self.tan_v*w/h
        v = (1-2*rows/(h-1))*self.tan_v
        d = depth[rows, cols]
        c, s = math.cos(self.tilt), math.sin(self.tilt)
        points = np.stack([d*(c-s*v)+self.nose, -d*u, d*(s+c*v)], axis=-1)
        median = np.median(points, axis=0)
        camera_origin = np.array([self.nose, 0., 0.])
        ray = median-camera_origin
        point = median+self.radius*ray/max(float(np.linalg.norm(ray)), .001)
        uncertainty = .65
        # The target's visual detection marker has a calibrated size. Fit its
        # observed surface, instead of mistaking axial depth for radial range.
        # Centering the least squares problem avoids poor conditioning far away.
        if len(points) >= 6:
            mean = points.mean(axis=0)
            centered = points-mean
            a = np.column_stack([2*centered, np.ones(len(points))])
            if np.linalg.cond(a) < 3000:
                solution, *_ = np.linalg.lstsq(a, np.sum(centered**2, axis=1), rcond=None)
                radius_squared = solution[3]+solution[:3]@solution[:3]
                fitted = mean+solution[:3]
                if radius_squared > 0:
                    fitted_radius = math.sqrt(radius_squared)
                    residual = float(np.std(np.linalg.norm(points-fitted, axis=1)))
                    if abs(fitted_radius-self.radius) < .15 and residual < .06 and np.linalg.norm(fitted-median) < 2.:
                        point, uncertainty = fitted, max(.08, residual*3)
        if np.linalg.norm(point) > self.max_range:
            return None
        return point, len(points), uncertainty

    def update(self, depth, mask, own_position, yaw, t, *, camera_yaw=None, camera='front'):
        own_position = np.asarray(own_position, dtype=float)
        if self.previous_own is not None:
            old_t, old_position, old_yaw = self.previous_own
            dt = t-old_t
            if dt > .001:
                self.own_velocity = (own_position-old_position)/dt
                self.yaw_rate = ((yaw-old_yaw+math.pi) % (2*math.pi)-math.pi)/dt
        self.previous_own = (t, own_position.copy(), yaw)
        measurement = self.measure(depth, mask)
        if measurement:
            point, pixels, uncertainty = measurement
            if camera_yaw is not None:
                point = rotate(point, camera_yaw-yaw)
            if self.last is not None:
                dt = t-self.last['t']
                if .02 <= dt <= .5:
                    displacement = (rotate(point, yaw)-rotate(self.last['point'], self.last['yaw'])
                                    +own_position-self.last['own_position'])
                    velocity = displacement/dt
                    magnitude = float(np.linalg.norm(velocity))
                    if magnitude > 15:
                        velocity *= 15/magnitude
                    alpha = dt/(.35+dt)
                    self.target_velocity = (velocity if self.target_velocity is None
                                            else (1-alpha)*self.target_velocity+alpha*velocity)
                elif dt > .5:
                    self.target_velocity = None
                    self.observations.clear()
            relative_velocity = (rotate(self.target_velocity-self.own_velocity, -yaw)
                                 if self.target_velocity is not None else None)
            out = {'visible': True, 'camera': camera, 'front_visible': camera == 'front',
                   **angles(point), 'relative_position_m': np.round(point, 2).tolist(),
                   'relative_velocity_mps': np.round(relative_velocity, 2).tolist() if relative_velocity is not None else None,
                   'position_uncertainty_m': round(uncertainty, 2), 'pixels': pixels,
                   'range_rate_mps': round(float(point@relative_velocity/max(np.linalg.norm(point), .01)), 2) if relative_velocity is not None else None,
                   'bearing_rate_deg_s': None, 'elevation_rate_deg_s': None,
                   'unseen_for_s': 0., 'track_confidence': round(min(1., pixels/12)/(1+uncertainty), 2)}
            if relative_velocity is not None:
                r2 = max(float(point[:2]@point[:2]), .01)
                out['bearing_rate_deg_s'] = round(math.degrees((point[0]*relative_velocity[1]-point[1]*relative_velocity[0])/r2-self.yaw_rate), 2)
            self.observations.append({'t': t, 'estimated_position': own_position+rotate(point, yaw),
                                      'uncertainty': uncertainty})
            out['trajectory'] = self.trajectory(own_position, yaw, t, out['track_confidence'])
            out['estimated_altitude_m'] = round(float(own_position[2]+point[2]), 2)
            self.last = {'point': point.copy(), 'own_position': own_position.copy(), 'yaw': yaw,
                         't': t, 'out': dict(out)}
            return out
        out = {'visible': False, 'camera': None, 'front_visible': False,
               'bearing_deg': None, 'range_m': None, 'elevation_deg': None,
               'relative_height_m': None, 'range_rate_mps': None, 'bearing_rate_deg_s': None,
               'elevation_rate_deg_s': None, 'relative_position_m': None, 'relative_velocity_mps': None,
               'position_uncertainty_m': None, 'pixels': 0, 'track_confidence': 0.,
               'unseen_for_s': None if self.last is None else round(t-self.last['t'], 2),
               'last_seen': None}
        if self.last is not None:
            gap = t-self.last['t']
            memory_point = rotate(self.last['own_position']+rotate(self.last['point'], self.last['yaw'])
                                  -own_position, -yaw)
            out['search_memory'] = {'age_s': round(gap, 2),
                                    'last_observed_relative_xyz_m': np.round(memory_point, 2).tolist(),
                                    'bearing_deg': angles(memory_point)['bearing_deg'],
                                    'source': 'last_camera_observation_compensated_by_own_odometry'}
            if gap <= 3.:
                displacement = self.target_velocity*gap if self.target_velocity is not None else np.zeros(3)
                point = rotate(rotate(self.last['point'], self.last['yaw'])+displacement
                               -(own_position-self.last['own_position']), -yaw)
                out['estimated_relative_position_m'] = np.round(point, 2).tolist()
                if self.target_velocity is not None:
                    out['relative_velocity_mps'] = np.round(rotate(self.target_velocity-self.own_velocity, -yaw), 2).tolist()
                out['estimated_current_bearing_deg'] = angles(point)['bearing_deg']
                out['track_confidence'] = round(self.last['out']['track_confidence']*math.exp(-gap/1.5), 2)
                out['last_seen'] = {**self.last['out'], 'estimated_current_bearing_deg': out['estimated_current_bearing_deg']}
            out['trajectory'] = self.trajectory(own_position, yaw, t, out['track_confidence'])
        return out


def aim_at_execution(target, yaw_rate_deg_s, latency_s, heading_error_deg=None):
    """Camera-derived motion extrapolation in the own future heading frame."""
    values = target.get('relative_position_m') if target.get('visible') else target.get('estimated_relative_position_m')
    if values is None:
        return None
    point = np.asarray(values, dtype=float)
    velocity = target.get('relative_velocity_mps')
    horizon = min(1., max(0., float(latency_s)))
    if velocity is not None:
        point += np.asarray(velocity)*horizon
    yaw_change = math.radians(float(yaw_rate_deg_s))*horizon
    if heading_error_deg is not None and horizon > 0:
        # The old heading command is finite. Extrapolating a brief yaw-rate peak
        # for the whole latency would predict an endless turn and reverse aim.
        yaw_change = 0.
        rate = math.radians(float(yaw_rate_deg_s))
        dt = horizon/8
        for _ in range(8):
            desired = float(np.clip(3*(math.radians(heading_error_deg)-yaw_change), -math.radians(60), math.radians(60)))
            rate += float(np.clip(desired-rate, -math.radians(120)*dt, math.radians(120)*dt))
            yaw_change += rate*dt
    point = rotate(point, -yaw_change)
    values = angles(point)
    return {'bearing_deg': values['bearing_deg'], 'height_m': values['relative_height_m'],
            'range_m': values['range_m'], 'horizon_s': round(horizon, 2)}


def pursuit_point_at_execution(target, own_velocity_body, yaw_rate_deg_s, latency_s, heading_error_deg):
    """Adaptive camera-derived lead; this computes geometry, never an action."""
    predictions = (target.get('trajectory') or {}).get('predictions') or []
    if not predictions or predictions[0]['confidence'] < .2:
        return target.get('aim')
    trajectory = target['trajectory']
    trend = trajectory['trend']
    current = target.get('relative_position_m') if target.get('visible') else target.get('estimated_relative_position_m')
    if current is None:
        return target.get('aim')
    distance = float(np.linalg.norm(current))
    horizon = .5 if distance <= 30 else 1.
    if (distance > 40 and len(predictions) >= 2 and abs(trend['turn_rate_deg_s']) < 10
            and abs(trend['speed_acceleration_mps2']) < .5
            and trajectory['last_observation_age_s'] < .2
            and predictions[1]['confidence'] >= .35
            and predictions[1]['uncertainty_m'] < distance*.08):
        horizon = 1.5
    first = predictions[0]
    if horizon <= 1:
        point = (1-horizon)*np.asarray(current)+horizon*np.asarray(first['relative_xyz_m'])
        uncertainty = first['uncertainty_m']*horizon
        confidence = first['confidence']
    else:
        second = predictions[1]
        point = (2-horizon)*np.asarray(first['relative_xyz_m'])+(horizon-1)*np.asarray(second['relative_xyz_m'])
        uncertainty = max(first['uncertainty_m'], second['uncertainty_m'])
        confidence = min(first['confidence'], second['confidence'])
    # Reuse the current-command yaw prediction. Only own translation before
    # action execution is subtracted, so this points at the future enemy area.
    temporary = {'visible': True, 'relative_position_m': point.tolist(),
                 'relative_velocity_mps': (-np.asarray(own_velocity_body)).tolist()}
    lead = aim_at_execution(temporary, yaw_rate_deg_s, latency_s, heading_error_deg)
    lead.update(horizontal_horizon_s=horizon, vertical_horizon_s=horizon,
                confidence=confidence, uncertainty_m=round(uncertainty, 2))
    return lead
