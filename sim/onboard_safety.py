"""Small camera-derived 3D clearance and action forecasts, without world poses.

All queries use a depth image in the stabilized onboard camera and local IMU /
odometry velocities. Forecasts describe risk; they never select or mask actions.
"""
from __future__ import annotations

import math
import numpy as np
from duel_tactics import TURN_DEG

BEARINGS = (60, 40, 20, 0, -20, -40, -60)  # +left, -right
LAYERS = (("above", 2.), ("level", 0.), ("below", -2.))
BRAKE_ACCEL = 2.0
BODY_RADIUS = .75
MARGIN = 1.2
HORIZON_S = 2.0
SAMPLE_OFFSETS = np.array([[0, 0, 0], [0, .75, 0], [0, -.75, 0],
                           [0, 0, .5], [0, 0, -.5],
                           [0, .55, .4], [0, -.55, .4],
                           [0, .55, -.4], [0, -.55, -.4]])


def wrap(x):
    return (x+math.pi) % (2*math.pi)-math.pi


class DepthSafety:
    def __init__(self, fovy=110., tilt=-12., nose=.28, max_range=70.):
        self.tan_v = math.tan(math.radians(fovy/2))
        self.tilt = math.radians(tilt)
        self.nose = nose
        self.max_range = max_range
        self.depth = self.ignored = None
        self.points = np.empty((0, 3))
        self.grid = None

    def observe(self, depth, segmentation, ignored_ids):
        self.depth = np.asarray(depth)
        self.ignored = np.isin(segmentation, ignored_ids)
        self.h, self.w = depth.shape
        self.tan_h = self.tan_v*self.w/self.h
        yy, xx = np.mgrid[:self.h, :self.w]
        u = (2*xx/(self.w-1)-1)*self.tan_h
        v = (1-2*yy/(self.h-1))*self.tan_v
        c, s = math.cos(self.tilt), math.sin(self.tilt)
        # Camera depth is axial Z, not radial range. This pinhole backprojection
        # includes the real -12 degree camera tilt and nose offset.
        points = np.stack([depth*(c-s*v)+self.nose, -depth*u, depth*(s+c*v)], axis=-1)
        solid = (segmentation >= 0) & ~self.ignored & (depth < self.max_range*.98)
        self.points = points[solid][::2]
        grid = {}
        for label, dz in LAYERS:
            grid[label] = [self.corridor(math.radians(b), dz) for b in BEARINGS]
        self.grid = grid

    def query(self, points):
        """Return covered pixels and obstruction at each local volume sample."""
        points = np.asarray(points)
        x, y, z = points[..., 0]-self.nose, points[..., 1], points[..., 2]
        c, s = math.cos(self.tilt), math.sin(self.tilt)
        axial, up = c*x+s*z, -s*x+c*z
        safe_axial = np.maximum(axial, .01)
        col = (-y/safe_axial/self.tan_h+1)*(self.w-1)/2
        row = (1-up/safe_axial/self.tan_v)*(self.h-1)/2
        covered = (axial > .15) & (col >= 0) & (col < self.w-1) & (row >= 0) & (row < self.h-1)
        ix = np.clip(np.nan_to_num(col), 0, self.w-2).astype(int)
        iy = np.clip(np.nan_to_num(row), 0, self.h-2).astype(int)
        # Conservative four-pixel sampling avoids tiny gaps at a mesh edge.
        ranges = np.minimum.reduce([self.depth[iy, ix], self.depth[iy+1, ix],
                                    self.depth[iy, ix+1], self.depth[iy+1, ix+1]])
        # Space in FRONT of an occluding target is observed free; beyond that
        # target it is unknown. Dropping the whole target pixel would falsely
        # mark the approach corridor unknown and prevent closing the range.
        covered &= ~self.ignored[iy, ix] | (axial < ranges-.15)
        blocked = covered & (ranges+.15 < axial)
        return covered, blocked

    def corridor(self, bearing, dz=0.):
        distances = np.arange(1.5, 35.1, 1.)
        centers = np.stack([distances*math.cos(bearing), distances*math.sin(bearing),
                            dz*np.minimum(distances/4, 1)], axis=-1)
        covered, blocked = self.query(centers[:, None, :]+SAMPLE_OFFSETS)
        coverage = covered.mean(axis=1)
        danger = blocked.any(axis=1)
        # Unknown space cannot be turned into a 70m clear corridor.
        first_unknown = np.flatnonzero(coverage < .75)
        first_hit = np.flatnonzero(danger)
        stop_index = min(first_unknown[0] if first_unknown.size else len(distances),
                         first_hit[0] if first_hit.size else len(distances))
        observed_to = 0. if stop_index == 0 else float(distances[min(stop_index, len(distances)-1)])
        hit_distance = float(distances[first_hit[0]]) if first_hit.size else None
        unknown_first = first_unknown.size and (not first_hit.size or first_unknown[0] <= first_hit[0])
        return {"clearance_m": None if unknown_first and stop_index == 0 else round(observed_to, 1),
                "status": "unknown" if unknown_first else "blocked" if first_hit.size else "clear_to_limit",
                "obstacle_at_m": hit_distance,
                "coverage": round(float(coverage[:max(1, stop_index+1)].mean()), 2)}

    def forecast(self, action, own):
        v = np.asarray(own["velocity_body_mps"], dtype=float)
        position = np.zeros(3)
        yaw = 0.
        yaw_rate = math.radians(float(own.get('yaw_rate_deg_s', 0.)))
        target_yaw = math.radians(own['heading_error_deg'])
        altitude_delta = own['altitude_setpoint_m']-own['altitude_m']
        decision_delay = min(1.2, max(0., float(own.get('last_decision_latency_s', .4))))
        action_applied = False
        speed = float(own['speed_setpoint_mps'])
        covered_steps, checked_steps = 0, 0
        collision_at = None
        clearance = None
        for i in range(1, 17):
            dt = HORIZON_S/16
            if not action_applied and i*dt >= decision_delay:
                # Existing commands continue while Jev is computing. The new
                # action is relative to the predicted pose at application time.
                if action in ('turn_left', 'turn_right'):
                    target_yaw = yaw+math.radians(TURN_DEG if action == 'turn_left' else -TURN_DEG)
                elif action in ('climb', 'descend'):
                    altitude_delta = position[2]+(2. if action == 'climb' else -2.)
                elif action == 'forward':
                    target_yaw = yaw
                    altitude_delta = position[2]
                action_applied = True
            desired_yaw_rate = float(np.clip(3*wrap(target_yaw-yaw), -math.radians(60), math.radians(60)))
            yaw_rate += float(np.clip(desired_yaw_rate-yaw_rate, -math.radians(120)*dt, math.radians(120)*dt))
            yaw += yaw_rate*dt
            desired = np.array([speed*math.cos(yaw), speed*math.sin(yaw),
                                np.clip(1.1*(altitude_delta-position[2]), -2., 2.)])
            change = desired-v
            n = np.linalg.norm(change[:2])
            if n > BRAKE_ACCEL*dt:
                change[:2] *= BRAKE_ACCEL*dt/n
            change[2] = np.clip(change[2], -2*dt, 2*dt)
            next_v = v+change
            position += (v+next_v)*dt/2
            v = next_v
            if np.linalg.norm(position) < 1.:
                continue
            covered, blocked = self.query(position[None, :]+SAMPLE_OFFSETS)
            checked_steps += 1
            covered_steps += int(covered.mean() >= .75)
            if blocked.any() and collision_at is None:
                collision_at = round(i*dt, 2)
            if self.points.size:
                distance = float(np.linalg.norm(self.points-position, axis=1).min())-BODY_RADIUS
                clearance = min(clearance, distance) if clearance is not None else distance
        return {"predicted_contact_s": collision_at,
                "min_clearance_m": round(clearance, 2) if clearance is not None else None,
                "observed_path_fraction": round(covered_steps/checked_steps, 2) if checked_steps else 0.,
                "outside_view_possible": covered_steps < checked_steps or not checked_steps}

    def describe(self, own, actions):
        v = np.asarray(own['velocity_body_mps'], dtype=float)
        horizontal_speed = float(np.linalg.norm(v[:2]))
        flow_bearing = math.atan2(v[1], v[0]) if horizontal_speed > .2 else 0.
        flow = self.corridor(flow_bearing)
        reaction = max(.3, float(own.get('last_decision_latency_s', .4)))+.15+.066
        result = {'source': 'onboard_depth_and_odometry', 'horizon_s': HORIZON_S,
                  'grid_bearings_deg': list(BEARINGS), 'grid': self.grid,
                  'body_clearance_radius_m': BODY_RADIUS,
                  'velocity_bearing_deg': round(math.degrees(flow_bearing), 1),
                  'velocity_path': flow,
                  'braking_distance_m': round(horizontal_speed*reaction+horizontal_speed**2/(2*BRAKE_ACCEL)+MARGIN, 2),
                  'reaction_budget_s': round(reaction, 2),
                  'actions': {a: self.forecast(a, own) for a in actions},
                  'nearest_obstacle_m': None, 'nearest_obstacle_bearing_deg': None,
                  'predicted_contact_if_velocity_continues_s': None}
        if self.points.size:
            norms = np.linalg.norm(self.points, axis=1)
            p = self.points[int(norms.argmin())]
            result['nearest_obstacle_m'] = round(float(norms.min())-BODY_RADIUS, 2)
            result['nearest_obstacle_bearing_deg'] = round(math.degrees(math.atan2(p[1], p[0])), 1)
            if np.linalg.norm(v) > .2:
                times = (self.points@v)/float(v@v)
                distance = np.linalg.norm(self.points-times[:, None]*v, axis=1)
                approaching = (times > 0) & (times <= HORIZON_S) & (distance < BODY_RADIUS+.4)
                if approaching.any():
                    result['predicted_contact_if_velocity_continues_s'] = round(float(times[approaching].min()), 2)
        return result
