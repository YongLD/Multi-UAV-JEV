# Copyright (c) 2026 the jev-drone authors
# Derived from RomanSlack/jev-drone (MIT); see licenses/jev-drone-MIT.txt.
"""Quadrotor controller + onboard-camera perception for the Skydio X2 arena.

Nothing here talks to an LLM. This is the fast, deterministic layer: it flies the
aircraft and turns camera pixels into a small symbolic scene description.
"""
import numpy as np
import mujoco

G = 9.81
IGNORE_BELOW_DEFAULT = 0.5   # None keeps everything below you as a threat (tunnels)
MJ_GEOM = mujoco.mjtObj.mjOBJ_GEOM


# --------------------------------------------------------------------------- #
# control
# --------------------------------------------------------------------------- #
class Pilot:
    """Cascaded velocity -> attitude -> motor-thrust controller."""

    def __init__(self, model):
        self.m = model
        self.mass = float(model.body_subtreemass[model.body("x2").id])
        self.inertia = model.body_inertia[model.body("x2").id].copy()
        self.kv = np.array([2.6, 2.6, 4.0])          # velocity -> accel
        self.kR = self.inertia * 190.0               # (unused; kept for reference)
        # Quaternion error is ~theta/2 where the geometric one was ~theta, so 2x the
        # old stiffness reproduces the original response. The tunnel overrides both
        # of these with much heavier damping; see tunnel.py.
        self.kq = self.inertia * 380.0               # quaternion attitude stiffness
        self.kw = self.inertia * 110.0               # attitude damping
        self.max_tilt = np.deg2rad(28.0)
        self.max_acc = 7.5            # slew the velocity command; step changes tumble it
        self.lat_acc = 35.0           # lateral slew (reversals are the tumble risk)
        self.airmode = True           # torque-priority mixing; see _mix below
        self.b3_rate = np.deg2rad(10000.0) # attitude-target slewing destabilises it
        self.recovering = False
        self.b3_prev = None
        self.v_cmd = np.zeros(3)
        # thrust-mixing matrix: [Fz, Mx, My, Mz] = MIX @ f
        self.mix_inv = np.linalg.inv(np.array([
            [1.0,    1.0,    1.0,    1.0],
            [-0.18,  0.18,   0.18,  -0.18],
            [0.14,   0.14,  -0.14,  -0.14],
            [-0.0201, 0.0201, -0.0201, 0.0201],
        ]))
        self.f_max = float(model.actuator_ctrlrange[0, 1])

    def __call__(self, data, v_des, yaw_des, dt=0.002):
        R = data.xmat[self.m.body("x2").id].reshape(3, 3)
        v = data.qvel[0:3]
        w = data.qvel[3:6]                            # body frame

        # A step change in the velocity command demands a step change in attitude,
        # which saturates the motors and tumbles the aircraft. Ramp it instead.
        step = self.max_acc * dt
        lim = np.array([step, self.lat_acc * dt, step])   # lateral slews slower
        self.v_cmd += np.clip(np.asarray(v_des) - self.v_cmd, -lim, lim)
        a = self.kv * (self.v_cmd - v)
        a_h = a[:2]
        lim = np.tan(self.max_tilt) * G
        if np.linalg.norm(a_h) > lim:                 # keep the tilt command sane
            a_h = a_h / np.linalg.norm(a_h) * lim
        # Attitude recovery. Rapid lateral reversals can drive the aircraft past 90
        # deg, where thrust = m*(a_des . b3) goes negative, clips to zero and the
        # thing free-falls inverted with no way back. Past ~55 deg, abandon the
        # translation command and prioritise getting the thrust axis upright again.
        tilt = float(np.arccos(np.clip(R[2, 2], -1.0, 1.0)))
        # Latched recovery: past 70 deg abandon the translation command entirely and
        # fly wings-level until back under 30 deg. Blending proportionally is not
        # enough -- the aircraft keeps being asked to accelerate sideways while it is
        # trying to right itself, and it carries on over the top.
        give_up = float(np.clip((tilt - np.deg2rad(55.0)) / np.deg2rad(25.0), 0.0, 1.0))
        a_h = a_h * (1.0 - give_up)

        # A quadrotor cannot thrust downward. On a hard commanded descent a[2]+G
        # goes NEGATIVE, so b3 = a_des/|a_des| points at the floor and the controller
        # dutifully commands an inverted attitude. The aircraft flips, drops into
        # attitude recovery (which zeroes horizontal accel) and then drifts sideways
        # with no lateral control at all -- while the navigator is still calling for
        # a dodge. Descent is limited by gravity and drag, not by pointing down.
        az = max(a[2] + G, 0.35 * G)
        a_des = np.array([a_h[0], a_h[1], az])

        b3 = a_des / np.linalg.norm(a_des)
        # Slew-limit the DESIRED thrust axis. The velocity loop can swing R_des by
        # ~100 deg between one lateral command and its reverse; the attitude loop
        # then chases a target moving faster than the airframe can follow, overshoots
        # past 90 deg, and the geometric controller has no recovery beyond that.
        if self.b3_prev is not None:
            dot = float(np.clip(self.b3_prev @ b3, -1.0, 1.0))
            ang = np.arccos(dot)
            max_ang = self.b3_rate * dt
            if ang > max_ang:
                axis = np.cross(self.b3_prev, b3)
                n = np.linalg.norm(axis)
                if n > 1e-9:
                    axis /= n
                    c, s = np.cos(max_ang), np.sin(max_ang)
                    b3 = (self.b3_prev * c + np.cross(axis, self.b3_prev) * s
                          + axis * (axis @ self.b3_prev) * (1 - c))
                    b3 /= np.linalg.norm(b3)
        self.b3_prev = b3.copy()
        b1c = np.array([np.cos(yaw_des), np.sin(yaw_des), 0.0])
        b2 = np.cross(b3, b1c)
        n2 = np.linalg.norm(b2)
        b2 = b2 / n2 if n2 > 1e-6 else np.array([0.0, 1.0, 0.0])
        Rd = np.column_stack([np.cross(b2, b3), b2, b3])

        # Quaternion attitude error instead of the geometric one.
        #   eR = 0.5*vee(Rd^T R - R^T Rd) has magnitude ~sin(theta): it PEAKS at 90 deg
        #   and falls to ZERO at 180 deg, so an inverted aircraft sits at a stationary
        #   point with no restoring torque. That is the 179 deg lawn-dart.
        # The quaternion vector part has magnitude sin(theta/2): monotonic all the way
        # to 180 deg, so recovery torque is maximal exactly when it is needed most.
        Re = np.ascontiguousarray(R.T @ Rd)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, Re.reshape(9))
        if q[0] < 0.0:
            q = -q                      # shortest way round
        tau = self.kq * q[1:4] - self.kw * w
        thrust = self.mass * float(a_des @ R[:, 2])
        if give_up > 0.0:             # keep authority in reserve to right itself
            thrust = max(thrust, 0.7 * self.mass * G)

        # Thrust-priority mixing: if the torque demand would drive a motor negative,
        # give up torque authority rather than silently losing lift.
        thrust = float(np.clip(thrust, 0.0, 4 * self.f_max))
        f_lift = np.full(4, thrust / 4.0)
        f_tau = self.mix_inv @ np.array([0.0, tau[0], tau[1], tau[2]])
        f = f_lift + f_tau
        if self.airmode:
            # Torque priority ("airmode"): shift the collective so the torque the
            # attitude loop asked for survives. Starving torque to protect thrust is
            # what lets a hard reversal tumble the aircraft past recovery.
            lo, hi = f.min(), f.max()
            if lo < 0.0:
                f = f - lo
            hi = f.max()
            if hi > self.f_max:
                f = f - (hi - self.f_max)
            return np.clip(f, 0.0, self.f_max)
        lo, hi = f.min(), f.max()
        scale = 1.0
        if lo < 0.0:
            scale = min(scale, thrust / 4.0 / max(1e-6, -(f_tau[np.argmin(f)])))
        if hi > self.f_max:
            scale = min(scale, (self.f_max - thrust / 4.0) / max(1e-6, f_tau[np.argmax(f)]))
        return np.clip(f_lift + np.clip(scale, 0.0, 1.0) * f_tau, 0.0, self.f_max)


# --------------------------------------------------------------------------- #
# perception
# --------------------------------------------------------------------------- #
class Eye:
    """Onboard forward camera -> symbolic scene summary.

    Uses the depth and segmentation buffers only; no ground-truth object poses
    leak into the summary, so the drone sees what a real camera would see.
    """

    W, H = 96, 72            # vertical resolution decides whether a thin sliver of
                             # clearance above an obstacle is seen at all
    N_SECTORS = 5
    SECTOR_NAMES = ["far_left", "left", "center", "right", "far_right"]
    SECTOR_NAMES_9 = ["hard_left", "far_left", "left", "inner_left", "center",
                      "inner_right", "right", "far_right", "hard_right"]
    MAX_RANGE = 45.0          # sight distance IS reaction time: at 15 m/s a 25 m
    TARGET_MIN_PIXELS = 3
    TARGET_RANGE_OFFSET_M = 0.0
                              # horizon is only 1.7s, and a 2 m dodge costs 0.9s
    TILT_DEG = -12.0
    IGNORE_BELOW_M = 0.5      # something we are already flying over is not a threat
    EYE_FOVY = 110.0          # wide FPV lens, independent of the cinematic chase camera
    THREAT_FOV_DEG = 40.0
    AHEAD_FOV_DEG = 12.0      # 'straight ahead' really must mean straight ahead:
                              # on a 125deg lens, the middle third of the image is
                              # +/-31deg and just measures the corridor wall     # a wide lens helps us SEE the target, but only what lies
                              # within this forward cone is something we could fly into

    def __init__(self, model, target_body="rover", ignore_below=IGNORE_BELOW_DEFAULT,
                 floor_geom="floor", shell=(), threat_fov=None, n_sectors=None,
                 walls=(), ignore_geoms=()):
        self.ignore_below = ignore_below
        # The tunnel shell is the environment, not an obstacle. Reported separately
        # as clearances, or it swamps every sector and the scene says nothing.
        # Ceiling and floor are not lateral obstacles: a ceiling 3.6m up shows as
        # ~3.9m of "range" in EVERY sector and drowns out what is actually ahead.
        # Altitude limits keep us off them; the walls stay as real obstacles.
        self.shell_ids = np.array([model.geom(n).id for n in shell], dtype=int)
        self.wall_ids = [model.geom(n).id for n in walls]   # (left, right)
        self.m = model
        self.depth = mujoco.Renderer(model, self.H, self.W)
        self.depth.enable_depth_rendering()
        self.seg = mujoco.Renderer(model, self.H, self.W)
        self.seg.enable_segmentation_rendering()
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.distance = 1.0
        self.cam.elevation = self.TILT_DEG
        # the whole rover body is the target, mast included -- not one geom of it
        self.target_ids = np.nonzero(model.geom_bodyid == model.body(target_body).id)[0]
        self.floor_id = model.geom(floor_geom).id
        self.ignore_ids = np.array([model.geom(name).id for name in ignore_geoms], dtype=int)
        # the camera sits inside the airframe: never mistake ourselves for an obstacle
        self.self_geoms = np.nonzero(model.geom_bodyid == model.body("x2").id)[0]
        fovy = np.deg2rad(self.EYE_FOVY)
        self.tan_h = np.tan(fovy / 2) * self.W / self.H   # horizontal half-fov
        self.hfov_deg = np.rad2deg(np.arctan(self.tan_h)) * 2
        # columns spanning the threat cone; the rest of the frame is for tracking only
        self.threat_fov = threat_fov or self.THREAT_FOV_DEG
        if n_sectors == 9:
            # A sector reports its MINIMUM, so a wide sector that is half open still
            # reads blocked. Narrow sectors let a real gap show up as a whole sector.
            self.N_SECTORS, self.SECTOR_NAMES = 9, self.SECTOR_NAMES_9
        ndc = np.tan(np.deg2rad(self.threat_fov)) / self.tan_h
        self.t0 = int(round((1 - min(ndc, 1.0)) / 2 * (self.W - 1)))
        self.t1 = int(round((1 + min(ndc, 1.0)) / 2 * (self.W - 1))) + 1
        nda = np.tan(np.deg2rad(self.AHEAD_FOV_DEG)) / self.tan_h
        self.a0 = int(round((1 - nda) / 2 * (self.W - 1)))
        self.a1 = int(round((1 + nda) / 2 * (self.W - 1))) + 1
        self._last_seen = None
        self._smooth = {}
        self._target_history = []
        self._last_target_fix = None
        self._last_target_yaw = None

    NOSE_OFFSET_M = 0.28      # camera on the nose, not buried in the middle of the
                              # airframe, so the aircraft is not in its own view

    def _aim(self, pos, yaw):
        self.m.vis.global_.fovy = self.EYE_FOVY    # free cameras share one fovy; claim it
        pos = np.asarray(pos, dtype=float) + self.NOSE_OFFSET_M * np.array(
            [np.cos(yaw), np.sin(yaw), 0.0])
        az, el = yaw, np.deg2rad(self.TILT_DEG)
        fwd = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        self.cam.azimuth = np.rad2deg(yaw)
        self.cam.lookat[:] = pos + self.cam.distance * fwd

    def look(self, data, pos, yaw, t):
        self._aim(pos, yaw)
        self.depth.update_scene(data, self.cam)
        self.last_depth_raw = self.depth.render()
        z = np.clip(self.last_depth_raw, 0.0, self.MAX_RANGE)
        self.seg.update_scene(data, self.cam)
        seg = self.seg.render()[:, :, 0]
        # The HUD and rain detector can reuse the actual latest camera buffers.
        # Rerendering them between perception ticks only consumes frame time.
        self.last_depth = z
        self.last_segmentation = seg

        # --- free space: ignore the floor, keep only real vertical obstructions
        obstacle = (seg != self.floor_id) & (seg >= 0)
        obstacle &= ~np.isin(seg, self.target_ids)
        obstacle &= ~np.isin(seg, self.self_geoms)
        if self.ignore_ids.size:
            obstacle &= ~np.isin(seg, self.ignore_ids)
        # Ceiling and floor are excluded from the LATERAL sectors (they would read as
        # a few metres in every direction) but they are exactly what decides whether
        # climbing or diving is possible, so the vertical bands keep them.
        solid = obstacle.copy()
        if self.shell_ids.size:
            obstacle &= ~np.isin(seg, self.shell_ids)
        # Height of every obstacle pixel relative to the aircraft. Anything well below
        # us is something we have already climbed over, not something we can hit.
        dz = self._height_offset(z, np.arange(self.H))
        if self.ignore_below is not None:
            obstacle &= dz > -self.ignore_below
        # MuJoCo depth is distance along the camera axis. Divide by cos(bearing)
        # to get true range, or everything off-axis reads deceptively close.
        sec_scale = 1.0 / np.cos(np.deg2rad(self._bearing_arr(np.arange(self.W))))
        rng_full = np.where(obstacle, z * sec_scale[None, :], self.MAX_RANGE)
        rng_full = np.minimum(rng_full, self.MAX_RANGE)
        rng_solid = np.minimum(np.where(solid, z * sec_scale[None, :], self.MAX_RANGE),
                               self.MAX_RANGE)
        rng = rng_full[:, self.t0:self.t1]
        cols = np.array_split(np.arange(rng.shape[1]), self.N_SECTORS)
        sectors = {n: self._filter("sec_" + n, float(rng[:, c].min()))
                   for n, c in zip(self.SECTOR_NAMES, cols)}

        # --- vertical extent of whatever is blocking us.
        # Without this the drone cannot tell a low beam it could hop over from a
        # tall pillar it must go around: both look identical in a horizontal scan.
        # Vertical free space, the same idea as the lateral sectors but stacked.
        # Robust: bucket obstacle pixels by their height relative to us and take
        # the nearest one in each band. No edge-finding, no trig on noisy rows.
        ahead = rng_full[:, self.a0:self.a1]
        dzc = dz[:, self.a0:self.a1]
        # Windows, not half-spaces: "above" must mean the altitude we would actually
        # climb to, otherwise it just measures the ceiling directly overhead.
        LVL, LO, HI = 0.75, 0.8, 2.2
        def _band(mask):
            v = ahead[mask]
            return round(float(v.min()), 2) if v.size else self.MAX_RANGE
        free_high = self._filter("hi", _band((dzc >= LO) & (dzc <= HI)))
        free_level = self._filter("lv", _band(np.abs(dzc) <= LVL))
        free_low = self._filter("lo", _band((dzc <= -LO) & (dzc >= -HI)))

        # How much room the tunnel itself leaves. Measured, not assumed.
        # (computed before the bands are finalised so it can veto them)
        room_up = room_dn = None
        if self.shell_ids.size:
            sh = np.isin(seg, self.shell_ids) & (z < self.MAX_RANGE * 0.8)
            if sh.any():
                hv = dz[sh]
                up, dn = hv[hv > 0.3], hv[hv < -0.3]
                room_up = round(float(np.percentile(up, 40)), 2) if up.size else None
                room_dn = round(float(-np.percentile(dn, 60)), 2) if dn.size else None
        # A band that sits outside the tunnel contains no obstacle pixels and so
        # reads "clear". Climbing into the ceiling is not clear: veto it.
        room_l = room_r = None
        if len(self.wall_ids) == 2:
            mid = slice(int(self.H * 0.35), int(self.H * 0.65))
            bear = np.deg2rad(self._bearing_arr(np.arange(self.W)))[None, :]
            for k, gid in enumerate(self.wall_ids):
                mk = (seg[mid] == gid) & (z[mid] < self.MAX_RANGE * 0.8)
                if not mk.any():
                    continue
                off = np.abs(z[mid][mk] * np.tan(np.broadcast_to(bear, seg.shape)[mid][mk]))
                val = round(float(np.percentile(off, 35)), 2)
                if k == 0:
                    room_l = val
                else:
                    room_r = val

        if room_up is not None and room_up < LO + 0.25:
            free_high = 0.0
        if room_dn is not None and room_dn < LO + 0.25:
            free_low = 0.0

        idx = self.t0 + int(np.argmin(rng.min(axis=0)))
        nearest = float(rng.min())
        nearest_brg = self._bearing(idx)

        # --- target: purely from the segmentation mask
        mask = np.isin(seg, self.target_ids)
        px = int(mask.sum())
        if px >= self.TARGET_MIN_PIXELS:
            self._last_seen = t
            ys, xs = np.nonzero(mask)
            brg = self._bearing(float(xs.mean()))
            rangem = round(float(np.median(z[mask])) + self.TARGET_RANGE_OFFSET_M, 2)
            elevation = round(float(np.rad2deg(self._elevation(float(ys.mean())))), 2)
            relative_height = round(float(self._height_offset(
                np.array([[rangem]]), np.array([float(ys.mean())]))[0, 0]), 2)
            self._target_history.append((t, rangem, brg, elevation))
            self._target_history = [row for row in self._target_history if t - row[0] <= 1.2]
            reference = self._target_history[0]
            span = t - reference[0]
            range_rate = round((rangem - reference[1]) / span, 2) if span >= 0.35 else None
            bearing_rate = round((brg - reference[2]) / span, 2) if span >= 0.35 else None
            elevation_rate = round((elevation - reference[3]) / span, 2) if span >= 0.35 else None
            target = {"visible": True, "bearing_deg": brg, "range_m": rangem,
                      "elevation_deg": elevation, "relative_height_m": relative_height,
                      "range_rate_mps": range_rate, "bearing_rate_deg_s": bearing_rate,
                      "elevation_rate_deg_s": elevation_rate,
                      "pixels": px, "unseen_for_s": 0.0}
            self._last_target_fix = dict(target)
            self._last_target_yaw = yaw
        else:
            gap = None if self._last_seen is None else round(t - self._last_seen, 2)
            last_fix = dict(self._last_target_fix) if self._last_target_fix else None
            if last_fix is not None and self._last_target_yaw is not None:
                change = (yaw - self._last_target_yaw + np.pi) % (2*np.pi) - np.pi
                last_fix["estimated_current_bearing_deg"] = round(
                    (last_fix["bearing_deg"] - np.rad2deg(change) + 180) % 360 - 180, 1)
            target = {"visible": False, "bearing_deg": None, "range_m": None,
                      "elevation_deg": None, "relative_height_m": None,
                      "range_rate_mps": None, "bearing_rate_deg_s": None,
                      "elevation_rate_deg_s": None, "pixels": 0,
                      "unseen_for_s": gap,
                      "last_seen": last_fix}

        blocked = sum(1 for v in sectors.values() if v < 3.0)
        # What lies in the PATH, as opposed to merely beside us. Braking for walls
        # you are flying between makes narrow gaps impossible to thread.
        path_ahead = round(min(sectors["left"], sectors["center"], sectors["right"]), 2)
        out = {"sector_range_m": sectors,
                "sectors_blocked": blocked,
                "path_ahead_m": path_ahead,
                "free_ahead_above_m": free_high,
                "free_ahead_level_m": free_level,
                "free_ahead_below_m": free_low,
                "room_above_m": room_up,
                "room_below_m": room_dn,
                "room_left_m": room_l,
                "room_right_m": room_r,
                "nearest_obstacle_m": round(nearest, 2),
                "nearest_bearing_deg": nearest_brg,
                "target": target}
        return out

    def _bearing_arr(self, cols):
        ndc = 2.0 * np.asarray(cols, dtype=float) / (self.W - 1) - 1.0
        return np.rad2deg(-np.arctan(ndc * self.tan_h))

    def _bearing(self, col):
        """Signed bearing in degrees, +ve = to the aircraft's left (+y body).

        Image-right is -y for a camera looking down +x with +z up, hence the negation.
        """
        ndc = 2.0 * col / (self.W - 1) - 1.0
        return round(float(np.rad2deg(-np.arctan(ndc * self.tan_h))), 1)

    def _filter(self, key, raw):
        """Median of the last three frames. Single-pixel depth dropouts otherwise
        make free space flicker 3x between frames and no decision layer can commit
        to a signal like that. A median kills the spikes without holding on to a
        stale minimum, which an attack/release filter does (and which leaves the
        aircraft convinced it is walled in long after it is clear)."""
        buf = self._smooth.setdefault(key, [])
        buf.append(float(raw))
        if len(buf) > 3:
            buf.pop(0)
        return round(float(sorted(buf)[len(buf) // 2]), 2)

    def _height_offset(self, z, rows):
        """World height of each pixel relative to the aircraft.

        depth*tan(elevation) is only right for a level camera. With the camera
        pitched down by TILT_DEG the depth is measured along the tilted axis, so
        the correct relation is depth * sin(elev) / cos(elev - tilt).
        """
        elev = self._elevation(rows)[:, None]
        phi = elev - np.deg2rad(self.TILT_DEG)
        return z * np.sin(elev) / np.maximum(np.cos(phi), 1e-3)

    def _elevation(self, rows):
        """Elevation of an image row in radians, +ve above horizontal."""
        half = (self.H - 1) / 2.0
        rel = (half - np.asarray(rows, dtype=float)) / half * np.deg2rad(self.EYE_FOVY) / 2
        return rel + np.deg2rad(self.TILT_DEG)

    def sector_bearing(self, name):
        i = self.SECTOR_NAMES.index(name)
        centre = self.t0 + (i + 0.5) * (self.t1 - self.t0) / self.N_SECTORS
        return np.deg2rad(self._bearing(centre))
