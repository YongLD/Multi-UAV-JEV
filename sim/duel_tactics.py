"""Text decisions from camera-derived state for the city pursuit."""
from __future__ import annotations

import json
import os
import queue
import threading
import time
from pathlib import Path
from urllib.request import Request, urlopen

ACTIONS = ("turn_left", "turn_right", "climb", "descend", "forward", "fire")
TURN_DEG = 45.0
MODEL_NAME = os.environ.get('JEV_MODEL', 'NeoHorse-Jev-4B')
POLICY_MODE = os.environ.get('DUEL_POLICY_MODE', 'conditions')
if POLICY_MODE not in ('free', 'effects', 'conditions', 'legacy'):
    raise ValueError('DUEL_POLICY_MODE must be free, effects, conditions or legacy')
POLICY_VERSION = f'camera6_turn45_v13_neohorse_text_{POLICY_MODE}'

LEGACY_QUESTION = json.loads(Path(__file__).with_name('policy_camera6_v8.json').read_text())
PROMPT = dict(LEGACY_QUESTION['action']['instructions'])
PROMPT['effects'] = PROMPT['effects'].replace('15deg', f'{TURN_DEG:g}deg')
PROMPT['rules'] = PROMPT['rules'].replace(
    'If track_status=lost, search LEFT/RIGHT only; do not descend, climb, fire or go forward for search.',
    'If track_status=lost, use last observed direction/history and observed open air to reacquire; avoid blind circling.')
# The two experiment arms differ only in these per-candidate descriptions.
# null renders as the action name alone in the deployed NeoHorse adapter.
CRITERIA = LEGACY_QUESTION['action']['criteria']
QUESTION = {'action': {'type': 'choice', 'instructions': PROMPT,
                       'criteria': CRITERIA if POLICY_MODE == 'conditions' else {a: None for a in ACTIONS}}}
if POLICY_MODE == 'effects':
    QUESTION['action']['criteria'] = {
        'turn_left': f'Turn the aircraft heading left by {TURN_DEG:g} degrees.',
        'turn_right': f'Turn the aircraft heading right by {TURN_DEG:g} degrees.',
        'climb': 'Increase the aircraft altitude by 2 meters.',
        'descend': 'Decrease the aircraft altitude by 2 meters.',
        'forward': 'Fly straight along the current heading at the current altitude.',
        'fire': 'Attempt one simulated shot at the enemy aircraft.',
    }
if POLICY_MODE == 'legacy':
    QUESTION = json.loads(Path(__file__).with_name('policy_camera6_v8.json').read_text())
    QUESTION['action']['instructions']['effects'] = QUESTION['action']['instructions']['effects'].replace('15deg', f'{TURN_DEG:g}deg')


def describe_relative_target(target):
    """Read camera geometry in words; no eligibility flags or action suggestion."""
    if not target.get('visible'):
        gap = target.get('unseen_for_s')
        return 'No current target observation.' + (f' Last seen {gap} seconds ago.' if gap is not None else '')
    bearing = float(target['bearing_deg'])
    height = float(target['relative_height_m'])
    side = 'left' if bearing > 0 else 'right' if bearing < 0 else 'ahead'
    vertical = 'higher' if height > 0 else 'lower' if height < 0 else 'at the same altitude'
    return (f"Visible target is {target['range_m']} meters away, {abs(bearing)} degrees {side}, "
            f"{abs(height)} meters {vertical} than our aircraft.")


def build_state(observed: dict, own: dict) -> dict:
    """Explicit allowlist: no MuJoCo world, enemy pose or building coordinates."""
    target = observed["target"]
    fields = ("visible", "bearing_deg", "elevation_deg", "relative_height_m",
              "range_m", "range_rate_mps", "bearing_rate_deg_s",
              "elevation_rate_deg_s", "unseen_for_s", "relative_position_m",
              "relative_velocity_mps", "track_confidence", "position_uncertainty_m", "aim",
              "camera", "front_visible", "search_memory")
    seen = {k: target.get(k) for k in fields}
    trajectory = target.get('trajectory')
    if trajectory and len(trajectory['history']) >= 3:
        seen['trajectory'] = trajectory
    predictions = (trajectory or {}).get('predictions') or []
    pursuit = target.get('lead') or target.get('aim')
    seen['pursuit_bearing_deg'] = pursuit.get('bearing_deg') if pursuit else None
    seen['pursuit_height_m'] = pursuit.get('height_m') if pursuit else None
    seen['pursuit_horizon_s'] = pursuit.get('horizontal_horizon_s', pursuit.get('horizon_s')) if pursuit else None
    seen['pursuit_uncertainty_m'] = pursuit.get('uncertainty_m') if pursuit else None
    predicted_height = (predictions[0]['relative_xyz_m'][2]-own['velocity_body_mps'][2]*(seen.get('aim') or {}).get('horizon_s',0)) if predictions and predictions[0]['confidence'] >= .2 else (seen.get('aim') or {}).get('height_m')
    seen['predicted_height_1s_m'] = round(predicted_height,2) if predicted_height is not None else None
    seen['track_status'] = ('visible' if target['visible'] else
                            'brief_occlusion' if target.get('aim') and target.get('track_confidence', 0) > .3 else 'lost')
    if not target["visible"] and target.get("last_seen"):
        seen["last_seen"] = {k: target["last_seen"].get(k) for k in
                             ("bearing_deg", "range_m", "bearing_rate_deg_s")}
        seen["estimated_current_bearing_deg"] = target.get("estimated_current_bearing_deg")
        seen["estimated_relative_position_m"] = target.get("estimated_relative_position_m")
    front_clearance = observed['observed_clearance']['level']
    speed = max(float(own['horizontal_speed_mps']), 0.1)
    nav = observed['navigation']
    navigation = {
        'horizon_s': nav['horizon_s'],
        'grid_bearings_deg': nav['grid_bearings_deg'],
        'grid_clearance_m': {label: [cell['clearance_m'] for cell in cells] for label, cells in nav['grid'].items()},
        'grid_limit_m': 35,
        'velocity_bearing_deg': nav['velocity_bearing_deg'],
        'velocity_path_clearance_m': nav['velocity_path']['clearance_m'],
        'velocity_path_unknown': nav['velocity_path']['status'] == 'unknown',
        'braking_distance_m': nav['braking_distance_m'],
        'reaction_budget_s': nav['reaction_budget_s'],
        'nearest_obstacle_m': nav['nearest_obstacle_m'],
        'nearest_obstacle_bearing_deg': nav['nearest_obstacle_bearing_deg'],
        'predicted_contact_if_velocity_continues_s': nav['predicted_contact_if_velocity_continues_s'],
        'imminent_contact': (nav['predicted_contact_if_velocity_continues_s'] is not None
                             and nav['predicted_contact_if_velocity_continues_s'] < 1.5),
        'actions': {a: {k: values[k] for k in ('predicted_contact_s', 'min_clearance_m', 'observed_path_fraction')}
                    for a, values in nav['actions'].items()},
    }
    return {
        "relative_target_description": describe_relative_target(target),
        "target_from_camera": seen,
        "own": {k: own[k] for k in ("speed_mps", "altitude_m", "roll_deg", "pitch_deg", "yaw_deg",
                                      "yaw_rate_deg_s", "heading_setpoint_deg", "heading_error_deg",
                                      "speed_setpoint_mps", "last_action",
                                      "shot_cooldown_remaining_s", "altitude_setpoint_m",
                                      "velocity_body_mps", "horizontal_speed_mps",
                                      "last_decision_latency_s", "altitude_error_m",
                                      "turn_in_progress", "vertical_in_progress")},
        "free_space_from_camera_m": {
            "horizontal": observed["sector_range_m"],
            "ahead_above": observed["observed_clearance"]["above"],
            "ahead_level": observed["observed_clearance"]["level"],
            "ahead_below": observed["observed_clearance"]["below"],
        },
        "front_obstacle_eta_s": round(front_clearance/speed, 2) if front_clearance is not None else None,
        "estimated_turn_radius_m": round(speed*speed/5.2, 1),
        "stuck_against_obstacle": front_clearance is not None and front_clearance < 3 and speed < 1,
        "rain_visible_from_camera": bool(observed.get("rain_visible", False)),
        "peripheral_free_space_from_camera_m": observed.get('peripheral_space', {}),
        "navigation_from_camera": navigation,
    }


class JevWorker:
    def __init__(self, endpoint: str, hz: float = 3.0):
        self.endpoint = endpoint
        self.model = MODEL_NAME
        self.backend = 'neohorse'
        self.input_mode = 'text'
        self.interval = 1.0 / hz
        self._in = queue.Queue(maxsize=1)
        self._out = queue.Queue()
        self._stop = threading.Event()
        self._sent_wall = 0.0
        self.calls = self.errors = 0
        self.abstentions = 0
        self.last_error = None
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def offer(self, state: dict, sim_time: float) -> None:
        now = time.monotonic()
        if now - self._sent_wall < self.interval:
            return
        try:
            state = state() if callable(state) else state
            if self._in.full():
                # Refresh only the waiting observation, never a model result.
                try:
                    self._in.get_nowait()
                except queue.Empty:
                    pass
            self._in.put_nowait((state, sim_time))
            self._sent_wall = now
        except queue.Full:
            pass

    def drain(self) -> list[dict]:
        result = []
        while True:
            try:
                result.append(self._out.get_nowait())
            except queue.Empty:
                return result

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                state, sim_time = self._in.get(timeout=.2)
            except queue.Empty:
                continue
            started = time.monotonic()
            try:
                value = {"model": self.model, "state": state, "questions": QUESTION}
                body = json.dumps(value).encode()
                headers = {"Content-Type": "application/json"}
                if os.environ.get('JEV_API_KEY'):
                    headers['Authorization'] = 'Bearer ' + os.environ['JEV_API_KEY']
                request = Request(self.endpoint, body, headers)
                with urlopen(request, timeout=12) as response:
                    payload = json.load(response)
                answer = payload["answers"]["action"]
                action = answer["choice"]
                if action is not None and action not in ACTIONS:
                    raise ValueError(f"invalid action: {action!r}")
                self.calls += 1
                record = {"policy_version": POLICY_VERSION, "model_name": self.model,
                          "input_mode": self.input_mode, "usage": payload.get('usage'),
                          "probabilities": answer.get("probabilities", {}),
                          "native_abstain_probability": answer.get('abstain'),
                          "native_valid": answer.get('valid'),
                               "confidence": answer.get("confidence"), "state": state,
                               "sim_time": sim_time, "latency_s": round(time.monotonic()-started, 3)}
                if action is None:
                    self.abstentions += 1
                    record['abstained'] = True
                else:
                    record['action'] = action
                self._out.put(record)
            except Exception as exc:
                self.errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"[:180]
                self._out.put({"error": self.last_error, "state": state,
                               "sim_time": sim_time, "latency_s": round(time.monotonic()-started, 3)})

    def close(self) -> None:
        self._stop.set()
        # HTTP timeout is 12 s. Allow it to finish so the episode can log the
        # final response instead of losing it at process exit.
        self.thread.join(timeout=14)
