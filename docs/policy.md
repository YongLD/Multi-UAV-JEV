# Prompt and action semantics

The policy is constructed by [sim/duel_tactics.py](../sim/duel_tactics.py) from the historical [policy_camera6_v8.json](../sim/policy_camera6_v8.json) template. The exported current policy replaces 15° turns with 45° and revises lost-target search wording. **The historical JSON alone is not the complete current prompt.**

Print the exact current request question, without loading a model:

```bash
.venv/bin/python -c 'import sys,json; sys.path.insert(0,"sim"); from duel_tactics import QUESTION; print(json.dumps(QUESTION,ensure_ascii=False,indent=2))'
```

Each actual episode writes the resolved question, actions, turn angle and mode to `runs/live/policy.json` and archives it in its case folder. That is the authoritative prompt for an experiment.

## Policy modes

| Mode | Candidate descriptions |
| --- | --- |
| `conditions` | Default. Each action has task-specific selection conditions from the template. |
| `free` | Action names only (`null` descriptions). Mission, general rules and physical effects remain. |
| `effects` | Each candidate describes only its physical effect; general mission/rules remain. |
| `legacy` | Retains the historical stricter lost-target instruction, with current 45° turn semantics. |

`free` does not mean an empty prompt or removal of all guidance. For a fair comparison, use the same scene seeds and report hits, contacts, lost-track duration, model errors, latency and stale response counts—not just a single favorable replay.

## Mission and rules

The prompt asks the model to close on E-01 and obtain ten valid virtual hits. It explains sign conventions, height alignment, firing geometry and cooldown, observed depth risk, short occlusion prediction and future pursuit bearing. It tells the model that speed/braking are managed by the controller and rain slows flight.

No candidate probability is modified after the model returns. A valid fresh choice is applied as returned. A response can be recorded without execution if it is stale, malformed, failed or explicitly abstained.

## Exact physical effects

- `turn_left`: set heading to **current execution-time yaw +45°**.
- `turn_right`: set heading to **current execution-time yaw −45°**.
- `climb`: set altitude to **current execution-time altitude +2 m**, capped at 36 m.
- `descend`: set altitude to **current execution-time altitude −2 m**, floored at 3 m.
- `forward`: set heading and altitude to the current pose; cancel prior targets. Inertial rates settle through the motor controller.
- `fire`: increment the virtual firing attempt counter and evaluate the shot; preserve flight setpoints. A cooldown-ineligible attempt cannot be a valid hit.

Repeated turns/climbs are relative to the aircraft's actual pose at each execution, not an arbitrarily accumulated sequence of queued setpoints.

## Virtual hit referee

The referee uses simulation truth solely for evaluation:

- 3D target distance ≤25 m.
- Absolute horizontal target bearing relative to pursuer heading ≤40°.
- Absolute altitude difference ≤1 m.
- At least 2 simulated seconds since the last cooldown-eligible attempt.

There is no additional occlusion/line-of-sight test in the current hit function. The policy asks to fire at a visible target, but that instruction is not an extra referee gate. Ten valid hits end the episode. This is a virtual interaction; no physical weapon or real drone integration is included.
