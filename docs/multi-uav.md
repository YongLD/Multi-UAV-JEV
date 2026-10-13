# Cooperative fleet pursuit

This release supports **1, 2 or 4 independent physical pursuers** against one kinematic evader. Start with `bash run.sh --drones 4`, or select a count in the browser. Count changes restart the episode; focus/view changes do not. Every pursuer has separate free-joint state, four actuator addresses, perception, controller and model queue.

## Shared observations

An aircraft converts camera-estimated target geometry into a common frame using **its own odometry**. Messages carry position, estimated velocity, capture time, confidence, uncertainty and source ID. Fusion weights uncertainty and rejects conflicting measurements. Receiving a teammate message never sets local camera visibility to true.

Short extrapolation uses observed motion; it never reads the evader route. Confidence decays without new measurements, and tracks expire after at most **3 simulated seconds**. The UI's observation age is time since the last valid measurement; prediction does not reset it. Own/peer odometry assumes a common ideal reference frame.

## Complementary missions

| Role | Planner | Execution |
| --- | --- | --- |
| P-01: track | Current fused observation / short forecast | Maintain contact and close on the target. Other pursuers temporarily relay this task if P-01 is unhealthy. |
| P-02: forward block | Constant-velocity projection and arrival-time comparison | Commit to a point ahead, travel there, then hold and face the target. Replan after a substantial course change or target passage. |
| P-03: intercept | Velocity and turn-rate fit from history | Choose an earliest reachable future meeting point and update with new observations. Low confidence suspends long-range planning. |
| P-04: support/rescue | Peer state, shared observations, local depth | Maintain a support position; approach an unhealthy teammate at safe spacing and authorize recovery. Own recovery takes priority. |

The model and UI expose mission stage, goal, prediction horizon, arrival-time estimates and reachability. The overview marks blockade/interception goals as B2/I3. **Late arrival is not reported as successful interception.** Initial deployment coordinates are fixed role configuration, not evader truth.

## Editable rules

[sim/role.md](../sim/role.md) contains four role instructions and a validated JSON configuration block. Edit it to change candidate conditions, deployment positions, blockage/interception parameters, teammate rescue or support self-rescue.

The running episode checks for edits approximately once per wall-clock second. Valid changes replace the active rules; invalid JSON, missing fields or unsafe bounds keep the previous rules and report an error. Deployment coordinates take effect at the next episode. Keep copies of the resolved `policy.json` and role version with experimental results.

## Scheduling and final safety

Physics steps at **0.002 s**; navigation and final safety update every **0.02 s**. Cameras observe at **0.15 s** intervals. Requests share a serial lock for one endpoint; each worker refreshes its latest waiting observation when its turn arrives. Request duration, queue wait and total latency are logged independently. Results older than **0.8 simulated seconds** are not applied.

Jev chooses a tactical intent, while mission navigation manages continuous velocity/heading. Peer-separation correction is applied **before** the final velocity guard. That guard checks fresh depth, observed pose/motion, braking clearance, unknown coverage and approaching peers; later code does not add unchecked velocity. A blocked route can select an already observed local detour. These are explicit deterministic controllers; their continued operation during endpoint failure is not a model fallback.

## Teammate rescue and P-04 self-rescue

P-04 has `rescue` and `self_rescue` alongside five flight actions, and has **no `fire`** candidate. Rescue approaches a troubled peer at safe spacing and asks its own motor controller to restore attitude and escape toward camera-observed clearance. Default automatic authorization avoids waiting on a slow model; the assignment, command delivery, recovery and timeout remain separate events. No tow force, teleport or pose reset is modeled.

For P-04, stuck/unstable health or sustained safety braking without progress can automatically start self-rescue. A valid explicit `self_rescue` choice can also initiate it. Default sequence and bounds:

1. Prioritize motor-based attitude recovery when tilted.
2. Scan in place if no fresh known-safe direction is available.
3. Escape at up to **1.2 m/s**, still subject to depth and peer guards.
4. Recover only after **≥2 m actual displacement** and healthy attitude below **25°** for **1.5 s**.
5. Time out an attempt at **18 s**; allow at most **three** attempts with a **2 s** retry delay. Exhaustion displays failure rather than a false recovery.

All values are configurable within validated bounds in `role.md`. Own recovery pauses teammate rescue until the support aircraft becomes healthy.

## Live view and replay

Each live follow image uses the same full single-aircraft HUD as recordings: own RGB/depth, flight instruments, probabilities/counts, metrics, prediction/shot effects and overview. Click a card to enlarge or select the single-aircraft instrumentation mode. The overview/referee imagery is for inspection and is excluded from model input.

Recordings capture all cameras every **0.05 simulated seconds** (20 FPS), independently of viewer focus. Per-camera output is 1280×720; two/four-camera mosaics are 2560×720 / 2560×1440. RGB/depth reflect the actual perception cycle, and the overview refreshes at 0.3 s; displayed ages identify these slower sources. Full-HUD composition and encoding run on a bounded background queue, with backpressure instead of dropping frames.

Live JPEG publication is every **0.2 simulated seconds**, not 20 wall-clock FPS. More cameras and encoding can reduce wall-clock simulation speed. `frame_times.json` records real simulation capture timestamps; smooth playback alone does not establish real-time execution.

Each 1/2/4-aircraft group retains its top **two** completed recordings with ≥5 hits, with newer ties preferred. Streams are retained as one immutable set. Ranking applies equally to historical recordings; old follow-only clips cannot gain RGB/depth or decisions that were never recorded.

## Scope and evaluation

Communication is ideal zero-delay broadcast. P-04's support role does not simulate an RF relay, packet loss or radio range. Target segmentation and odometry are idealized; the evader is kinematic. Contacts, missed tracks and failed recovery remain possible. Compare matched seeds and report contacts, visibility, latency/errors, expired decisions, rescue outcomes and wall-clock throughput rather than infer effectiveness from one replay.
