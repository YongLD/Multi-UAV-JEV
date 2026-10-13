# Architecture and observation boundary

This release adds 2/4-aircraft cooperative pursuit to the single-aircraft baseline below. Each physical pursuer owns independent joints, motors, camera observations and a decision worker. [The fleet guide](multi-uav.md) describes observation fusion, role missions, 50 Hz final safety, rescue and synchronized recordings. Shared tracks remain camera-derived; they never turn teammate visibility into local visibility.

## Runtime layers

1. **Scene and evader.** `urban_scene.py` builds a waterfront district. `Evader` in `duel.py` generates random starting points, turns, speed changes and smooth altitude changes. It uses scene geometry to generate its own motion. It is kinematically moved, unlike the physically simulated pursuer.
2. **Onboard perception.** `DuelEye` renders depth and segmentation from cameras attached to the pursuer's heading frame. `CameraTargetTrack` reconstructs relative target geometry, compensates for the pursuer's own odometry and maintains observed history.
3. **Local free space.** `DepthSafety` describes depth coverage, obstacle clearance, braking distance, velocity-path risk and approximate short maneuver forecasts.
4. **Jev decision.** `build_state` explicitly selects observable fields. `JevWorker` sends one typed choice question and preserves the returned action and full distribution.
5. **Execution.** `ActionController` converts the choice into heading/altitude setpoints or a virtual shot. It separately governs translation speed. The inherited `Pilot` converts setpoints into quadrotor thrust through velocity and attitude control.
6. **Referee and viewer.** Simulation truth computes virtual hits and contacts. Third-person and top-down views may show truth. This information is not fed back as enemy/world coordinates to Jev.

## What the model sees

| Input group | Content and source |
| --- | --- |
| Relative target | Camera-estimated bearing, elevation, height difference, range and derivatives. Positive bearing means left; positive height means above. |
| Tracking memory | Visibility, elapsed occlusion, confidence, relative velocity, observed samples and 1/2/3 s extrapolations. Forecasts estimate motion from observed history, not from the evader controller's future route. |
| Own aircraft | Odometry-derived speed, attitude, heading/altitude targets and errors, motion in progress, last action, cooldown and recent inference latency. |
| Free space | Depth sectors and upper/level/lower clearances, local risk grid, coverage, estimated braking needs and maneuver forecasts. Unknown space remains unknown. |
| Rain | A camera-visible rain indicator. Only rain is modeled; there are no thunderstorm hazards. |

The frontend's overview and evader speed display are spectator telemetry. They are not evidence that these truth values enter model state.

## Camera scope

The primary camera is stabilized to the pursuer's heading, with 192×144 depth/segmentation samples and a 110° vertical field of view. RGB is rendered separately for the UI. Auxiliary left, right and rear target scans are used when the front camera cannot detect the target. Their observations are transformed back to the same forward/left/up frame; the visible RGB panel remains the front camera.

Target segmentation currently uses MuJoCo's ideal object identity buffer, including a synthetic target halo. This is a simulation perception proxy, not a learned detector operating on ordinary camera images. It avoids feeding target coordinates directly but remains easier than real-world visual detection. Own odometry is also idealized.

The local navigation grid covers seven horizontal bearings at three height layers. Approximate action sweeps estimate contact timing and observed coverage. There is no persistent occupancy map or complete 360° obstacle knowledge. MuJoCo collision meshes can use convex hull approximations that are more conservative than the visible concave building shape.

## Scheduling and control limits

Physics uses 0.002 s steps. Single-aircraft perception runs approximately every 0.066 simulated seconds; fleet perception runs every 0.15 s. Fleet navigation and final safety run every 0.02 s. Workers share a serialized endpoint lock and refresh the latest waiting observation when admitted. Actual decision frequency depends on inference, rendering and queue time.

A response older than 1.2 simulated seconds in the single baseline, or 0.8 s in the fleet, is logged but not executed. Errors and abstentions are recorded without fabricating a model choice. Single-aircraft setpoints continue; fleet role navigation, safety and automatic recovery continue independently of the model.

The model does not choose acceleration or braking actions in this release. The controller limits speed using depth clearance, reaction delay, a braking estimate, camera-estimated target range/range rate and rain reduction. It can bring translation speed down to zero near obstacles. Altitude setpoints for climb/descend are bounded to 3–36 m. These execution constraints are visible parts of the architecture, not evidence of unconstrained model flight.

## Code map

- [duel.py](../sim/duel.py): perception integration, evader, action execution, virtual hits and episode loop.
- [duel_tactics.py](../sim/duel_tactics.py): allowed state, policy construction and model client.
- [onboard_target.py](../sim/onboard_target.py): relative tracking and prediction.
- [onboard_safety.py](../sim/onboard_safety.py): observed depth and risk forecasts.
- [flight.py](../sim/flight.py): inherited velocity/attitude/motor control.
- [urban_scene.py](../sim/urban_scene.py): visual city and simulation geometry.
- [urban.html](../sim/urban.html): browser cockpit, restart, camera controls and replay.
- [scripts/run.py](../scripts/run.py): process ownership and completed episode collection.
