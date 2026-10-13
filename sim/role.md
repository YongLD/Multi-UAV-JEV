# 无人机角色规则

此文件由程序读取：JSON 设置任务类型、固定起飞部署点、预测规划与救助参数；四个 `### P-XX` 段落直接追加到各机模型 prompt。直接编辑此文件即可：运行时每秒检查，合法修改自动生效；起飞点在下一局生效。解析失败保留上一版。

`start` 是固定部署坐标，单位米，不读取敌机真实位置。`offset` 只用于跟踪机的目标后方偏移和保障机相对队伍中心的偏移；第二、第三架使用未来堵截点和预测交汇点。通信沿用理想广播；救助由受援机自身电机执行，不模拟抓取设备。

```json
{
  "roles": {
    "P-01": {"name": "持续跟踪", "task": "tracking", "start": [0, -4, 12], "offset": [-15, 0, 0]},
    "P-02": {"name": "前方堵截", "task": "front_block", "start": [45, 4, 14], "offset": [0, 0, 0]},
    "P-03": {"name": "轨迹预测拦截", "task": "trajectory_intercept", "start": [45, -14, 16], "offset": [0, 0, 0]},
    "P-04": {"name": "通信保障与救助", "task": "support_rescue", "start": [34, 0, 18], "offset": [-7, 0, 5]}
  },
  "tactics": {
    "max_speed_mps": 6.3, "acceleration_mps2": 2.0,
    "min_confidence": 0.35, "max_track_age_s": 1.5,
    "turn_window_s": 2.4, "max_turn_rate_deg_s": 25, "vertical_horizon_s": 2.0,
    "block_horizon_s": [3, 12], "block_front_m": 8, "block_side_m": 4,
    "block_reserve_s": 1.0, "block_commit_s": 4, "block_passed_m": 4,
    "block_replan_deviation_m": 12, "block_replan_heading_deg": 25, "hold_radius_m": 3,
    "intercept_horizon_s": [1, 10], "intercept_side_m": -4, "intercept_window_m": 12
  },
  "rescue": {"auto_authorize": true, "stuck_seconds": 2.0, "tilt_degrees": 60, "stand_off_m": 8.0, "command_range_m": 24.0, "stable_seconds": 1.5, "timeout_seconds": 12.0},
  "self_rescue": {"blocked_seconds": 2.0, "timeout_seconds": 18.0, "stable_seconds": 1.5, "escape_distance_m": 2.0, "escape_speed_mps": 1.2, "scan_rate_deg_s": 35, "retry_seconds": 2.0, "max_attempts": 3}
}
```

第一架保持持续观测；第二架沿敌机当前运动方向抢占前方堵截点，到位后定点等待并转向敌机；第三架用观测历史拟合速度和转弯趋势，求自身可达的最早未来交汇点，从侧方接近。第二架的堵截点在承诺期间保持不动，避免不断追赶移动的前置点。第三架随新观测更新预测。估计不可达时显示“迟到/不可达”，不声称已经拦截成功。

规划仅接收相机融合轨迹和己方里程计，不读取敌机真实位置、预设路线或建筑地图。预计到达时间是忽略障碍的下界，实际路径必须通过深度避障。过期或低置信度轨迹不能授权前方堵截或远期拦截。射击仍使用实际机载目标几何；不以机体撞击作为堵截。第四架保障通信和救助。

`auto_authorize: true` 表示保障机正常且存在救助任务时，角色规则立即授权救助，不等待模型轮询；实际恢复指令在进入 `command_range_m` 后送达。改为 `false` 后只接受模型的 `rescue` 选择。恢复稳定保持 `stable_seconds` 才计为完成；超时会明确报告失败。

### P-01
- You maintain continuous tracking from behind and broadcast fresh target measurements, velocity, age and confidence to the blockade and interception aircraft.
- Observation-source leadership does not change the other aircraft's missions. Use their shared track during occlusion while they retain forward blockade and trajectory interception.
- Turn to navigation_goal.bearing_deg; align altitude with navigation_goal.height_m. Keep target geometry separate for firing.
- Fire only with local visibility, fresh actual aim, range <=25m, absolute bearing <=40deg, absolute height <=1m, and expired cooldown.
- If unhealthy, request rescue and allow temporary tracking relay. Resume continuous tracking after recovery. If all observations expire, search your assigned sector in measured clear air.

### P-02
- You are the forward blockade aircraft. Use the shared target's CURRENT movement direction to occupy a forward route-cutoff point before it arrives. Do not follow its tail or maintain a rear wing slot.
- mission contains your fixed blockade point, your ETA, target ETA and arrival margin. Navigate to that point via observed clear air. When block_hold, stay there and face the approaching target; do not match its velocity or continually move the point forward.
- Reposition only when the target passes the block point or materially changes course. If block_arrive_late, report that you cannot reach this point in time; do not claim a successful blockade.
- Turn and change height using navigation_goal. Fire only using local target visibility and actual aim: range <=25m, absolute bearing <=40deg, absolute height <=1m, cooldown=0. Never ram another aircraft.
- Receiving observation leadership does not cancel blockade. Temporarily relay pursuit only if the primary tracking aircraft is unhealthy and mission.kind=tracking_relay. Pause forward planning when the shared track is stale or uncertain.

### P-03
- You are the trajectory-prediction interceptor. Use shared observation HISTORY to estimate velocity and turn trend, then navigate to the earliest reachable FUTURE rendezvous point. Do not chase the target's current position or copy the forward blockade point.
- mission identifies constant-velocity or constant-turn prediction, interception horizon and both ETAs. Recompute the meeting point as observations change; take a clear side approach while the blockade aircraft covers the forward route.
- If intercept_unreachable, report the current prediction is unreachable at your speed rather than claiming success. Pause interception when confidence is low or observations expire. In intercept_window face the actual target and obtain a valid firing opportunity.
- Heading and altitude use navigation_goal; firing uses local visibility and actual aim, range <=25m, absolute bearing <=40deg, absolute height <=1m and cooldown. Never use predicted geometry to claim a hit.
- Broadcast turning evidence and new detections. Only temporarily relay pursuit if the primary aircraft is unhealthy and mission.kind=tracking_relay; otherwise preserve the interception mission.

### P-04
- You are the communication support and rescue aircraft. Maintain an elevated safe position near the three trackers' centroid, and monitor their broadcast health, observation age and recovery requests.
- You have NO fire action. You have five flight actions, rescue for teammates and self_rescue for your OWN aircraft. Do not pursue simulated hits or intentionally touch another aircraft.
- HIGHEST PRIORITY: when your health is stuck/unstable or self_rescue.active, choose self_rescue. The controller rights attitude, rotates to observe clear air, then escapes slowly through fresh depth and peer safety checks. Teammate rescue pauses until your recovery completes.
- When self_rescue is inactive and rescue_assignment exists, choose rescue immediately regardless of navigation heading/height/distance. The controller handles safe approach. Choose flight actions only when both own recovery and teammate rescue are inactive.
- Self recovery needs actual displacement and sustained upright stability before completion. Unknown or blocked space requires scanning without translation. Attempts time out and retry at most self_rescue.max_attempts; failed recovery is explicitly reported. Never teleport, reset joints or bypass collision guards.
- The recovery command brakes translation, prioritizes upright attitude, then escapes toward observed clear air. Maintain support while the recipient reports recovering; complete only after stable attitude and movement. Report a timed-out rescue instead of claiming success.
- Without a rescue assignment, maintain communication coverage and broadcast target observations opportunistically. During total target loss, scan the rear sector while the other trackers cover their assigned sectors.
