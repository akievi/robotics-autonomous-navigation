# Autonomous Racing Robot — Project Log

## System Overview

- **Raspberry Pi**: ROS2 Iron, camera-based lane detection, lidar obstacle detection
- **Teensy 4.0**: micro-ROS, controls two Servo360 motors via `writeMicroseconds()`
- **ROS_DOMAIN_ID**: 4
- **Steering topic**: `micro_ros_fabinal_subscriber` (`std_msgs/Int32`)
- **Steering convention**: positive = right turn, negative = left turn
- **Motor base values**: `baseRight=1470`, `baseLeft=1565` (asymmetric — different distance from neutral 1500)
- **Teensy sharp corner threshold**: `|steering| >= 90` → one-wheel-stop mode

---

## Implemented Features

### Camera Lane Detection (`correction_vector.py`)
- White line detection via OpenCV threshold + `findContours` + `fitLine`
- ROI: lower 80% of frame (`ROI_START=0.2`)
- `cx` extrapolated to bottom of ROI (not bounding box center) for accurate left/right assignment
- Memory-based lane tracking (`last_left_cx`, `last_right_cx`, `MAX_JUMP_PX=150`) to prevent cross-lane false assignment
- Memory only updated when `cx` is within frame bounds (prevents out-of-frame cx poisoning memory)
- `MIN_LANE_WIDTH_PX=300` check to reject spurious close-together contours

**Steering cases (in priority order):**
1. **BOTH lines visible**: `error = |left_angle| - |right_angle|`, `steering = clip(-error, ±MAX_STEERING)`
2. **Single line — exits same side** (`cx` and angle point same direction outward): immediate `±SHARP_CORNER_STEERING`, no delay
3. **Single line — crosses to center** (`cx` on one side, angle pointing toward center): immediate `±SHARP_CORNER_STEERING`, no delay
4. **Single line — steep angle** (`|angle| >= SHARP_CORNER_ANGLE_DEG`): `±SHARP_CORNER_STEERING` after `CORNER_DELAY_S` delay
5. **Single line — normal**: `±MAX_STEERING * 0.25` (left) or `±MAX_STEERING * 0.5` (right)
6. **No lines, horizontal detected**: steer away from side where horizontal line is (`cx < mid` → steer right)
7. **Nothing**: `steering = 0`

**Key constants:**
- `MAX_STEERING = 50`
- `SHARP_CORNER_STEERING = 90`
- `SHARP_CORNER_ANGLE_DEG = 50`
- `CORNER_DELAY_S = 0.2`
- `THRESHOLD = 190` (white detection)

### Camera Publisher (`camera_publisher.py`)
- Publishes `/camera_image` at ~62 Hz (`timer_period=0.016`)
- Resolution set to 320×240 for faster processing

### Teensy Sketch (`sketch_Servo360_microROS.ino`)
- Subscribes to `micro_ros_fabinal_subscriber`
- Normal mode (`|steering| < 90`): differential correction, `factor=2.2`
- Sharp corner mode (`|steering| >= 90`): inner wheel stops (1500µs), outer wheel drives with `SHARP_CORNER_STEERING_VALUE=45` correction
- Obstacle stop mode (`|steering| >= 500`): both wheels set to 1500µs (full stop)
- State machine: `WAITING_AGENT` → `AGENT_AVAILABLE` → `AGENT_CONNECTED` → `AGENT_DISCONNECTED`
- **IMPORTANT**: Must be reflashed via Arduino IDE after any `.ino` changes — changes are not live until uploaded to the Teensy

### Lidar Filter (`simple_lidar_filter.cpp`)
- Subscribes to `/scan`, publishes `/scan_filtered`
- Configurable angle range via params `angle_begin` (default -80°) and `angle_end` (default 80°)

### Obstacle Detector (`obstacle_detector.cpp`)
- Subscribes to `/scan_filtered`
- Detects if any reading < `detection_distance` (default 0.5m)
- **Issue**: publishes to wrong topic (`micro_ros_algorri_subscriber`) and sends 0/1 flag instead of overriding steering

---

## Identified Problems & Solutions

| Problem | Solution | Status |
|---|---|---|
| `PyLong_Check` crash | `steering = int(MAX_STEERING * 0.5)` — float passed to Int32 | Fixed |
| Wrong `cx` for angled lines | Extrapolate `fitLine` to bottom of ROI instead of using bounding box center | Fixed |
| Corner detection too sensitive | Confirmation window (later replaced by geometry-based checks) | Fixed |
| Wrong corner steering direction | `left is None` → steer left (was inverted) | Fixed |
| Gegenspur (adjacent lane) false detection | Take only contour closest to last known position per side + MIN_LANE_WIDTH_PX | Fixed |
| Memory poisoned by out-of-frame cx | Only update memory when `cx` within `[0, width]` | Fixed |
| `CONFIRM_WINDOW_SIZE=1` but `MIN_HITS=4` | Corner never fired — set MIN_HITS=1 | Fixed |
| `SHARP_CORNER_STEERING < SPOT_TURN_THRESHOLD` | Teensy one-wheel-stop never triggered — raised to 90 | Fixed |
| Asymmetric single-line steering | LEFT ONLY gave half of RIGHT ONLY — partially corrected | Ongoing |
| Camera too slow at high speed | Reduced resolution to 320×240, timer to 0.016s | Fixed |
| `CAP_PROP_FPS=60` crashes GStreamer pipeline | Removed FPS cap, rely on native camera rate | Fixed |
| Obstacle detector on wrong topic | Changed topic to `obstacle_detected` | Fixed |
| QoS mismatch on `/scan` | Filter subscribed `RELIABLE`, RPLidar publishes `BEST_EFFORT` → no data received → obstacle never detected. Fixed by using `SensorDataQoS()` for `/scan` subscription | Fixed |

---

## Lidar Obstacle Stop — Implementation Plan

**Approach**: Subscribe to `/scan_filtered` inside `correction_vector.py`. If any valid range reading is below `OBSTACLE_STOP_DISTANCE_M` (0.15m), publish `steering = 0` (stop) and skip all camera-based steering logic for that frame.

**Why in correction_vector.py**: Centralizes the final steering decision in one node. The existing `obstacle_detector.cpp` publishes to a separate topic which would require a third node to arbitrate — unnecessary complexity.

**Status**: Implemented — hybrid architecture with dynamic lidar steering:

### Dynamic Lidar Angle Shifting
`correction_vector.py` calls `/lidar_filter/set_parameters` after each steering publish to shift the lidar window proportionally to the steering value:
- Default window: `±20.0°` (40° total)
- Normal steering (`|steering| < 90`): `center = (steering / SHARP_CORNER_STEERING) * LIDAR_MAX_SHIFT_DEG`
- Sharp corner (`|steering| >= 90`): fixed shift of `±LIDAR_CORNER_SHIFT_DEG`
- Exception: steering=500 (obstacle stop) does not trigger a lidar angle update
- `LIDAR_HALF_WIDTH_DEG = 10.0°`, `LIDAR_MAX_SHIFT_DEG = 30.0°`, `LIDAR_CORNER_SHIFT_DEG = 75.0°`

| Steering | Lidar window |
|---|---|
| 0 | -10° to +10° |
| ±50 (normal curve) | ±16.7° to ±36.7° |
| ±90 (sharp corner) | ±65° to ±85° |
| 500 (obstacle stop) | frozen at last position |


- `obstacle_detector.cpp` publishes `1` to topic `obstacle_detected` when any ray in `/scan_filtered` < `detection_distance`
- `correction_vector.py` subscribes to `obstacle_detected` (0/1) via `obstacle_callback`, sends `500` to Teensy when triggered
- Teensy catches `|steering| >= 500` and stops both motors (`writeMicroseconds(1500)`)
- Distance threshold set to `0.35m` in `lidar_system.launch.py`

## Launch Commands

```bash
# Terminal 1 — Lidar + filter + obstacle detector (detection_distance=0.30m, angle ±10° default)
sinstall
ros2 launch lidar_logger lidar_system.launch.py

# Terminal 2 — Camera publisher
sinstall
ros2 run camera camera

# Terminal 3 — Lane detection + obstacle stop + dynamic lidar angle shifting
sinstall
ros2 run camera correction_vector

# Terminal 4 — micro-ROS agent LAST (car starts driving immediately when agent connects)
cd ~/microros_ws && source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-Teensyduino_USB_Serial_14091700-if00
```

**Verify lidar angle is shifting dynamically:**
```bash
sinstall
ros2 param get /lidar_filter angle_begin
```

## USB Port Conflict Note
Both Teensy and RPLidar may claim the same `/dev/ttyUSB0` or `/dev/ttyACM0`. Use stable paths:
```bash
ls -l /dev/serial/by-id/
```
Then use the full `/dev/serial/by-id/...` path for both the micro_ros_agent and the rplidar serial_port parameter.

---

## Week 2 — Overtaking Maneuver Plan

### Goal
Robot must overtake a fixed 8×8cm obstacle (always inside the 30cm track) without stopping. Must complete 4 maneuvers and return to lane-following after each.

### Chosen Approach: Hard-coded Timed Open-Loop State Machine in `correction_vector.py`

**Trigger**: `obstacle_detected == 1` (same as Week 1 stop) while robot is already moving (state is `LANE_FOLLOWING`).

**Side detection**: On trigger, sample the raw `/scan` topic over a ±45° window. The median range on the left arc (0° to +45°) vs. the right arc (-45° to 0°) tells which side has more clearance. Obstacle closer on left → go right; obstacle closer on right → go left. This is a single snapshot taken once at trigger time, not continuously.

**Architecture**: Pi detects obstacle → publishes `200` (right) or `-200` (left) once → Teensy runs full odometry maneuver blocking → publishes `1` on `overtake_done` → Pi resumes camera.

**Teensy maneuver sequence** (tune in `doOvertake()` in the sketch):
| Phase | Action | Value |
|---|---|---|
| 1 | `turnSetAngle(±60°)` | turn away from obstacle |
| 2 | `driveSetDistance(0.25m, 1)` | drive past obstacle |
| 3 | `turnSetAngle(∓60°)` | turn back parallel |
| 4 | `driveSetDistance(0.25m, 1)` | drive back into lane |
| 5 | `turnSetAngle(±60°)` | realign to original heading |

**Topics**: `micro_ros_fabinal_subscriber` (Pi→Teensy trigger), `overtake_done` (Teensy→Pi done signal)
**Teensy special values**: `200` = overtake right, `-200` = overtake left
**Requires Teensy reflash after any `.ino` changes**

**How to derive T1/T2/T3 experimentally:**
1. Place robot 15cm from obstacle, measure actual forward speed at base PWM (approx 0.2–0.3 m/s).
2. Obstacle is 8cm deep; add ~5cm safety margin each side → need ~18cm of forward travel during T2. At 0.25 m/s: T2 ≈ 0.7 s.
3. Track is 30cm wide, obstacle up to ~22cm to one side → lateral displacement needed ≈ 10–15cm. At turning radius ~20cm for sharp corner PWM: T1 ≈ 0.4–0.6 s. T3 = T1 (symmetric geometry).
4. Run a single straight-line distance calibration: send steering=0 for 1.0 s, measure distance. Use that ratio for T2.
5. Run a single turn calibration: send steering=90 for T seconds, measure lateral offset. Use that ratio for T1/T3.

**State machine enum** (added to `correction_vector.py`):
```
LANE_FOLLOWING → OVERTAKE_STEER_OUT → OVERTAKE_STRAIGHT → OVERTAKE_STEER_BACK → LANE_FOLLOWING
```

**Integration points in `correction_vector.py`:**
- Add `self.overtake_state` and `self.overtake_phase_start` instance variables.
- In `obstacle_callback`: instead of just setting `self.obstacle_detected = True`, also store the current `/scan` snapshot for side detection. Only trigger overtake if `self.overtake_state == LANE_FOLLOWING`.
- Add a new timer (e.g. 20 Hz = `create_timer(0.05, self._overtake_timer)`). This timer drives the state machine independently of camera frames.
- During overtake phases T1–T3: the timer publishes the override steering value; the `callback` (camera) is suppressed (same `if self.obstacle_detected: return` guard, but gated by `overtake_state != LANE_FOLLOWING`).
- On re-entry to `LANE_FOLLOWING`: reset `self.last_left_cx = None`, `self.last_right_cx = None` so lane memory is rebuilt fresh (avoids stale memory from pre-overtake position).
- Set lidar detection distance to a larger value (0.25–0.30m) during Week 2 to give more reaction time.

**Key risk — obstacle still present after T3**: If T2 is too short, the robot returns to lane with obstacle still in detection window → immediate re-trigger. Mitigations:
- After completing T3, set `self.overtake_cooldown = True` for 1.0 s; ignore obstacle signals during cooldown.
- Measure T2 conservatively (obstacle length 8cm + robot body length + 5cm margin).

### Alternative Approaches

**Option A — Lidar-guided lateral control (closed-loop)**
Use the side-facing lidar slices (±30–60°) as a continuous distance signal. Steer to maintain a fixed clearance of ~15cm from the obstacle side wall. More robust to speed variation. Complexity: medium-high. Requires tuning a proportional controller. Risk: lidar A1 has ~8Hz update rate, slow at speed.

**Option B — Camera-only obstacle avoidance**
Detect that one lane line has disappeared (obstacle is blocking it) and steer toward the visible line. Already partially supported by the existing single-line logic. Complexity: low (reuse existing code). Risk: at 15cm distance there may be no time to react; single-line logic was tuned for corners, not obstacles; will re-trigger erratically.

**Option C — Hybrid: timed maneuver + lane re-acquisition check**
Same timed T1/T2/T3 as the primary approach, but after T3 ends, wait up to 1.0 s for the camera to confirm both lane lines are visible before releasing control. Complexity: medium. Reliability: best of all options — guards against incomplete maneuver and avoids hard-coding perfect timing. Recommended if timed approach is unreliable on the track.

### No Teensy changes required
The Teensy already handles `|steering| >= 90` for sharp corners and `|steering| >= 500` for stop. The overtake maneuver only uses steering values already supported (0, ±90). No reflash needed unless motor tuning requires new base values.
