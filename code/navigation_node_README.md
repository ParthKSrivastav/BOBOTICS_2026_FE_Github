# Navigation Node: `navigation_node_colour`

`navigation_node_colour` makes the driving decisions for an autonomous WRO Future Engineers robot running ROS 2 Humble on a Raspberry Pi 4B. Each control tick, it combines IMU heading, front and rear TF-Luna distances, and camera colour detections, then publishes a single throttle and steering command on `/cmd_vel`.

The node handles:

- **Lap driving:** holds a straight heading using the IMU and turns at corners using the front TF-Luna.
- **Wall recovery:** reverses away when the robot gets too close to a wall.
- **Obstacle avoidance:** goes around red and green pillars (passing green on the right, red on the left).
- **Parking:** after three laps, searches for the magenta parking marker and runs the parking manoeuvre.
- **Start/stop control:** stays stationary until the physical start button is pressed.
- **Logging:** writes the robot's state to a log file 10 times per second for tuning and debugging.

---

## Contents

1. [How it fits into the robot](#how-it-fits-into-the-robot)
2. [Topics](#topics)
3. [Control loop overview](#control-loop-overview)
4. [Heading estimation (IMU)](#heading-estimation-imu)
5. [Main driving state machine](#main-driving-state-machine)
6. [Obstacle avoidance](#obstacle-avoidance)
7. [Parking](#parking)
8. [Start/stop button behaviour](#startstop-button-behaviour)
9. [Tuning parameters](#tuning-parameters)
10. [Logging](#logging)
11. [Building and running](#building-and-running)
12. [Testing](#testing)
13. [Troubleshooting](#troubleshooting)
14. [Known limitations and future work](#known-limitations-and-future-work)
15. [Safety](#safety)

---

## How it fits into the robot

```text
button_node ──── /robot_enabled ───────────────┐
imu_node ─────── /imu/data ────────────────────┤
tf_luna nodes ── /tf_luna/front, /tf_luna/rear ┤
camera bridge ── /camera/* ────────────────────┤
                                               ▼
                                     navigation_node_colour
                                               │
                                          /cmd_vel
                                               ▼
                                   motor_node ── L298N ── motors
```

Design principles:

- **One decision-maker.** Only this node decides how the robot moves. Sensors only report data.
- **One motor owner.** Only `motor_node` touches the motor GPIO pins. Navigation never drives the hardware directly.
- **No threads.** All logic runs in a single 50 Hz ROS timer, so commands never conflict.
- **Fail-safe start.** The robot cannot move until the button enables it and IMU data has arrived.

The camera runs outside the ROS Docker container. Its colour results are bridged into ROS topics.

---

## Topics

### Subscribed

| Topic | Type | Meaning |
|---|---|---|
| `/robot_enabled` | `std_msgs/Bool` | `True` = run, `False` = stop (from the button node) |
| `/imu/data` | `sensor_msgs/Imu` | Orientation quaternion, used for yaw |
| `/tf_luna/front` | `std_msgs/Float32` | Front distance in **metres** (best-effort QoS, depth 1) |
| `/tf_luna/rear` | `std_msgs/Float32` | Rear distance in **metres** |
| `/camera/green_detected` | `std_msgs/Bool` | Green pillar visible |
| `/camera/green_x` | `std_msgs/Float32` | Green pillar horizontal position (pixels) |
| `/camera/red_detected` | `std_msgs/Bool` | Red pillar visible |
| `/camera/red_x` | `std_msgs/Float32` | Red pillar horizontal position (pixels) |
| `/camera/magenta_detected` | `std_msgs/Bool` | Magenta parking marker visible |
| `/camera/magenta_x` | `std_msgs/Float32` | Magenta marker horizontal position (pixels) |
| `/camera/magenta_area` | `std_msgs/Float32` | Magenta marker contour area (pixels²) |

### Published

| Topic | Type | Meaning |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` = throttle, `angular.z` = steering |

`motor_node` reads `/cmd_vel` as **throttle and steering values**, not as velocities in m/s or rad/s. Positive `linear.x` drives forward and negative reverses. The sign of `angular.z` sets the steering direction.

### Parameter

| Parameter | Default | Note |
|---|---|---|
| `mode` | `OPEN` | Declared for selecting Open/Obstacle challenge behaviour. Not currently used by the logic. |

---

## Control loop overview

`navigation_loop()` runs every **0.02 s (50 Hz)**. Each tick runs these checks in order and stops at the first one that applies:

```text
1. Button disabled?          → stop, return
2. No IMU data yet?          → stop, return
3. Write log line (10 Hz)
4. Parking manoeuvre active? → run parking logic, return
5. 3 laps done?              → search for magenta / start parking, return
6. Otherwise                 → run the main driving state machine
```

Because each tick publishes only one command, the robot always has exactly one clear instruction.

---

## Heading estimation (IMU)

`imu_callback()` converts the IMU quaternion to a yaw angle in degrees:

```python
siny = 2 * (w*z + x*y)
cosy = 1 - 2 * (y*y + z*z)
raw_yaw = degrees(atan2(siny, cosy))
```

### Relative heading

The yaw is measured relative to a starting direction (`yaw_offset`):

- When the node starts, the first IMU reading becomes 0°.
- Every **START** button press resets the offset, so 0° is wherever the robot is pointing on the start line.

### Smoothing

A low-pass filter reduces IMU noise:

```text
filtered_yaw += filter_alpha × (relative_yaw − filtered_yaw)
```

- `filter_alpha = 0.15`: lower values are smoother but react more slowly.
- The difference is wrapped to ±180° first, so the filter doesn't jump at the −180°/+180° boundary.

### Heading hold

To drive straight, `heading_hold()` uses a proportional controller:

```text
error    = target_yaw − yaw            (wrapped to ±180°)
steering = kp × error                  (kp = 0.027)
steering = clamp(steering, ±0.70)
```

For example, if the robot drifts 10° off its target heading, the steering command is 0.27 back towards it.

---

## Main driving state machine

| State | What the robot does | Exit condition |
|---|---|---|
| `DRIVING` | Holds `target_yaw` at `drive_speed` (0.30). Checks for pillars. | Pillar seen → `OBSTACLE`; front < 0.30 m → `REVERSE`; front ≤ 0.80 m → `TURNING` |
| `TURNING` | Turns at throttle 0.30, steering 0.60 | Turned ≥ 87° **or** front ≥ 5.0 m → `TURN_EXIT`; front ≤ 0.30 m → `TURN_REVERSE` |
| `TURN_REVERSE` | Reverses at −0.40 with steering −0.75 | Front ≥ 0.60 m for 5 ticks in a row → `TURNING` |
| `TURN_EXIT` | Adds 1 to `turn_count`, sets the new heading | Immediately → `DRIVING` |
| `REVERSE` | Reverses straight at −0.40 | Front ≥ 0.60 m → `TURNING` |
| `OBSTACLE` | Runs the obstacle state machine | Obstacle sequence complete → `DRIVING` |

```text
            front ≤ 0.80 m                 turned ≥ 87°
 DRIVING ──────────────────► TURNING ───────────────────► TURN_EXIT ──► DRIVING
    │  ▲                       │  ▲                                     (turn_count += 1)
    │  │                       │  │ front ≥ 0.60 m (5 ticks)
    │  │            front≤0.30 ▼  │
    │  │                     TURN_REVERSE
    │  │
    │  └──── obstacle done ───┐
    ▼ pillar seen             │
 OBSTACLE ────────────────────┘

 DRIVING ── front < 0.30 m ──► REVERSE ── front ≥ 0.60 m ──► TURNING
```

### Corner detection

The front TF-Luna detects corners. When the wall ahead is within `turn_distance` (0.80 m), the robot starts turning. The turn finishes when either:

- the IMU shows ≥ `turn_target_angle` (87°), or
- the front sensor suddenly reads ≥ `turn_disarm_distance` (5.0 m), meaning the robot is now looking down the next straight.

87° is used instead of 90° because the robot keeps turning slightly after the steering command changes, and the heading controller corrects the last few degrees.

### Lap counting

Each completed corner adds 1 to `turn_count`. The code assumes **4 corners per lap**, so after **12 turns** (3 laps) the robot moves into the parking phase.

### Turn direction

`TURNING` always uses positive `turn_speed` (0.60), so the robot turns the same way at every corner. That suits a run in one fixed direction around the track. To drive the other direction, use a negative steering value. See [Known limitations](#known-limitations-and-future-work).

---

## Obstacle avoidance

### Trigger

`colour_detection()` runs during `DRIVING`. A pillar triggers avoidance only if it is detected **and** in the central part of the camera image:

```text
150 < x < 400   (pixels)
```

| Colour | WRO rule | Direction | Target heading |
|---|---|---|---|
| Green | Pass on the **left** side of the pillar (robot goes left)* | `RIGHT` label | start yaw **+45°** |
| Red | Pass on the **right** side of the pillar* | `LEFT` label | start yaw **−45°** |

\*Whether +45° turns the robot left or right depends on how the IMU is mounted and its sign convention. Check this on the robot (see [Testing](#testing)) and swap the signs if the robot passes on the wrong side.

### Obstacle state machine

| State | Action | Exit condition |
|---|---|---|
| `TURN_OUT` | Steers ±0.60 at throttle 0.30 towards the 45° target | Within 3° of target, or turned ≥ 90° (safety limit) → `PASS` |
| `PASS` | Drives straight at 0.35 | Front ≥ 1.00 m (path clear) → `TURN_BACK` |
| `TURN_BACK` | Steers back to the original heading | Within 3° → done, back to `DRIVING` |
| `REVERSE` | Backs up straight at −0.20 | Front ≥ 0.60 m for 5 ticks → `TURN_OUT` with a new start heading |

Any obstacle state switches to `REVERSE` if the front distance drops to ≤ 0.30 m.

```text
pillar ─► TURN_OUT ─► PASS ─► TURN_BACK ─► DRIVING
             ▲  (front ≤ 0.30 m from any state)
             └──── REVERSE ◄─────────┘
```

Steering in `TURN_OUT` and `TURN_BACK` is **bang-bang**: full ±0.60 until the robot is within 3° of the target. It is simple and predictable, but it can overshoot at higher speeds.

A new pillar can't start another avoidance until the current one finishes (`obstacle_state` must be `NONE`).

---

## Parking

Parking starts after the 12th corner.

### 1. Searching

When `turn_count ≥ 12` and the robot is on the parking straight:

- `parking_state` changes from `DISABLED` to `SEARCHING`.
- The robot holds its heading at the slower `parking_search_speed` (0.16).
- Normal corner and obstacle logic is **skipped**, so the robot doesn't start an extra lap.

### 2. Confirming the magenta marker

To avoid false starts, a detection only counts as valid when **all** of these are true:

| Check | Value |
|---|---|
| Camera reports magenta | `magenta_detected == True` |
| Marker large enough | `magenta_area ≥ 300` px² |
| Detection is recent | last seen < 0.30 s ago |

The marker must be valid for **5 ticks in a row** (`magenta_required_ticks`), which is 0.1 s at 50 Hz. A single bad frame resets the count.

### 3. Parking turn (current version)

Once the marker is confirmed:

| State | Action | Exit |
|---|---|---|
| `TURN_IN` | Turns at throttle 0.30, steering 0.60 | Turned ≥ 45° → `PARKED`; more than 10 s → `ABORT` |
| `PARKED` | Stops | Final |
| `ABORT` | Stops and logs a warning | Final |

**This is a test version of parking.** It checks the trigger and a 45° turn-in. The full reverse parking manoeuvre (approach, reverse at `parking_entry_angle`, straighten using the IMU, stop on rear TF-Luna distance) is set up with tuning values but not yet implemented. See [Known limitations](#known-limitations-and-future-work).

---

## Start/stop button behaviour

| Event | Result |
|---|---|
| Node starts | `robot_enabled = False`; robot publishes zero commands |
| START press (`/robot_enabled` → `True`) | `reset_run()` runs, then driving begins |
| STOP press (`/robot_enabled` → `False`) | Robot stops at once |
| Repeated messages (the button node republishes at 5 Hz) | Ignored unless the value changes |
| No IMU data received yet | Robot stays stopped even if enabled |

`reset_run()` resets everything that belongs to a single run:

- Heading: offset re-zeroed, `yaw`, `filtered_yaw` and `target_yaw` set to 0.
- Laps: `state = DRIVING`, `turn_count = 0`.
- Obstacles: all obstacle state cleared.
- Parking: `parking_state = DISABLED`, `parking_attempted = False`, `in_parking_straight = False`, magenta tick count reset.

So every START is a clean run from the start line.

---

## Tuning parameters

All values are set in `__init__`. Distances are in **metres** and angles in **degrees**. Speeds and steering are motor-command values (roughly −1.0 to 1.0).

### Heading

| Variable | Value | Effect of increasing |
|---|---|---|
| `filter_alpha` | 0.15 | Faster yaw response, more noise |
| `kp` | 0.027 | Stronger heading correction; too high causes weaving |
| `max_steering` | 0.70 | Allows larger corrections |

### Driving and corners

| Variable | Value | Meaning |
|---|---|---|
| `drive_speed` | 0.30 | Straight-line throttle |
| `turn_throttle` | 0.30 | Throttle while turning |
| `turn_speed` | 0.60 | Steering while turning |
| `reverse_speed` | 0.40 | Reverse throttle (applied as negative) |
| `turn_distance` | 0.80 | Start turning when the front wall is this close |
| `turn_target_angle` | 87.0 | IMU angle that finishes a corner |
| `turn_disarm_distance` | 5.0 | Front reading that also finishes a corner |
| `reverse_enter` | 0.30 | Too close: start reversing |
| `reverse_threshold` | 0.60 | Far enough: stop reversing |

### Obstacles

| Variable | Value | Meaning |
|---|---|---|
| `obstacle_turn_speed` | 0.60 | Steering during turn-out and turn-back |
| `obstacle_drive_speed` | 0.35 | Throttle while passing |
| `obstacle_clear_distance` | 1.00 | Front distance that counts as clear |
| `max_obstacle_turn_angle` | 90.0 | Safety cap on the turn-out angle |
| Trigger window | 150–400 px | Pillar must be in this x-range (hard-coded in `colour_detection()`) |

### Parking

| Variable | Value | Meaning |
|---|---|---|
| `parking_search_speed` | 0.16 | Speed while looking for magenta |
| `magenta_min_area` | 300 | Minimum marker size (px²) |
| `magenta_required_ticks` | 5 | Consecutive valid frames needed |
| `magenta_timeout_s` | 0.30 | Maximum age of a detection |
| `parking_turn_angle` | 45.0 | Turn-in angle (current test version) |
| `parking_turn_timeout_s` | 10 | Abort if the turn takes longer |

Set up for full parking but **not used yet**: `parking_approach_speed`, `parking_forward_time`, `parking_reverse_speed`, `parking_straighten_speed`, `parking_entry_angle`, `parking_parallel_yaw_tolerance`, `parking_straighten_distance`, `parking_stop_distance`, `parking_emergency_stop_distance`, `parking_timeout_s`.

---

## Logging

The node writes to `~/robot_log.txt` at **10 Hz**. The file is **overwritten each time the node starts**.

Example line:

```text
[13:42:07.215] STATE=TURNING | PARKING=DISABLED | Yaw=42.18 | TargetYaw=0.0 | Front=0.64 | Rear=1.92 | Magenta=False | MagentaArea=0 | MagentaTicks=0
```

Useful commands:

```bash
tail -f ~/robot_log.txt                         # watch live
grep "STATE=TURN" ~/robot_log.txt | head        # inspect corners
grep "MagentaTicks=[1-9]" ~/robot_log.txt       # parking detections
cp ~/robot_log.txt ~/logs/run_$(date +%H%M).txt # save a run before restarting
```

The log is the main tuning tool. After a bad run, find the moment things went wrong and check the state, yaw and distances at that time.

---

## Building and running

Inside the ROS 2 Humble container:

```bash
cd ~/ros2_ws
python3 -m py_compile src/tf_stack/tf_stack/navigation_node_colour.py && echo "Syntax OK"
colcon build --packages-select tf_stack
source install/setup.bash
```

Start all nodes. Navigation stays stationary until the button is pressed:

```bash
ros2 run tf_stack imu_node
ros2 run tf_stack tf_luna_front        # plus the rear TF-Luna node
ros2 run tf_stack motor_node
ros2 run tf_stack button_node
ros2 run tf_stack navigation_node_colour
```

> `ros2 run` uses the copy in `install/`, not `src/`. Rebuild after every edit, or build once with `colcon build --symlink-install`.

Use your package's real executable names. List them with `ros2 pkg executables tf_stack`.

---

## Testing

Test in stages, from safest to most realistic. Don't move to the next stage until the current one passes.

### Stage 0: static checks

```bash
python3 -m py_compile src/tf_stack/tf_stack/navigation_node_colour.py
grep -n "def " src/tf_stack/tf_stack/navigation_node_colour.py   # every method at 4-space indent
```

Every method called in `navigation_loop()` must exist with exactly that name. Otherwise Python only crashes when that branch first runs, which might be in the middle of a run.

### Stage 1: ROS wiring

With every node running:

```bash
ros2 node list
ros2 topic info /robot_enabled      # 1 publisher, 1 subscriber
ros2 topic info /cmd_vel            # 1 publisher (navigation), 1 subscriber (motor)
ros2 topic hz /imu/data             # steady rate
ros2 topic hz /tf_luna/front        # steady rate
ros2 topic echo /camera/red_detected
```

**Pass:** every expected node is listed, every topic has a publisher, and sensor rates are steady.

### Stage 2: start/stop (wheels off the ground)

1. Lift the robot so the wheels spin freely.
2. Watch the motor commands with `ros2 topic echo /cmd_vel`.
3. Before pressing the button, check that `/cmd_vel` is all zeros.
4. Press START. You should see `START pressed: new run started.` once, and `linear.x` should go to 0.30.
5. Press STOP. You should see `STOP pressed: robot stopped.` and zeros straight away.
6. Press START again and check in the log that `turn_count` and the state were reset.

Without the button:

```bash
ros2 topic pub --once /robot_enabled std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /robot_enabled std_msgs/msg/Bool "{data: false}"
```

### Stage 3: sensor sign checks (wheels off the ground)

**IMU direction.** Enable the robot, then rotate it by hand and watch `Yaw` in `tail -f ~/robot_log.txt`:

- Rotate in the robot's normal turning direction. Yaw should move steadily towards ±90°.
- Check that the heading-hold steering pushes **back** towards 0°. If it pushes away, the steering sign is reversed. Fix it in `motor_node` or flip `kp`.

**Front TF-Luna.** Move your hand towards the front sensor:

- Below 0.80 m, the state should change to `TURNING`.
- Below 0.30 m, it should change to `REVERSE` / `TURN_REVERSE` and `linear.x` should go negative.

**Camera.** Hold a red pillar, then a green pillar, in the centre of the camera view:

- The state should change to `OBSTACLE` and the log should show `RED -> LEFT` or `GREEN -> RIGHT`.
- Check which way the wheels steer. If the robot would pass on the wrong side, swap the ±45° signs.

### Stage 4: simulated distances (optional)

Stop the real TF-Luna node and publish fake distances to test each transition:

```bash
ros2 topic pub -r 20 /tf_luna/front std_msgs/msg/Float32 "{data: 2.0}"   # open straight
ros2 topic pub -r 20 /tf_luna/front std_msgs/msg/Float32 "{data: 0.7}"   # corner
ros2 topic pub -r 20 /tf_luna/front std_msgs/msg/Float32 "{data: 0.2}"   # too close
```

This lets you test the state machine at a desk, with the expected state for each distance shown in the log.

### Stage 5: on the mat, step by step

Start each test at low speed (for example, `drive_speed = 0.20`), then build up.

| Test | Setup | Pass criteria |
|---|---|---|
| 5a. Straight line | One straight, no corners | Stays within a few cm of a straight line; no weaving |
| 5b. Single corner | Start before one corner | Turns once, `turn_count = 1`, exits parallel to the next wall |
| 5c. One lap | Empty track | 4 corners, no wall contact, `turn_count = 4` |
| 5d. Three laps | Empty track | 12 corners, then `PARKING=SEARCHING` in the log |
| 5e. One pillar | A single red or green pillar | Passes on the correct side and returns to its heading |
| 5f. Mixed pillars | Competition-style layout | All pillars passed correctly, no contact |
| 5g. Parking trigger | Magenta marker on the parking straight | `MagentaTicks` rises to 5, `PARKING TURN STARTED` logged |
| 5h. Parking turn | As above | Turns about 45° and stops (`PARKED`) |
| 5i. Full run | Complete competition setup | 3 laps, pillars and parking, repeated 5 times |

### Stage 6: repeatability

A single good run doesn't prove much. For each layout:

- Run it **at least 5 times** and record pass or fail.
- Change the start position, battery level and lighting.
- Save every log with `cp ~/robot_log.txt ~/logs/run_N.txt`.
- Change **one parameter at a time**, and note what you changed and the result.

Example test record:

| Run | Layout | Battery (V) | Result | Notes |
|---|---|---|---|---|
| 1 | Empty, clockwise | 9.1 | Pass | |
| 2 | Empty, clockwise | 8.9 | Fail | Corner 7 too wide; front read 0.95 m at turn start |
| 3 | 4 pillars | 9.0 | Pass | |

### Tuning guide

| Symptom | Likely fix |
|---|---|
| Weaves on straights | Lower `kp` or increase smoothing (lower `filter_alpha`) |
| Drifts slowly off line | Raise `kp` slightly |
| Turns too late, hits the outer wall | Increase `turn_distance` |
| Turns too early, cuts the inner corner | Decrease `turn_distance` |
| Turns too far | Lower `turn_target_angle` |
| Turns too little | Raise `turn_target_angle` |
| Misses pillars at the edge of the view | Widen the 150–400 px trigger window |
| Reacts to pillars too early or late | Adjust the trigger window or camera angle |
| Parking triggers on noise | Raise `magenta_min_area` or `magenta_required_ticks` |
| Parking never triggers | Check `/camera/magenta_area` values and lower `magenta_min_area` |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Button pressed, nothing happens | No publisher on `/robot_enabled` | `ros2 topic info /robot_enabled`; start `button_node` |
| START logged, robot doesn't move | No IMU data or `motor_node` not running | `ros2 topic hz /imu/data`; `ros2 node list` |
| `/cmd_vel` changes but wheels don't move | Motor power or wiring | Check the L298N supply and battery voltage |
| START prints many times | `self.robot_enabled` never updated (typo) | Check the assignment in `enabled_callback` |
| `AttributeError: ... has no attribute` | Method missing, misspelled or wrongly indented | Check the method name and that it is indented 4 spaces |
| `IndentationError` | Mixed tabs and spaces | `sed -i 's/\t/    /g' file.py` |
| `RCLError: publisher's context is invalid` on Ctrl+C | Publishing after ROS shut down | Guard `node.stop()` with `if rclpy.ok():` |
| Edits have no effect | Running the old build | `colcon build`, then `source install/setup.bash` |
| Front distance stuck at 999.0 | TF-Luna node not publishing | `ros2 topic hz /tf_luna/front` |
| Robot spins in place / keeps turning | Front never reads ≥ 5 m and the IMU angle never reaches 87° | Check the IMU rate and `Yaw` in the log |

---

## Known limitations and future work

- **Turn direction is fixed.** All corners use positive steering. Add a direction parameter (clockwise or anticlockwise) detected at the start of the run.
- **Lap counting assumes 4 corners per lap.** A missed or extra corner will make parking start at the wrong time.
- **Parking is a test version.** Only the 45° turn-in is done. Still to add: approach, reverse into the space at `parking_entry_angle`, straighten using the IMU, and stop on the rear TF-Luna distance.
- **Bang-bang obstacle steering** can overshoot at higher speeds. A proportional controller like `heading_hold()` would be smoother.
- **The obstacle trigger window (150–400 px) is hard-coded.** Move it into class variables or ROS parameters.
- **`mode` and `magenta_distance` are declared but unused.**
- **Camera data has no freshness check** for red and green. If the camera bridge stops, the last value stays in use.
- **The log file is overwritten** on every start. Add a timestamp to the filename to keep every run.
- **Tuning values are hard-coded.** Making them ROS parameters would allow tuning without rebuilding.

---

## Safety

- The start/stop button is a software control, **not** an emergency stop. Fit a hard-wired switch that cuts motor power.
- Test new code with the wheels off the ground first, then at low speed.
- `motor_node` should stop the motors if no `/cmd_vel` arrives for about 0.3 s, so a crashed navigation node can't leave the motors running.
- Keep hands away from the steering linkage while the robot is enabled.


