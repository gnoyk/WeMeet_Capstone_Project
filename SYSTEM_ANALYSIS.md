# Remote Parking System Analysis

## Table of Contents

1 . [Full Program Flow](# 1 -Full-Program-Flow)
2 . [ LIMO 1 main-algorithm](# 2 -limo -1 -main-algorithm)
3 . [ LIMO 2 main-algorithm](# 3 -limo -2 -main-algorithm)

---

## 1. Overall System Flow

### 1.1 System Overview

This is a remote parking scenario in which the double-parked LIMO 1 pulls out, the user-selected vehicle (one of A2 /A3/A4) pulls out, and LIMO 1 re-parks in the empty slot.

**Parking lot layout (scale 1 /5.6):**

```
                   North Wall (y=1.585)
 ┌───────────────────────────────────────────────────────────────┐
 │ [A-1]  [A0]  [A1]  [A2★] [★A3★] [A4★]  [A5]  [A6]          │
 │-1.575 -1.125 -0.675 -0.225 0.225  0.675  1.125  1.575       │
│ y = 0.985 (slot center) │
 │                                                               │
│──────────────── by car (y ≈ 0, top and bottom edges ± 0.535 ) ──────────────│
 │                                                               │
 │ [B-1]  [B0]  [B1]   [B2]   [B3]   [B4]   [B5]  [B6]         │
 │                      y = -0.985                               │
 └───────────────────────────────────────────────────────────────┘
                   South Wall (y=-1.585)
West Wall (x= -1.90 ) Exit → +X (open to the east)
```

- Static vehicle 13 vs. (fixed placement in Gazebo world):
- Row A: A -1 , A 0 , A 1 , A 5 , A 6 (5)
- Row B: B -1 , B 0 , B 1 , B 2 , B 3 , B 4 , B 5 , B 6 (vs. 8)
- Dynamic robot 4 (spawn at spawn_robots.launch.py):
- LIMO 1: Double parking in front of A3 in lane A
- LIMO 2 : A3 slot parking
- LIMO A2 : A2 slot parking
- LIMO A4 : A4 slot parking
- A2, A3, and A4 are vehicles that can be parked according to user selection (marked with ★)

### 1.2 Package Structure

| package | Type | Role |
|--------|------|------|
| `remote_parking_world` | CMake | Gazebo world, robot URDF /Xacro , Launch file, utility script |
| `limo_car` | CMake |  LIMO robot URDF /Xacro model definition |
| `remote_parking_manager` | Python | master FSM (`mission_manager`) + route planner (`hybrid_astar_planner`) |
| `my_valet_parking` | Python | LiDAR slot detection (`dynamic_tracker_node`), independent exit node (`limo2_exit_handler`), keyboard teleop (`limo2_arrow_teleop`), etc. |

### 1.3 Execution order and node startup timing

```
[Terminal 1] ros2 launch remote_parking_world world_server.launch.py gui:=true
→ Start gzserver (load static world + 13 vs. static vehicle)
→ Start gzclient (GUI)

[Terminal 2] ros2 launch remote_parking_world spawn_robots.launch.py
    → T=0s:  robot_state_publisher × 4 + joint_state_publisher × 4
(limo 1 , limo 2 , limo_a2 , limo_a4 namespace)
→ T= 3s : Spawn entities LIMO 1 , LIMO 2 , LIMO A2 , LIMO A4 in Gazebo
→ T= 5s: Start dynamic_tracker_node (Detect LiDAR slot)
→ T= 6s : Start mission_manager (wait for master FSM and IDLE status)
→ T= 0s : start rviz 2 (visualization)

[Terminal 3] ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'
→ call start_cb() of mission_manager
→ After confirming reception of both robot odometry, transfer FSM to IDLE → LIMO1_EVADE

[Terminal 4 ] bash scripts/select_car.sh a3 (or a2, a4)
→ Select exit vehicle in WAIT_FOR_SELECTION state
→ /select_exit_car/a3 service call → Transition to LIMO2_EXIT_INIT
```

### 1.4 Initial robot deployment

| robot | position (x, y, z) | direction (yaw) | status |
|------|----------------|-----------|------|
|  LIMO 1 | (0.225, 0.35, 0.145) |  0 rad (+X direction) | Double parking in front of A3 in the lane |
|  LIMO 2 | (0.225, 0.985, 0.145) | -π /2 rad (-Y direction) | A3 slot normal parking |
|  LIMO A2 | (-0.225, 0.985, 0.145) | -π /2 rad (-Y direction) | A2 slot normal parking |
|  LIMO A4 | (0.675, 0.985, 0.145) | -π /2 rad (-Y direction) | A4 slot normal parking |

### 1.5 FSM (Finite State Machine) state transition

```
IDLE
│ /start_remote_parking service call
  ▼
LIMO1_EVADE ─────────────────────────────────────────────────────────
│ LIMO 1 Reverse along this lane to secure the exit route.
│ End condition: Target position (-0.9, 0.35) reached OR rear obstacle detected for more than 1.5 seconds
  ▼
WAIT_FOR_SELECTION ──────────────────────────────────────────────────
│ User selects vehicle to exit (one of a2 /a3/a4)
  │  bash scripts/select_car.sh a3
  │  → ros2 service call /select_exit_car/a3 std_srvs/srv/Trigger '{}'
  ▼
LIMO2_EXIT_INIT ─────────────────────────────────────────────────────
│ Calculate exit route (sinusoidal waypoint) of selected vehicle
│ Dynamically updated with the slot coordinates from which repark_goal was withdrawn
│ Immediately transition to the next state
  ▼
LIMO2_EXITING ───────────────────────────────────────────────────────
│ The selected vehicle follows the waypoint using Pure Pursuit and leaves the vehicle.
│ End condition: final waypoint reached (tolerance 0.12m ) OR timeout 30 seconds
  ▼
LIMO1_SCAN ──────────────────────────────────────────────────────────
│ dynamic_tracker_node checks for empty slots
│ End condition: Receive /limo1/target_slot OR timeout 10 seconds (use fallback coordinates)
  ▼
LIMO1_REPARK ────────────────────────────────────────────────────────
│ Hybrid A* route planning → Pure Pursuit tracking by segment → Final Alignment
│ End condition: Mission completion criteria of position error < 4cm and yaw error < 3° maintained for 0.8 seconds, OR timeout 60 seconds
  ▼
FINISH ──────────────────────────────────────────────────────────────
│ Stop all robots (cmd_vel = 0)
  ▼
(Wait IDLE — can be recalled)

※ ABORT: Stop all robots in case of emergency such as path planning failure, timeout, etc.
```

### 1.6 sensor usage flow

#### Runtime node configuration and topic flow

```
┌─────────────────────────────────────────────────────────────────┐
│                         Gazebo                                  │
│  /limo1/odom ──┐   /limo1/scan ──┐   /limo1/rear_camera/... ──┐│
│  /limo2/odom ──┤   /limo2/scan ──┤   /limo1/front_camera/...──┤│
│  /limo_a2/odom─┤                 │   /limo2/rear_camera/... ──┤│
│  /limo_a4/odom─┤                 │                             ││
└────────────────┼─────────────────┼─────────────────────────────┤┘
                 │                 │                              │
                 ▼                 ▼                              ▼
  ┌──────────────────────────────────────────────────────────────┐
  │             mission_manager (remote_parking_manager)         │
  │                                                              │
│ Subscription: │
│ /limo1/odom → Position/yaw extraction │
│ /limo1/scan → Front/rear obstacle distance (± 30° sector) │
│ /limo1/rear_camera → posterior depth image → 5 th percentile distance │
│ /limo1/front_camera → Front depth image │
│ /limo2/odom → LIMO 2 position/yaw extraction │
│ /limo_a2/odom → LIMO A2 position/yaw extraction │
│ /limo_a4/odom → LIMO A4 position/yaw extraction │
│ /limo2/scan → LIMO 2 Front/rear obstacle distance │
│ /limo2/rear_camera → LIMO 2 posterior depth image │
│ /limo1/target_slot → Slot location (received from dynamic_tracker) │
│ /select_exit_car → Select exit vehicle (topic, auxiliary) │
│ /remote_parking/reset → Reset command │
  │                                                              │
│ Service server: │
│ /start_remote_parking → Start mission │
│ /reset_parking → Mission Reset │
│ /select_exit_car/a2 → A2 Select vehicle │
│ /select_exit_car/a3 → A3 Select vehicle │
│ /select_exit_car/a4 → A4 Select vehicle │
  │                                                              │
│ Published by: │
│ /limo1/cmd_vel → LIMO 1 speed command │
│ /limo2/cmd_vel → LIMO 2 speed command │
│ /limo_a2/cmd_vel → LIMO A2 speed command │
│ /limo_a4/cmd_vel → LIMO A4 speed command │
│ /remote_parking/status → Current FSM status string │
  └──────────────────────────────────────────────────────────────┘
                 ▲
                 │ /limo1/target_slot
  ┌──────────────────────────────────────────────────────────────┐
  │            dynamic_tracker_node (my_valet_parking)           │
  │                                                              │
│ Subscription: │
│ /limo1/scan → + 90° ± 8° side scan + full FOV projection │
│ /limo1/odom → ego position correction (odom frame projection) │
  │                                                              │
│ Published by: │
│ /limo1/target_slot → Empty slot position (PoseStamped) │
  └──────────────────────────────────────────────────────────────┘
```

#### Sensor Usage Summary

| sensor | Topic | Use Nodes | Use |
|------|------|----------|------|
| Odometry ( LIMO 1 ) | `/limo1/odom` |  mission_manager , dynamic_tracker | Position·yaw tracking, odom frame coordinate conversion |
| Odometry ( LIMO 2 ) | `/limo2/odom` |  mission_manager | Location/yaw tracking while leaving the car |
| Odometry ( LIMO A2 ) | `/limo_a2/odom` |  mission_manager | Location/yaw tracking while leaving the car |
| Odometry ( LIMO A4 ) | `/limo_a4/odom` |  mission_manager | Location/yaw tracking while leaving the car |
|  2 D LiDAR ( LIMO 1 ) | `/limo1/scan` |  mission_manager , dynamic_tracker | Front/rear obstacle detection, side slot occupancy determination |
|  2 D LiDAR ( LIMO 2 ) | `/limo2/scan` |  mission_manager | Front/rear obstacle detection |
| Rear Depth Camera (LIMO 1) | `/limo1/rear_camera/depth/image_raw` |  mission_manager | Rear obstacle distance when reversing (32 FC1 → 5 th percentile) |
| Front Depth Camera (LIMO 1) | `/limo1/front_camera/depth/image_raw` |  mission_manager | Forward Obstacle Assist Detection |
| Rear Depth Camera (LIMO 2) | `/limo2/rear_camera/depth/image_raw` |  mission_manager | Rear obstacle distance when leaving the car |

#### Obstacle detection method

- **LiDAR sector analysis**: `_ scan_min_in_range ()` — Extract the minimum value of a specified angle range (center ± half angle)
- Front: 0° ± 30° → `front_obstacle_m`
- Rear: 180° ± 30° → `rear_obstacle_m`
- **Depth camera**: encode 32 FC1 → `np.percentile(data, 5 )` (subordinate 5% distance = nearest obstacle)
- **Emergency stop criteria**: Obstacle distance < 0.18m (`ESTOP_DIST_M`)

---

## 2. Core LIMO1 Algorithms

LIMO 1 performs the 3 level mission: **backward avoidance** → **slot detection** → **repark**.

### 2.1 LIMO1_EVADE: Straight Reverse to Clear the Aisle

**Purpose**: Back up the lane to free up space in front of the slot so the selected vehicle can exit.

**Algorithm**: P control based straight backwards

```
Input: current position (cx, cy), current yaw, target position (-0.9, 0.35)
Output: cmd_vel (linear.x, angular.z)

1 . Check for rear obstacles
- rear_obstacle_m < 0.18m → Stop
- Continuous stop for more than 1.5 seconds → End of reverse (transition to WAIT_FOR_SELECTION)

2 . Check if you reached your goal
- dist(current, target) < 0.04m → End of backward movement (transition to WAIT_FOR_SELECTION)

3 . Speed ​​command generation
- linear = -0.12 m/s (fixed reverse speed)
- yaw_err = start yaw - current yaw
- | yaw_err | < 0.03 → No correction required (dead zone)
- angular = clip( -1.2 × yaw_err , -0.3 , 0.3 ) ← P gain = 1.2
```

**Key point**: Set the yaw at the starting point as a reference value and correct it with P control to prevent the vehicle body from twisting during reverse.

### 2.2 WAIT_FOR_SELECTION: Select exit vehicle

**Purpose**: Wait for the user to select which vehicle (A2 /A3/A4 ) to exit.

**How ​​it works**:
- Run `bash scripts/select_car.sh a3 ` in a separate terminal.
- The script calls the `/select_exit_car/a3` service.
- After validation in `_ try_select_car ()`of mission_manager, transition to` LIMO2_EXIT_INIT `
- odom /cmd_vel publisher is dynamically bound according to the selected vehicle

| Select | Odometry topics |  cmd_vel Topic | Initial X coordinate | Re-parking goal
|------|-------------|------------|-----------|-------------|
| a2 | `/limo_a2/odom`|`/limo_a2/cmd_vel` | -0.225 | -0.225 |
| a3 | `/limo2/odom`|`/limo2/cmd_vel` | 0.225 | 0.225 |
| a4 | `/limo_a4/odom`|`/limo_a4/cmd_vel` | 0.675 | 0.675 |

### 2.3 LIMO1_SCAN: LiDAR based slot detection

**Purpose**: Check whether the slot where the selected vehicle left is actually empty and determine the re-parking target coordinates.

**Execution Node**: `dynamic_tracker_node` (separate node, mission_manager only receives results)

#### 2.3. 1 Side Scan Projection

```
1 . Li Extract rays from DAR to +90° ± 8° (left side of robot)
2 . Car body self-interference section (± 130° ~170°) masking
3 . Convert each ray to an odom frame:
   p_odom = [ego_x, ego_y] + R(ego_yaw) × [range×cos(θ), range×sin(θ)]
4 . Statistical output:
- representative_clearance = 70 th percentile distance
- clear_ratio = (number of lays with distance ≥ 0.65m) / (total number of lays)
- close_count = distance ≤ 0.45m number of inlays
```

#### 2.3. 2 Baseline Anchoring

```
1 . If there are more than 3 nearby points (≤ 0.75m ) → Occupied object detected
2 . Anchor the average position of that point into the odom frame (baseline_center_odom)
3 .  Incrementally update the position to EMA (α= 0.2 )
4 . Set up a tracking box ( 0.35m × 0.30m ) around the anchor
```

#### 2.3. 3 Clearance Evaluation

```
1 . Project entire FOV LiDAR into odom frame (not side window)
※ Reason for using all FOV: If ego vehicle moves among EVADE,
Side windows alone can make you miss parked cars
2 . Count the number of points ( roi_count ) in the tracking box
3 . Empty condition:
- roi_count ≤ max( 2 , baseline_roi_count × 0.25 ) ← object disappears
- representative_clearance ≥ 0.65m ← Side open
- clear_ratio ≥ 0.25 ← 25% or more rays are detected far away
4 . The above condition is maintained continuously for 0.70 seconds → slot is confirmed to be empty.

5 . Generate target coordinates:
target = baseline_center + side_unit × 0.30m (slot center offset)
```

**Fallback**: If `target_slot` is not received within 10 seconds, use `repark_goal` parameter coordinates (dynamically determined depending on the selected slot).

### 2.4 LIMO1_REPARK : Hybrid A* path planning + segment-specific tracking

**Purpose**: Park by combining forward and backward from the current location to an empty slot (goal_x, 0.985, yaw=π /2).

#### 2.4. 1 Occupancy Grid

```
Range: x ∈ [ -1.5 , 1.2 ], y ∈ [ -0.6 , 1.7 ]
Resolution: 0.02m/cell
obstacle:
- Among A 0 ( -1.125 ), A 1 ( -0.675 ), A2 ( -0.225 ), A3 ( 0.225 ), A4 ( 0.675 )
Except for exited slots → static vehicle obstacles
- North wall (y= 1.585 ), West wall (x= -1.90 )
- Size of each vehicle: 0.40m × 0.26m (including margin)
```

#### 2.4. 2 Hybrid A* navigation

General A* is grid-based, so it does not reflect the non-holonomic constraints of the vehicle. Hybrid A* searches for continuous states that fit the actual vehicle kinematics while removing redundancy with the grid index.

```
State: (x, y, yaw)
Grid resolution: xy= 0.03m , yaw= 5°
Steering angle: [ -30° , -15° , 0° , + 15° , + 30° ](5 steps)
Direction: forward (+ 1 ), backward ( -1 )
Step size: 0.06m /expansion

Expansion 1 times:
  nx = x + step × dir × cos(yaw + β/2)
  ny = y + step × dir × sin(yaw + β/2)
  nyaw = yaw + β
Where β = step × dir × tan(steer) / wheelbase

Cost function:
Base cost = step (0.06)
Reverse Penalty = × 2.5
Steering Penalty = × 1.1
Direction penalty = + 0.3

Heuristic:
h = dist(current, target) + 0.5 × min_turn_radius × | yaw_err |

Collision check:
Project the entire vehicle footprint (0.31m × 0.19m) + margin (0.02m) onto the grid.

RS connection attempt:
Every 5 expansion, we attempt to connect the Reeds-Shepp curve from the current node to the target.
If there is no collision, the route is confirmed immediately → Search ends early.
```

#### 2.4. 3 Reeds-Shepp curve

It is an analytical method that creates the shortest path between two poses by combining forward, backward, and rotation.

```
3 basic formula: LSL , LSR , LRL
L = turn left, R = turn right, S = go straight

4 transformations (applied to each formula):
Original: (x, y, ϕ) → advance
Time reversal: (-x, y, -ϕ) → backward
Reflection: (x, -y, -ϕ) → L↔R replacement
Time inversion + reflection: (-x,-y, ϕ) → L↔R replacement + backward

= 3 × 4 = 12 Shortest + collision-free path selection among path candidates
```

#### 2.4. 4 Path tracking by segment

The path generated by Hybrid A* is a mixture of forward and backward sections. This is divided into segments in the same direction and followed sequentially.

```
Path = [(x, y, yaw, dir), ...] → Segment separation (based on dir change point)

Following each segment (20Hz):
1 . Search for the nearest waypoint from your current location (index ±3 to 20 range)
2 . Set the target as the first point that is greater than the lookahead distance (0.15m)
3 . Target direction calculation:
- Forward: yaw_err = atan 2 (dy, dx) - cur_yaw
- Backwards: yaw_err = atan 2 (dy, dx) - cur_yaw - π
4 . Speed ​​command:
- Forward: 0.08 m/s (0.03 m/s when approaching slot)
- Reverse: -0.06 m/s (-0.03 m/s when approaching slot)
     - angular = clip(2.0 × yaw_err, -0.8, 0.8)
- Minimum turning radius limit: |angular| ≤ |speed| /0.42m
5 . Segment end point reached (within 0.06m) → Stop → Go to next segment

After all segments are exhausted → Enter the final alignment stage
```

#### 2.4. 5 Final Alignment

```
Entry condition: Distance to target < 0.12m ( ALIGN_DIST_ENTER ) + all segments exhausted

1 . Slow forward speed: 0.03 m/s ( ALIGN_SPEED )
2 . yaw correction: angular = clip( 1.5 × yaw_err , -0.3 , 0.3 )
3 . Deadzone: | yaw_err | < 0.02 → No correction

Mission completion decision:
- Mission completion position criterion: error < 0.04m (`REPARK_ARRIVE`)
- Mission completion yaw criterion: error < 0.052 rad (~3°, `REPARK_YAW_TOL`)
- Maintain both criteria continuously for 0.8 seconds → FINISH
```

### 2.5 LIMO 1 Algorithm summary diagram

```
LIMO1_EVADE              WAIT_FOR          LIMO1_SCAN                 LIMO1_REPARK
                         SELECTION
┌─────────────┐   ┌──────────────┐  ┌──────────────────────────┐  ┌──────────────────────────┐
│ P control straight line │ │ User terminal │ │ dynamic_tracker_node │ │ 1 . Establishment of Occupancy Grid │
│ Reverse │ │Wait for input │ │ │ │ 2 . Hybrid A* navigation │
│ │ │ │ │ LiDAR →odom projection │ │ (try to connect RS curve) │
│ Rear obstacle │→ │ a2 /a3/a4 │→ │ Baseline anchoring │→ │ 3 . Segment separation │
│ Detect (LiDAR │ │ Select │ │ Tracking box ROI Aggregate │ │ 4. Pure Pursuit by section │
│ + depth camera)│ │ │ │ Maintain 0.70s → Confirm slot │ │ 5 . Final alignment (P control) │
│ │ │ select_car │ │ │ │ 6 .  Maintain 0.8s → Complete │
│ Speed: -0.12 │ │ .sh service │ │ → /limo1/target_slot │ │ │
│ m/s │ │ Call │ │ Issue │ │ Speed: ± 0.08 (→ 0.03 ) m/s │
└─────────────┘   └──────────────┘  └──────────────────────────┘  └──────────────────────────┘
```

---

## 3. Core Exit-Vehicle Algorithm

The selected vehicle (one of A2 /A3/A4) performs a single mission: **Exit** from its slot.

### 3.1 Exit path generation (Sinusoidal Waypoint Generation)

**Purpose**: Create a smooth exit path starting from the slot (-Y direction), exiting south, and then east (+X). The starting X coordinate is determined dynamically depending on the selected vehicle.

```
Starting batch: ( start_x , 0.985 , yaw=-π /2 )
  start_x = -0.225 (A2), 0.225 (A3), 0.675 (A4)
Speed: 0.20 m/s (constant)
Simulation dt: 0.01s
Waypoint interval: 0.08m

Phase 1 ( 0 ~2.5s ): Straight ahead — exit slot
  ω = 0
→ Move 0.5m to the south (-Y)

Phase 2 ( 2.5 ~6.5s ): Sinusoidal curve — change direction
  tc = t - 2.5
  ω(tc) = (π² / 4T) × sin(π × tc / T),  T = 4.0s
→ Total rotation amount = π /2 (90°, south → east direction change)
→ Smooth S-shaped rotation with continuous acceleration

Phase 3 ( 6.5 ~10.5s ): Go straight — escape to the east
  ω = 0
→ Move 0.8m to the east (+X)

Each step:
  yaw += ω × dt
  x += speed × cos(yaw) × dt
  y += speed × sin(yaw) × dt
Record waypoints for every move over 0.08m
```

**Why use a sinusoidal profile**: A typical constant angular velocity rotation has discontinuities in angular acceleration at the start/end of the curve. The sinusoidal profile (of the form `sin(π·t /T )`) has ω= 0 at the start and end, ensuring continuous angular acceleration and smooth transitions that meet the physical limits of Ackermann steering.

### 3.2 Pure Pursuit Path Following

**Purpose**: Move the actual vehicle by following the created waypoint list.

```
Parameters:
  Wheelbase    = 0.24m
  Lookahead    = 0.25m
  Max Steering = 30° (0.5236 rad)
Arrival tolerance = 0.12m

Every control cycle (20Hz):
1 . Navigate to the nearest waypoint from your current location
2 . It moves forward starting from the closest point.
Set the first point with distance ≥ Lookahead ( 0.25m ) as the lookahead point.
3 . Convert the lookahead point to robot local coordinates:
     dx = target_x - cx
     dy = target_y - cy
     local_x =  cos(yaw) × dx + sin(yaw) × dy
     local_y = -sin(yaw) × dx + cos(yaw) × dy
4 . Curvature calculation:
     ld = sqrt(local_x² + local_y²)
     curvature = 2 × local_y / ld²
5 . Steering angle calculation:
     δ = atan(wheelbase × curvature)
     δ = clip(δ, -30°, +30°)
6 . Speed ​​command:
     linear  = 0.20 m/s
     angular = linear × tan(δ) / wheelbase
7 . End verdict:
- Distance to final waypoint < 0.12m → Completed exit
- Elapsed time > 30s → Timeout ends
```

### 3.3 Summary of exit flow

```
WAIT_FOR_SELECTION                LIMO2_EXIT_INIT                    LIMO2_EXITING
┌─────────────────────┐   ┌────────────────────────────┐     ┌────────────────────────────────┐
│ Select user vehicle │ │ Create sinusoidal waypoint │ │ Follow Pure Pursuit (20Hz) │
│ a Choose between 2 /a3/a4 1 │ │ │ │ │
│ │ │ Phase 1: Straight south (2.5s) │ │ 1. Nearest waypoint navigation │
│ → odom /cmd_vel │→ │ Phase 2 : S curve ( 4.0s ) │ ──→ │ 2 . lookahead point (≥ 0.25m ) │
│ Dynamic Binding │ │ Phase 3: Straight East (4.0s) │ │ 3. Curvature → Steering angle → cmd_vel │
│ → repark_goal │ │ │ │ 4 . Arrival or timeout → End │
│ Dynamic update │ │ ≈ Create 26 waypoints │ │ │
│ │ │ (0.08m interval) │ │ Speed: 0.20 m/s │
└─────────────────────┘   └────────────────────────────┘     └────────────────────────────────┘
```

---

## Appendix: Key Parameters

### Vehicle physics specifications (common)

| parameters | value | Unit |
|---------|-----|------|
| mass |  4.34 | kg |
| car body size |  0.31 × 0.19 × 0.12 | m (L×W×H) |
| wheel radius |  0.045 | m |
| Wheelbase |  0.24 | m |
| Minimum turning radius |  0.42 | m |
| maximum steering angle |  30° | deg |

### mission_manager Key constants

| constant | value | Use |
|-----|-----|------|
| `CTRL_HZ` |  20 Hz | Control loop cycle |
| `ESTOP_DIST_M` |  0.18 m | Emergency stop obstacle distance |
| `EVADE_SPEED` |  -0.12 m/s |  LIMO 1 Reverse speed |
| `EVADE_YAW_KP` |  1.2 | Reverse yaw P gain |
| `LIMO2_EXIT_SPEED` |  0.20 m/s | exit speed |
| `LIMO2_LOOKAHEAD` |  0.25 m | Departure Pure Pursuit lookahead |
| `LIMO2_ARRIVE_TOL` |  0.12 m | Departure/arrival tolerance |
| `REPARK_FWD_SPEED` |  0.08 m/s | Re-parking forward speed |
| `REPARK_BWD_SPEED` |  -0.06 m/s | Re-parking reverse speed |
| `REPARK_LOOKAHEAD` |  0.15 m | repark lookahead |
| `REPARK_ARRIVE` |  0.04 m | Re-parking arrival tolerance |
| `REPARK_YAW_TOL` |  0.052 rad (~3° ) | Repark yaw tolerance |
| `REPARK_HOLD_S` |  0.8 s | Arrival confirmation holding time |
| `SCAN_TIMEOUT_S` |  10.0 s | slot scan timeout |
| `ALIGN_SPEED` |  0.03 m/s | Final Sort Timelapse |
| `ALIGN_YAW_KP` |  1.5 | final alignment yaw P gain |

### dynamic_tracker_node main parameters

| parameters | value | Use |
|---------|-----|------|
| `side_scan_center_deg` |  90° | Side scan center (left) |
| `side_scan_half_deg` |  8° | side scan half width |
| `occupied_distance_m` |  0.45 m | Possession Judgment Distance |
| `clear_distance_m` |  0.65 m | Open Judgment Distance |
| `baseline_capture_distance_m` |  0.75 m | Baseline Capture Distance |
| `clear_hold_s` |  0.70 s | Time to remain open |
| `slot_center_offset_m` |  0.30 m | Slot Center Offset |
| `track_box_length_m` |  0.35 m | Length before and after tracking box |
| `track_box_width_m` |  0.30 m | Tracking box left/right width |
| `side_percentile` |  70% | Representative distance percentile |
| `min_clear_ratio` |  0.25 | Minimum open rate |

### Hybrid A* planner parameters

| parameters | value | Use |
|---------|-----|------|
| XY resolution |  0.03 m | grid indexing |
| Yaw resolution |  5° | grid indexing |
| step size |  0.06 m | Node extension distance |
| Steering angle | [ -30° , -15° , 0° , + 15° , + 30° ] | Expansion direction |
| RS connection attempt interval |  5 times | Every 5 expansion |
| Backward Cost Multiple |  2.5 × | Prepare for advance |
| Steering Cost Multiple |  1.1 × | Compared to going straight |
| Cost of redirection | + 0.3 | Forward↔Backward conversion |
| crash margin |  0.02 m | Clearance around bodywork |
| Maximum iterations |  80 , 000 times | Exploration upper limit |
