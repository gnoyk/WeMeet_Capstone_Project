# Remote Parking Workspace 1 - System Analysis Document

---

## Table of Contents

1. [Overall System Flow](#1-overall-system-flow)
2. [Core LIMO1 Logic](#2-core-limo1-logic)
3. [Core Logic for LIMO2 and Other Vehicles](#3-core-logic-for-limo2-and-other-vehicles)
4. [Summary](#4-summary)

---

## 1. Overall System Flow

### 1.1 System Overview

This system is a **remote parking simulation system** based on **ROS2 + Gazebo**. The double-parked user's vehicle (LIMO1) moves backwards to clear the road, and the other vehicle (one of A2/A3/A4) selected by the user autonomously pulls out, and then LIMO1 autonomously performs the entire process of automatically re-parking in the empty slot.

### 1.2 Package Configuration

| package | Role |
|--------|------|
| `remote_parking_manager` | Mission Master FSM Node + Hybrid A* Path Planner |
| `my_valet_parking` | LIMO Collection of individual control nodes (slot tracking, independent exit, keyboard teleop, etc.) |
| `remote_parking_world` | Gazebo world file, URDF/XACRO, launch file, utility script |
| `limo_car` | LIMO vehicle URDF model and Gazebo plugin definition |

### 1.3 Parking lot placement and robot placement

```
North wall (y=1.585)
   ┌───────────────────────────────────────────────────────────────┐
│ [A-1] [A0] [A1] [A2★] [★A3★] [A4★] [A5] [A6] │ ← Parking slot (y=0.985)
   │-1.575-1.125-0.675-0.225 0.225  0.675  1.125  1.575          │
   │                                                               │
│ [LIMO1] │ ← Double parking location
│ x=0.225, y=0.35, yaw=0 │ (lane in front of A3)
   │                                                               │
│─── By edge y=+0.535 ──── By (center y≈0) ──── y=-0.535 ──│
   │                                                               │
│ [B-1] [B0] [B1] [B2] [B3] [B4] [B5] [B6] │ ← Parking slot (y=-0.985)
   └───────────────────────────────────────────────────────────────┘
South wall (y=-1.585)
West wall (x=-1.90) exit → +X (open to east)
```

**Vehicle Deployment:**

| Category | Slots | status |
|------|------|------|
| static vehicle 5 | Row A: A-1(-1.575), A0(-1.125), A1(-0.675), A5(1.125), A6(1.575) | Pinned to Gazebo World |
| static vehicle 8 | Row B: B-1(-1.575), B0(-1.125), B1(-0.675), B2(-0.225), B3(0.225), B4(0.675), B5(1.125), B6(1.575) | Pinned to Gazebo World |
| **LIMO1** | by (0.225, 0.35) yaw=0 | Double parking — blocking the front of A3 |
| **LIMO2** | A3 (0.225, 0.985) yaw=-π/2 | Dynamic Spawn |
| **LIMO A2** | A2 (-0.225, 0.985) yaw=-π/2 | Dynamic Spawn |
| **LIMO A4** | A4 (0.675, 0.985) yaw=-π/2 | Dynamic Spawn |

★ A2, A3, A4: Vehicles available according to user selection

### 1.4 Sensor Overview

| sensor | Topic | Use |
|------|------|------|
| 2D LiDAR | `/limo1/scan`, `/limo2/scan` | Obstacle detection, slot occupancy determination, E-stop |
| Odometry | `/limo1/odom`, `/limo2/odom`, `/limo_a2/odom`, `/limo_a4/odom` | Position/posture tracking, path following |
| Rear Depth Camera | `/limo1/rear_camera/depth/image_raw` | Measuring rear obstacle distance when reversing |
| Front Depth Camera | `/limo1/front_camera/depth/image_raw` | Forward obstacle detection |
| Rear Depth Camera (LIMO2) | `/limo2/rear_camera/depth/image_raw` | Rear obstacle distance when leaving the car |

### 1.5 Full mission FSM (mission_manager.py)

The mission master FSM is managed by the `RemoteParkingManager` node with the 20Hz control loop.

```
IDLE
│ /start_remote_parking service call
  ▼
LIMO1_EVADE ──────────────────────────────────────────┐
│ LIMO1 moves backwards to release double parking │
│ (Detect rear obstacle or reach target (-0.9, 0.35)) │
  ▼                                                    │
WAIT_FOR_SELECTION                                     │
│ User selects vehicle to exit (a2/a3/a4) │
  │  bash scripts/select_car.sh a3                     │
│ → /select_exit_car/a3 service call │
  ▼                                                    │
LIMO2_EXIT_INIT                                        │
│ Create departure waypoint for selected vehicle │
│ Dynamically update repark_goal with exit slot coordinates │
  ▼                                                    │
LIMO2_EXITING                                          │
│ Follow exit path with Pure Pursuit │
│ (Ends upon arrival/timeout 30s) │
  ▼                                                    │
LIMO1_SCAN                                             │
│ Check if the slot is empty with LiDAR │
│ (dynamic_tracker_node is waiting for target_slot to be issued) │
│ (Timeout 10s → use fallback coordinates) │
  ▼                                                    │
LIMO1_REPARK                                           │
│ Hybrid A* route planning → tracking by segment │
│ → Final Alignment → Arrival decision │
  ▼                                                    │
FINISH ←──────────────────────────────────────── ABORT │
All vehicles stop route failure/timeout │
```

### 1.6 Inter-node communication structure

```
[mission_manager] ──pub──→ /limo1/cmd_vel ──→ LIMO1 operation
──pub──→ /limo2/cmd_vel ──→ LIMO2 operation
──pub──→ /limo_a2/cmd_vel ──→ LIMO A2 operation
──pub──→ /limo_a4/cmd_vel ──→ LIMO A4 operation
                  ──pub──→ /remote_parking/status
                  ←─sub── /limo1/odom, /limo2/odom, /limo_a2/odom, /limo_a4/odom
                  ←─sub── /limo1/scan, /limo2/scan
                  ←─sub── /limo1/rear_camera/depth/image_raw
                  ←─sub── /limo1/front_camera/depth/image_raw
                  ←─sub── /limo2/rear_camera/depth/image_raw
                  ←─sub── /limo1/target_slot (from dynamic_tracker_node)
←─sub── /select_exit_car (topic, secondary)
                  ←─sub── /remote_parking/reset
←─srv── /start_remote_parking (Mission starts)
←─srv── /reset_parking (Mission Reset)
←─srv── /select_exit_car/a2, /a3, /a4 (select vehicle)

[dynamic_tracker_node] ←─sub── /limo1/scan, /limo1/odom
                       ──pub──→ /limo1/target_slot
```

### 1.7 Launch sequence

1. **world_server.launch.py** (runs 1 times) - Load Gazebo physics engine + static world (including 13 vs. static vehicles)
2. **spawn_robots.launch.py** (run per test):
- 0s: Start robot_state_publisher × 4 + joint_state_publisher × 4
- 3s: Gazebo robot spawn × 4 (LIMO1, LIMO2, LIMO_A2, LIMO_A4)
- 5s: Start dynamic_tracker_node
- 6s: Start mission_manager (Master FSM)
- Start RViz visualization

---

## 2. Core LIMO1 Logic

LIMO1 is the **primary actor** that performs the entire process of canceling double parking and re-parking in an empty slot after the other vehicle leaves. mission_manager directly issues and controls `/limo1/cmd_vel`.

### 2.1 LIMO1_EVADE: Reverse to Clear the Aisle

#### Purpose
Reverse LIMO1, which is blocking the front of the slot, to secure an exit path for the other vehicle.

#### Sensors used
- **Odometry** (`/limo1/odom`): Track current location/yaw
- **Rear Depth Camera** (`/limo1/rear_camera/depth/image_raw`): Measures rear obstacle distance
- **2D LiDAR** (`/limo1/scan`): Rear 180 degree zone obstacle distance auxiliary measurement

#### How the algorithm works

```
1. When entering, save the current yaw as _evade_yaw (reference angle for going straight)

2. Every control cycle (50ms):
a. Check rear obstacle distance:
- If rear_obstacle_m < 0.18m (ESTOP_DIST_M), stop immediately
- When blocked continuously for more than 1.5 seconds → switches to WAIT_FOR_SELECTION
      
b. Check goal reached:
- Done if distance between current location and evade_goal(-0.9, 0.35) < 0.04m
→ Switch to WAIT_FOR_SELECTION
      
c. Reverse control:
- linear = -0.12 m/s (EVADE_SPEED, backward)
- yaw_err = _evade_yaw - Current yaw (to keep straight)
- |yaw_err| < 0.03 → Dead zone (no correction)
- angular = clip(-1.2 * yaw_err, -0.3, 0.3) (P control)
```

#### How to measure rear obstacle distance

By fusing the two sensors, the `rear_obstacle_m` value is determined:

1. **Based on LiDAR** (`_update_obstacle_distances`):
- Extract minimum distance of 180 degrees ± 30 degrees (posterior) area from LiDAR scan data
- Effective range filtering: `isfinite & (range > range_min) & (range < range_max)`

2. **Based on Depth Camera** (`_update_rear_obstacle`):
- Extract effective pixels from 32FC1 encoded Depth image (finite & > 0.01m)
- Use **5th percentile** (lowest 5%) as representative distance → Detect nearest obstacle
- Depth camera value overrides LiDAR value (more precise)

### 2.2 WAIT_FOR_SELECTION: Waiting for vehicle selection

#### Purpose
Wait for the user to decide which vehicle (A2/A3/A4) to exit.

#### How it works
- Select vehicle with `/select_exit_car/a2`, `/select_exit_car/a3`, `/select_exit_car/a4` service call
- Run `bash scripts/select_car.sh a3` in a separate terminal
- Inside the script: `ros2 service call /select_exit_car/a3 std_srvs/srv/Trigger '{}'`
- Secondary: String messages can also be received with topic `/select_exit_car` (one of a2, a3, a4)
- When a valid selection is made → switch to `LIMO2_EXIT_INIT`
- odom/cmd_vel publisher dynamic binding + `repark_goal` dynamic update according to selected vehicle

| Select | Odometry topics | cmd_vel Topic | Initial X coordinate | Reparking Goals |
|------|-------------|------------|-----------|-----------|
| a2 | `/limo_a2/odom`|`/limo_a2/cmd_vel` | -0.225 | (-0.225, 0.985) |
| a3 | `/limo2/odom`|`/limo2/cmd_vel` | 0.225 | (0.225, 0.985) |
| a4 | `/limo_a4/odom`|`/limo_a4/cmd_vel` | 0.675 | (0.675, 0.985) |

### 2.3 LIMO1_SCAN: Empty slot detection

#### Purpose
After the other vehicle leaves, check with LiDAR whether the slot is actually empty.

#### Sensors used
- **2D LiDAR** (`/limo1/scan`) → `dynamic_tracker_node` handles

#### How the algorithm works

`dynamic_tracker_node` runs as an independent node and performs the following steps:

```
1. Side Scan Projection:
- Extract +90 degree ± 8 degree (left side) region from LiDAR scan
- Masking of car body self-interference section (±130 to 170 degrees)
- Convert each ray to odom frame:
     p_odom = [x,y] + R(yaw) * [range*cos(θ), range*sin(θ)]

2. Establish baseline (record occupancy status):
- If there are more than 3 points within 0.75m on the side, it is judged as “vehicle presence”
- Save the center of the point cluster as baseline_center_odom
- Smoothly update baseline position with EMA (α=0.2) filter
- Record the number of points in the odom frame's ROI (0.35m x 0.30m)

3. Clearance verdict (check for empty slots):
- Project entire FOV ray into odom frame (not side window)
※ To prevent false detection due to ego movement among EVADE, all FOV are used.
- The number of points in ROI decreases to below 25% compared to the baseline.
- Lateral representative distance (70th percentile) >= 0.65m
- clear_ratio (ray rate above 0.65m) >= 25%
- If the above conditions persist for more than 0.7 seconds → target_slot is issued

4. Target Slot issued by:
- baseline_center + side_unit * 0.30m position calculated based on empty slot
- Issue 0.1 seconds interval to /limo1/target_slot with PoseStamped message
```

mission_manager is converted to `LIMO1_REPARK`when receiving`/limo1/target_slot`. When timing out in 10 seconds, use the `repark_goal` parameter coordinates as a fallback (dynamically determined depending on the selected slot).

### 2.4 LIMO1_REPARK: Hybrid A* automatic reparking

#### Purpose
LIMO1 autonomously drives to empty slots and parks precisely. **This is the most complicated step**.

#### Sensors used
- **Odometry** (`/limo1/odom`): Real-time location tracking /yaw

#### Algorithm details

##### (A) Hybrid A* route planning (`hybrid_astar_planner.py`)

```
Input: start(x,y,yaw), goal(x,y,yaw)
Output: [(x, y, yaw, direction), ...] list of path points

1. Building an Occupancy Grid:
- Range: x[-1.5, 1.2], y[-0.6, 1.7], resolution 0.02m
- Obstacle candidate slots: A0(-1.125), A1(-0.675), A2(-0.225), A3(0.225), A4(0.675)
→ Excluded slots (goal_x) are excluded from obstacles
- Register the north wall (y=1.585) and west wall (x=-1.90) as obstacles.
- Vehicle size: 0.40m x 0.26m (including margin)

2. Hybrid A* navigation:
- State space: (x, y, yaw) — continuous coordinates + discretized grid index
- xy resolution: 0.03m, yaw resolution: 5
- Step size: 0.06m
- Steering angle candidates: [-30, -15, 0, 15, 30]
- Direction: Search both forward (+1) and backward (-1)
   
3. Cost function:
- Basic cost: step size (0.06m)
- Reverse penalty: x2.5 (restricts backward preference)
- Steering penalty: x1.1 (prefers to go straight)
- Direction change penalty: +0.3 (suppresses frequent forward and backward changes)
   
4. Heuristic:
   - h = euclidean_dist + 0.5 * min_turn_radius * |yaw_diff|

5. Pruning the Reeds-Shepp curve:
- At every 5 node, attempt the RS curve from the current position to the goal.
- When a collision-free RS path is found → the path is immediately confirmed (search ends)
- 3 basic formula (LSL, LSR, LRL) × 4 conversion = 12 RS path type
   
6. Collision check:
- Check the entire vehicle footprint (0.31m x 0.19m + margin 0.02m) on the grid
- If any cell within the footprint is occupied, a collision is determined.
```

##### (B) Path tracking by segment

```
1. Split the path into segments in the same direction (forward/backward)
Example: [Forward section → Reverse section → Forward section]

2. Pure Pursuit Style Follows Within Each Segment:
- Search for the nearest waypoint from the current location (±3~20 index range)
- Target the first point that is greater than the lookahead distance (0.15m)
   
3. Steering Calculation:
   - target_angle = atan2(dy, dx)
- Forward: yaw_err = target_angle - Current yaw
- Reverse: yaw_err = target_angle - Current yaw - π
   - angular = clip(2.0 * yaw_err, -0.8, 0.8)
- Minimum turning radius limit: |angular| <= |speed| /0.42m
   
4. speed:
- Forward: 0.08 m/s, Backward: -0.06 m/s
- Slot proximity (within 0.20m): decelerate to 0.03 m/s

5. Segment end point reached (within 0.06m) → Stop → Switch to next segment
```

##### (C) Final Alignment

```
1. Slot proximity (within 0.12m) + Enter sorting phase when all segments are exhausted

2. Low speed alignment control:
- Speed: 0.03 m/s (0 if the remaining distance to the slot is 0.04m or less)
- yaw correction: angular = clip(1.5 * yaw_err, -0.3, 0.3)
- Dead zone: |yaw_err| < 0.02 → No correction

3. Mission completion decision:
- Mission completion criteria: position error < 0.04m AND yaw error < 3°
- When both criteria remain satisfied for more than 0.8 seconds, transition to FINISH
   
4. safety:
- 60 seconds timeout → ABORT
- Attempt to re-plan when route is exhausted
```

### 2.5 LIMO1 Logic overall flowchart

```
[Mission starts]
    │
    ▼
(1) LIMO1_EVADE: Reverse double parking disabled
Sensor: Odometry + Rear Depth Camera + LiDAR (Rear)
Algorithm: P control straight forward, rear obstacle E-stop
End condition: Target (-0.9, 0.35) reached OR blocked backwards for more than 1.5 seconds
    │
    ▼
(2) WAIT_FOR_SELECTION: Select exit vehicle
Enter: /select_exit_car/a2|a3|a4 service call
    │
    ▼
(3) LIMO2_EXIT_INIT + LIMO2_EXITING: Exiting the other vehicle (see Chapter 3 below)
    │
    ▼
(4) LIMO1_SCAN: Empty slot detected
Sensor: LiDAR (via dynamic_tracker_node)
Algorithm: Side Scan → Odom projection → ROI occupancy comparison → Clear decision
    │
    ▼
(5) LIMO1_REPARK: Automatic reparking
Sensor: Odometry
Algorithm: Hybrid A* path planning → tracking by segment → final alignment
    │
    ▼
[FINISH]
```

---

## 3. Core Logic for LIMO2 and Other Vehicles

### 3.1 Exit vehicle selection mechanism

When the user selects one of a2/a3/a4 with the `select_car.sh` script (service call method), mission_manager dynamically binds the odom of the vehicle and the cmd_vel publisher.

| Select | Odometry topics | cmd_vel Topic | Initial X coordinate |
|------|-------------|------------|-----------|
| a2 | `/limo_a2/odom`|`/limo_a2/cmd_vel` | -0.225 |
| a3 | `/limo2/odom`|`/limo2/cmd_vel` | 0.225 |
| a4 | `/limo_a4/odom`|`/limo_a4/cmd_vel` | 0.675 |

Depending on the selection result, `repark_goal` is also dynamically updated to the corresponding slot coordinates.

### 3.2 LIMO2_EXIT_INIT: Generate the Exit Path

#### Algorithm - Sinusoidal curve waypoint generation

The exit route is created in advance through 3-level simulation:

```
Initial state: (start_x, 0.985, yaw=-π/2) → facing south in slot
start_x = Initial X coordinate of selected vehicle (-0.225 / 0.225 / 0.675)

Phase 1: Escape the slot and go straight (2.5 seconds)
- moving straight south, omega = 0
- Exit the slot and enter the lane area

Phase 2: Sinusoidal curve rotation (4.0 seconds)
  - omega(t) = (π²/4T) * sin(πt/T)   (T = 4.0)
- Cumulative rotation: +π/2 (smooth direction change from south to east)
- Sinusoidal profile: angular velocity 0 at start/end, maximum at middle
- Generates smooth curves without sudden steering changes

Phase 3: Go straight east (4.0 seconds)
- moving straight east, omega = 0
- Leave toward the parking lot exit

Simulation parameters:
- dt = 0.01 seconds (precision discretization)
  - speed = 0.20 m/s
- Waypoint interval: Save only when 0.08m or more
```

#### Generated path type

```
slot
  │
│ Phase 1 (Go straight south)
  │
  │
   ╲
╲ Phase 2 (sinusoidal curve, -π/2 → 0)
     ╲
────────────── Phase 3 (Go straight east) ──→ Exit
```

### 3.3 LIMO2_EXITING: Pure Pursuit Path Following

#### Sensors used
- **Odometry** (odom of selected vehicle): Track current location/yaw

#### Algorithm details

```
Every control cycle (50ms):

1. Safety timeout check:
- Forced shutdown when 30 seconds are exceeded → LIMO1_SCAN conversion

2. Check final waypoint reached:
- Distance to last waypoint < 0.12m → Completed exit

3. Lookahead point navigation:
a. Search for the closest index to your current location across all waypoints
b. Forward from the index, select the first point that is at least 0.25m from the current location.
c. If not, use last waypoint

4. Pure Pursuit Steering Calculations:
a. Transform the target point to the local coordinate system:
      local_x = cos(yaw)*dx + sin(yaw)*dy
      local_y = -sin(yaw)*dx + cos(yaw)*dy
      
b. Curvature calculation:
      curvature = 2 * local_y / (ld²)
      
c. Steering angle calculation:
      delta = atan(wheelbase * curvature)
      delta = clip(delta, -30°, +30°)
      
d. Convert cmd_vel to:
      linear  = 0.20 m/s
      angular = linear * tan(delta) / wheelbase

5. Issue Twist with topic cmd_vel of selected vehicle
```

#### Pure Pursuit parameters

| parameters | value | Description |
|---------|-----|------|
| LIMO2_EXIT_SPEED | 0.20 m/s | forward speed |
| LIMO2_WHEELBASE | 0.24 m | Distance between axles |
| LIMO2_LOOKAHEAD | 0.25 m | Lookahead Street |
| LIMO2_MAX_STEER | 30 also | maximum steering angle |
| LIMO2_ARRIVE_TOL | 0.12 m | Arrival tolerance |
| LIMO2_WP_SPACING | 0.08 m | waypoint spacing |

### 3.4 Standalone LIMO2 Exit Node (`limo2_exit_handler.py`)

Separate from mission_manager, there is also an independent withdrawal node dedicated to LIMO2. This node operates as its own FSM and performs autonomous judgment based on LiDAR.

#### FSM status

```
WAIT (wait 10 seconds)
  │
  ▼
EXIT_SLOT (slot escape)
│ yaw alignment → advance → reach passage
  ▼
DECIDE_FREE (empty space direction search)
│ Scan 360 degrees LiDAR → Search for optimal direction with sliding window (±30 degrees)
│ Select maximum opening direction based on median distance for each direction
  ▼
TURN_TO_FREE (rotate towards empty space)
│ Ackermann forward arc rotation (linear required)
  ▼
DRIVE_FREE (Go straight to empty space 3m)
│ When detecting an obstacle within 0.6m → redirect to DECIDE_FREE
  ▼
FINISH
```

#### Core algorithm: empty space search (DECIDE_FREE)

```
1. Effective ray (angle_deg, range) collected from entire LiDAR scan

2. Traverse the candidate angles -180 degrees to +180 degrees in the interval 5 degrees:
Calculate the median ray distance within a ±30 degree window for each candidate.

3. Direction with the largest median = widest open direction
(In case of a tie, the direction closer to the front is preferred)

4. Convert the selected direction to world coordinate yaw and set free_target_yaw
```

#### Sensors used
- **Odometry** (`/limo2/odom`): Location/yaw
- **2D LiDAR** (`/limo2/scan`): E-stop obstacles ahead, search for direction in empty space

### 3.5 Secondary Node (`car_exit_controller.py`)

This is a node that directly moves the `moving_passenger_car`entity using Gazebo's`SetEntityState` service. Kinematically reproduces the exit without actual physical simulation.

```
BACK_OUT: Straight backwards from slot to aisle (y: -0.75 → 0.0)
TURN_TO_MINUS_X: Quarter-circle rotation with radius 0.45m
EXIT_STRAIGHT: Go straight in -X direction (x: → -4.20)
FINISH: Stop
```

### 3.6 Keyboard Teleop Node (limo2_arrow_teleop.py)

This is a teleop node that allows you to manually control LIMO2 with the direction keys.

- Topic: `/limo2/cmd_vel`
- Forward: 0.1 m/s, Backward: -0.1 m/s
- Assistance going straight when turning: 0.05 m/s (Due to Ackermann driving characteristics, steering is not possible if linear=0)
- Input: pynput keyboard listener (arrow keys)

---

## 4. Summary

### 4.1 System one-line summary

> **This is a ROS2-based remote parking autonomous driving system in which the double-parked LIMO1 vacates the road in reverse, the other vehicle selected by the user (A2Z

### 4.2 Summary of key algorithms

| steps | Algorithm | core sensor |
|------|---------|----------|
| LIMO1 Double parking unlocked | P Control Forward Forward + Depth Camera E-stop | Odometry, Posterior Depth, LiDAR |
| Select exit vehicle | Service call (a2/a3/a4) | - |
| Create an exit route for the other vehicle | Sinusoidal curve profile waypoint | - |
| Following the departure of the other vehicle | Pure Pursuit (Lookahead 0.25m) | Odometry |
| empty slot detection | LiDAR Side Scan → Odom Projection → ROI Occupancy Comparison | LiDAR, Odometry |
| Re-parking route planning | Hybrid A* + Reeds-Shepp curve (12 types) | Odometry (grid is static) |
| Re-parking route following | Pure Pursuit by Segment + Minimum Turn Radius Limit | Odometry |
| final sort | Low speed P control (position + yaw) + 0.8 seconds hold judgment | Odometry |

### 4.3 System Features

1. **Master-Slave Architecture**: mission_manager manages the entire mission FSM and directly controls cmd_vel of all vehicles
2. **Multi-vehicle support**: Dynamically change vehicle departure destination according to user selection among A2/A3/A4 (service call method)
3. **Sensor Fusion**: LiDAR + Depth camera fusion for reliable obstacle/slot detection
4. **Safety mechanisms**: E-stop (0.18m), timeout (30s/60s), ABORT states
5. **Ackermann Drive**: Control reflecting the Ackermann steering characteristics of the LIMO vehicle (minimum turning radius 0.42m)
6. **Hybrid A* + Reeds-Shepp**: Hybrid forward and backward route planning creates feasible routes in tight parking lot environments
7. **Expanded parking lot**: 16 slots (A-1~A6, B-1~B6), 13 static vehicles + 4 dynamic robots
