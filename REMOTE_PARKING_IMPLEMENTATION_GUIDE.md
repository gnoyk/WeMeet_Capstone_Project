# Remote Autonomous Parking System — Coding Agent Implementation Guide

> **Purpose**: This document was written so that the entire system can be implemented and debugged from scratch with one document.
> **Platform**: AgileX LIMO Pro · ROS 2 Humble · Gazebo Classic 11 · Ubuntu 22.04

---

## 1. Scenario & Goal

```
[Initial state]
LIMO 1 (your car) ── Double parking in front of LIMO 2 in the lane (blocking the lane)
LIMO 2 (Opponent car) ── A Normal parking in 3 slot (unable to exit due to blockage at LIMO 1)

[Press user button 1 times] ──→ ros 2 service call /start_remote_parking

① LIMO 1 reverse (reverse to slot B 3, open lane)
② LIMO 2 Autonomous exit (lane → exit, waypoint P control)
③ LIMO 1 detects an empty slot (A3) with LiDAR
④ LIMO 1 A3 reparking (3 step FSM)
⑤ Done — Both robots stop
```

**Key Constraints**
- Remote Button: Simulate with ROS2 service call
- Camera streaming: not required
- Slot detection: LiDAR (dynamic) + rear depth camera (auxiliary)
- Quick startup of the simulator: Gazebo 1 starts once and then resets only the robot

---

## 2. Prerequisites

### Required workspace (must already exist)

```
~/parking_ws/ ← parking_ws repo
  src/
    limo_car/          ← URDF + Gazebo xacro
limo_gazebosim / ← Existing world file
    my_valet_parking/  ← dynamic_tracker_node, limo_valet_parking_node

~/LIMO_platform/ ← LIMO_platform repo
  src/
wego_2d_nav / ← Nav 2 parameters
    robot_localization/← EKF
```

### Check limo_car package directory structure

```
limo_car/
├── urdf/
│ ├── limo_ackerman_base.xacro ← base_footprint commented out, root= base_link
│   ├── limo_anteil.xacro
│   └── limo_steering_hinge.xacro
├── gazebo/ ★ Gazebo plugin is located in this directory
│ ├── ackermann_with_sensor.xacro ← include target (sensor.xacro + ackermann.xacro)
│ ├── ackermann.xacro ← Drive Plug-in
│ └── sensor.xacro ← LiDAR /Camera /IMU plug-in
└── launch/
└── ackermann.launch.py ← gazebo/ackermann_with_sensor.xacro load
```

### URDF Key figures (based on ackermann.xacro)

| Item | value | Remarks |
|------|----|------|
| root link | `base_link` |  base_footprint commented out |
|  wheel_vertical_offset | `-0.10 m` |  base_link reference wheel center z |
| wheel_radius | `0.045 m` | |
| **spawn z** | **` 0.145 m`** | The value at which the bottom of the wheel touches z= 0 |
| wheelbase | `0.24 m` | |
| track | `0.168 m` | |
|  cmd_vel Topic | `cmd_vel` (Opponent) | spawn - namespacing as robot_namespace |
| odom topic | `odom` (relative) | |
| scan topic | `~ /out → scan` | namespace=/ or spawn override |

### Gazebo sensor topic (spawn - based on robot_namespace limo 1)

| sensor | Topic |
|------|------|
| LiDAR | `/limo1/scan` |
| Depth (RGB) | `/limo1/rgb/image_raw` |
| Depth | `/limo1/depth/image_raw` |
| Rear Camera (New) | `/limo1/rear_camera/depth/image_raw` |
|  IMU | `/limo/imu` (**hardcoded**, shared by two robots) |
| Odometry | `/limo1/odom` |
|  cmd_vel received | `/limo1/cmd_vel` |

---

## 3. Workspace Structure

```
~/remote_parking_ws/
├── README.md
└── src/
├── remote_parking_world / ← Gazebo World + Spawn launch
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── worlds/
    │   │   └── remote_parking_static.world
    │   ├── urdf/
    │   │   └── limo_with_rear_camera.urdf.xacro
    │   ├── launch/
│ │ ├── Run only world_server.launch.py ← 1 times
│ │ └── spawn_robots.launch.py ← Run each test
    │   └── scripts/
│ └── reset_robots.sh ← Fast reset (~0.5 seconds)
└── remote_parking_manager / ← Mission Master FSM
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/remote_parking_manager
        └── remote_parking_manager/
            ├── __init__.py
            └── mission_manager.py
```

---

## 4. Core Architecture

### 2 Stage Launch Separation (Simulator Speed ​​Optimization)

```
[Run 1 times] world_server.launch.py
gzserver + static world load (parking lot + static vehicles 7)
→ Maintain Gazebo process, no restart

[Every test] spawn_robots.launch.py
LIMO 1 · 2 spawn + start all ROS2 nodes
Timing: RSP ( 0s ) → Spawn ( 3s ) → Control node ( 5s ) → FSM ( 6s )

[reset] bash reset_robots.sh teleport
Move only the position to set_entity_state (~0.5 seconds)
No Gazebo restart
```

### FSM state transition

```
IDLE
│ Receive /start_remote_parking service
  ▼
LIMO1_EVADE ── LIMO 1 backward ( 0.225 , 0 ) → ( 0.225 , -0.985 ) [B 3 slot]
│ Retraction complete (odometry standard reached)
  ▼
LIMO2_EXIT_INIT ── LIMO 2 Waypoint reset
│ Instant conversion
  ▼
LIMO2_EXITING ── LIMO 2 exit P control (4 waypoints)
│ Complete all waypoints
  ▼
LIMO1_SCAN ── Confirm that A3 slot is empty with LiDAR
│ Receive target_slot OR timeout (10s) → Use fallback
  ▼
LIMO1_REPARK       ── APPROACH → ARC_TURN → PRECISION
│ Precision parking completed (yaw < 1° , x_err < 8mm , 0.6s maintained)
  ▼
FINISH ── Both robots stop
```

### Division of package roles

| Features | Source | file |
|------|------|------|
|  URDF + Gazebo plugin | `parking_ws /limo_car` | `gazebo/ackermann_with_sensor.xacro` |
| Add rear camera | **New** | `remote_parking_world/urdf/limo_with_rear_camera.urdf.xacro` |
| Gazebo World | **New** | `remote_parking_world /worlds/remote_parking_static.world` |
| LiDAR slot detection | `parking_ws /my_valet_parking` | `dynamic_tracker_node.py` |
| Mission FSM | **New** | `remote_parking_manager/mission_manager.py` |

---

## 5. Parking lot world design (scale 1 /5.6 by law)

```
Real slot: 2.5m × 5.0m → Gazebo: 0.45m × 0.90m
Actual car: 6.0m → Gazebo: 1.07m
LIMO Size: → 0.34m (W) × 0.78m (L)

[World Top-view] +Y (North)

  North wall ────────────────────── Y = +1.585
  [A1:red][A2:blue][A3:LIMO2][A4:silver]  Y = +0.985
  Lane edge ───────────────────────  Y = +0.535
              LANE (center Y=0)
← LIMO 1 double parking →
  Lane edge ───────────────────────  Y = -0.535
  [B1:yel][B2:grn][B3:empty][B4:ora]      Y = -0.985
  South wall ────────────────────── Y = -1.585

West wall X= -0.95 exit → +X

Slot
A3, B 3: No static model (LIMO 1 · 2 spawn location)
```

### Initial spawn pose

| robot | x | y | z | yaw |
|------|---|---|---|-----|
| LIMO 1 | 0.225 | 0.0 | **0.145** | +π/2 |
| LIMO 2 | 0.225 | 0.985 | **0.145** | -π/2 |

> z= 0.145 Ground: base_link root, wheel_vertical_offset = -0.10 , wheel_radius = 0.045 → Bottom of wheel = spawn_z - 0.145 = 0

---

## 6. Implementation Files

### 6-1. `remote_parking_world/package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>remote_parking_world</name>
  <version>0.1.0</version>
  <description>Remote parking Gazebo world and robot spawn launch</description>
  <maintainer email="you@example.com">Jihyun</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>my_valet_parking</exec_depend>
  <exec_depend>remote_parking_manager</exec_depend>
  <ament_cmake_depend>ament_cmake</ament_cmake_depend>
</package>
```

### 6-2. `remote_parking_world/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(remote_parking_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds models launch scripts urdf config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/reset_robots.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
```

### 6-3. `remote_parking_world/worlds/remote_parking_static.world`

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="remote_parking">

<!-- Physics optimization: 1000Hz → 500Hz , quick solver, shadow OFF -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

<!-- ROS2 plugin -->
    <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so"/>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <update_rate>10.0</update_rate>
    </plugin>

<!-- Light (Shadow OFF ) -->
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

<!-- Ground -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
        </visual>
      </link>
    </model>

<!-- Asphalt -->
    <model name="parking_asphalt">
      <static>true</static>
      <pose>0.05 0 0.001 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>2.0 3.17 0.002</size></box></geometry></collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>2.0 3.17 0.002</size></box></geometry>
          <material><ambient>0.22 0.22 0.22 1</ambient><diffuse>0.22 0.22 0.22 1</diffuse></material>
        </visual>
      </link>
    </model>

<!-- North Wall -->
    <model name="wall_north">
      <static>true</static><pose>0.05 1.585 0.10 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>2.0 0.10 0.20</size></box></geometry></collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>
          <geometry><box><size>2.0 0.10 0.20</size></box></geometry>
          <material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>

<!-- South Wall -->
    <model name="wall_south">
      <static>true</static><pose>0.05 -1.585 0.10 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>2.0 0.10 0.20</size></box></geometry></collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>
          <geometry><box><size>2.0 0.10 0.20</size></box></geometry>
          <material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>

<!-- Western Wall -->
    <model name="wall_west">
      <static>true</static><pose>-0.95 0 0.10 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.10 3.17 0.20</size></box></geometry></collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>
          <geometry><box><size>0.10 3.17 0.20</size></box></geometry>
          <material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>
        </visual>
      </link>
    </model>

<!-- Lane border north Y=+ 0.535 -->
    <model name="line_lane_north"><static>true</static><pose>0.05 0.535 0.003 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>1.80 0.02 0.002</size></box></geometry></collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>
          <geometry><box><size>1.80 0.02 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>
      </link>
    </model>

<!-- South of lane boundary Y= -0.535 -->
    <model name="line_lane_south"><static>true</static><pose>0.05 -0.535 0.003 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>1.80 0.02 0.002</size></box></geometry></collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>
          <geometry><box><size>1.80 0.02 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>
      </link>
    </model>

<!-- Row A slot partition line 5 (X = -0.90 ~ + 0.90) -->
    <model name="line_a0"><static>true</static><pose>-0.90 0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_a1"><static>true</static><pose>-0.45 0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_a2"><static>true</static><pose>0.0 0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_a3"><static>true</static><pose>0.45 0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_a4"><static>true</static><pose>0.90 0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>

<!-- Row B slot partition line 5 -->
    <model name="line_b0"><static>true</static><pose>-0.90 -0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_b1"><static>true</static><pose>-0.45 -0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_b2"><static>true</static><pose>0.0 -0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_b3"><static>true</static><pose>0.45 -0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <model name="line_b4"><static>true</static><pose>0.90 -0.985 0.003 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.02 0.90 0.002</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.02 0.90 0.002</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>

<!-- static vehicle (static=true, box geometry, A3 /B3 left empty)
         크기: 0.78m(길이) × 0.34m(폭) × 0.13m(높이), center z=0.065
         Row A yaw=-π/2 (앞면 차로 방향), Row B yaw=+π/2 -->

    <!-- A1 red -->
    <model name="static_car_a1"><static>true</static><pose>-0.675 0.985 0.065 0 0 -1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.78 0.34 0.13</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.78 0.34 0.13</size></box></geometry>
          <material><ambient>0.85 0.1 0.1 1</ambient><diffuse>0.85 0.1 0.1 1</diffuse></material></visual>
      </link>
    </model>
    <!-- A2 blue -->
    <model name="static_car_a2"><static>true</static><pose>-0.225 0.985 0.065 0 0 -1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.78 0.34 0.13</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.78 0.34 0.13</size></box></geometry>
          <material><ambient>0.1 0.1 0.85 1</ambient><diffuse>0.1 0.1 0.85 1</diffuse></material></visual>
      </link>
    </model>
<!-- A3 : LIMO 2 dynamic spawn -->
    <!-- A4 silver -->
    <model name="static_car_a4"><static>true</static><pose>0.675 0.985 0.065 0 0 -1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.78 0.34 0.13</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.78 0.34 0.13</size></box></geometry>
          <material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual>
      </link>
    </model>
    <!-- B1 yellow -->
    <model name="static_car_b1"><static>true</static><pose>-0.675 -0.985 0.065 0 0 1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.78 0.34 0.13</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.78 0.34 0.13</size></box></geometry>
          <material><ambient>0.9 0.8 0.0 1</ambient><diffuse>0.9 0.8 0.0 1</diffuse></material></visual>
      </link>
    </model>
    <!-- B2 green -->
    <model name="static_car_b2"><static>true</static><pose>-0.225 -0.985 0.065 0 0 1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.78 0.34 0.13</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.78 0.34 0.13</size></box></geometry>
          <material><ambient>0.1 0.75 0.1 1</ambient><diffuse>0.1 0.75 0.1 1</diffuse></material></visual>
      </link>
    </model>
<!-- B 3 : LIMO 1 Temporary reverse slot — left empty -->
    <!-- B4 orange -->
    <model name="static_car_b4"><static>true</static><pose>0.675 -0.985 0.065 0 0 1.5708</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.78 0.34 0.13</size></box></geometry></collision>
        <visual name="v"><cast_shadows>false</cast_shadows><geometry><box><size>0.78 0.34 0.13</size></box></geometry>
          <material><ambient>0.95 0.5 0.0 1</ambient><diffuse>0.95 0.5 0.0 1</diffuse></material></visual>
      </link>
    </model>

  </world>
</sdf>
```

### 6-4. `remote_parking_world/urdf/limo_with_rear_camera.urdf.xacro`

```xml
<?xml version="1.0"?>
<robot name="limo" xmlns:xacro="http://www.ros.org/wiki/xacro">

<!-- robot_namespace : Set the same value as spawn_entity - robot_namespace
       후방 카메라 플러그인 식별용; 실제 토픽 namespacing은 spawn이 처리 -->
  <xacro:arg name="robot_namespace" default="/"/>
  <xacro:property name="robot_ns" value="$(arg robot_namespace)"/>

<!-- Original LIMO Ackermann URDF (with Gazebo plugin)
       경로: limo_car/gazebo/ (urdf/ 아님!)
       포함 내용:
         - libgazebo_ros_ackermann_drive (cmd_vel, odom)
         - libgazebo_ros_ray_sensor (scan)
         - libgazebo_ros_camera (rgb, depth)
         - libgazebo_ros_imu_sensor (imu)
       주의: ackermann_with_sensor.xacro 는 외부 인자를 받지 않음 -->
  <xacro:include filename="$(find limo_car)/gazebo/ackermann_with_sensor.xacro"/>

<!-- Rear camera link (rear -0.30m based on base_link, height + 0.10m) -->
  <link name="rear_camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.025 0.090 0.025"/></geometry>
      <material name="cam_mat"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.025 0.090 0.025"/></geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <joint name="rear_camera_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="rear_camera_link"/>
    <origin xyz="-0.30 0 0.10" rpy="0 0 3.14159"/>
  </joint>

<!-- Optical frame ( ROS convention: x-right, y-down, z-forward) -->
  <link name="rear_camera_optical_frame"/>
  <joint name="rear_camera_optical_joint" type="fixed">
    <parent link="rear_camera_link"/>
    <child  link="rear_camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>

<!-- Gazebo Rear Depth Camera Plugin
       namespace 미지정 → spawn_entity -robot_namespace 가 자동 적용
       limo1: /limo1/rear_camera/depth/image_raw
       limo2: /limo2/rear_camera/depth/image_raw -->
  <gazebo reference="rear_camera_link">
    <sensor name="rear_depth_camera" type="depth">
      <always_on>true</always_on>
      <update_rate>15</update_rate>
      <visualize>false</visualize>
      <camera>
        <horizontal_fov>1.0472</horizontal_fov>
        <image><width>320</width><height>240</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>3.0</far></clip>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.005</stddev></noise>
      </camera>
      <plugin name="rear_depth_camera_plugin" filename="libgazebo_ros_camera.so">
        <ros>
          <remapping>image_raw:=rear_camera/depth/image_raw</remapping>
          <remapping>camera_info:=rear_camera/depth/camera_info</remapping>
          <remapping>points:=rear_camera/points</remapping>
        </ros>
        <camera_name>rear_camera</camera_name>
        <frame_name>rear_camera_optical_frame</frame_name>
        <min_depth>0.05</min_depth>
        <max_depth>3.0</max_depth>
        <hack_baseline>0.0</hack_baseline>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### 6-5. `remote_parking_world/launch/world_server.launch.py`

```python
#!/usr/bin/env python3
"""
world_server.launch.py — 1회만 실행
gzserver + 정적 월드 로드 (LIMO 없음)
gui:=true 로 gzclient 포함 가능
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('remote_parking_world')
    world_file = PathJoinSubstitution([
        pkg_share, 'worlds', 'remote_parking_static.world'
    ])
    gazebo_model_path = PathJoinSubstitution([pkg_share, 'models'])

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[gazebo_model_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
        ),
        DeclareLaunchArgument('gui', default_value='false',
                              description='gzclient 도 시작할지 여부'),

        ExecuteProcess(
            cmd=[
                'gzserver', '--verbose',
                '-s', 'libgazebo_ros_factory.so',
                '-s', 'libgazebo_ros_state.so',
                world_file,
            ],
            output='screen',
            additional_env={'GAZEBO_MODEL_PATH': gazebo_model_path},
        ),

        ExecuteProcess(
            cmd=['gzclient', '--verbose'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
    ])
```

### 6-6. `remote_parking_world/launch/spawn_robots.launch.py`

```python
#!/usr/bin/env python3
"""
spawn_robots.launch.py — 테스트마다 실행
전제: world_server.launch.py 가 이미 실행 중

타이밍:
  0s  → robot_state_publisher × 2 시작
  3s  → LIMO 1·2 Gazebo 스폰
  5s  → dynamic_tracker_node 시작
  6s  → remote_parking_manager FSM 시작
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, TimerAction
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node, PushRosNamespace

# ── Initial spawn pose ──────────────────────────────────────────────────────
# z= 0.145 : base_link root, wheel_offset = -0.10 , wheel_r = 0.045 → Bottom of wheel=z= 0
LIMO1_INIT = dict(x='0.225', y='0.0',   z='0.145', R='0', P='0', Y='1.5708')
LIMO2_INIT = dict(x='0.225', y='0.985', z='0.145', R='0', P='0', Y='-1.5708')


def generate_launch_description():
    pkg_world = get_package_share_directory('remote_parking_world')
    urdf_xacro = os.path.join(pkg_world, 'urdf', 'limo_with_rear_camera.urdf.xacro')

# xacro runtime processing (Command substitute)
    limo1_desc = Command([FindExecutable(name='xacro'), ' ', urdf_xacro, ' robot_namespace:=limo1'])
    limo2_desc = Command([FindExecutable(name='xacro'), ' ', urdf_xacro, ' robot_namespace:=limo2'])

# robot_state_publisherGroup
    limo1_group = GroupAction([
        PushRosNamespace('limo1'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': limo1_desc, 'use_sim_time': True}],
             remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': True}]),
    ])
    limo2_group = GroupAction([
        PushRosNamespace('limo2'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': limo2_desc, 'use_sim_time': True}],
             remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': True}]),
    ])

# Spawn Gazebo (after 3 seconds — wait for RSP to be ready)
# - robot_namespace Required: Gazebo plugin topic namespacing
    spawn_limo1 = TimerAction(period=3.0, actions=[ExecuteProcess(cmd=[
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', 'limo1',
        '-topic',  '/limo1/robot_description',
        '-robot_namespace', 'limo1',
        '-x', LIMO1_INIT['x'], '-y', LIMO1_INIT['y'], '-z', LIMO1_INIT['z'],
        '-R', LIMO1_INIT['R'], '-P', LIMO1_INIT['P'], '-Y', LIMO1_INIT['Y'],
        '-timeout', '120.0',
    ], output='screen')])

    spawn_limo2 = TimerAction(period=3.0, actions=[ExecuteProcess(cmd=[
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', 'limo2',
        '-topic',  '/limo2/robot_description',
        '-robot_namespace', 'limo2',
        '-x', LIMO2_INIT['x'], '-y', LIMO2_INIT['y'], '-z', LIMO2_INIT['z'],
        '-R', LIMO2_INIT['R'], '-P', LIMO2_INIT['P'], '-Y', LIMO2_INIT['Y'],
        '-timeout', '120.0',
    ], output='screen')])

# dynamic_tracker_node (after 5 seconds — wait for spawn to complete)
    dynamic_tracker = TimerAction(period=5.0, actions=[Node(
        package='my_valet_parking', executable='dynamic_tracker_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_topic':   '/limo1/scan',
            'odom_topic':   '/limo1/odom',
            'target_topic': '/limo1/target_slot',
            'odom_frame':   'odom',
            'occupied_distance_m':            0.45,
            'clear_distance_m':               0.65,
            'baseline_capture_distance_m':    0.75,
            'clear_hold_s':                   0.70,
            'slot_center_offset_m':           0.30,
            'allow_fallback_without_baseline': True,  # 초기 baseline 없이 탐지
            'fallback_target_x':              0.225,
            'fallback_target_y':              0.985,
            'target_yaw_rad':                -1.5708,
            'side_scan_center_deg':           90.0,
            'side_scan_half_deg':             8.0,
            'min_clear_ratio':                0.25,
            'track_box_length_m':             0.35,
            'track_box_width_m':              0.30,
            'publish_period_s':               0.10,
        }],
    )])

# mission_manager FSM (after 6 seconds)
    mission_manager = TimerAction(period=6.0, actions=[Node(
        package='remote_parking_manager', executable='mission_manager',
        output='screen',
        parameters=[{
            'use_sim_time':    True,
            'limo1_evade_x':   0.225,
            'limo1_evade_y':  -0.985,
            'limo2_exit_x':    3.0,
            'limo2_exit_y':    0.0,
            'repark_x':        0.225,
            'repark_y':        0.985,
            'repark_yaw':     -1.5708,
            'max_linear_speed': 0.05,
            'max_angular_speed': 0.8,
        }],
    )])

    return LaunchDescription([
        limo1_group, limo2_group,
        spawn_limo1, spawn_limo2,
        dynamic_tracker,
        mission_manager,
    ])
```

### 6-7. `remote_parking_world/scripts/reset_robots.sh`

```bash
#!/usr/bin/env bash
# Quick reset: Reset only the robot position without restarting Gazebo
# Usage: bash reset_robots.sh [teleport|respawn]
# teleport (default): move to set_entity_state immediately (~0.5 seconds)
# respawn: delete + respawn (~5 seconds, full reset)

set -e
MODE=${1:-"teleport"}

# z= 0.145 : Height at the bottom of the wheel based on ackermann.xacro z= 0
LIMO1_POSE="{position: {x: 0.225, y: 0.0,   z: 0.145}, orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}}"
LIMO2_POSE="{position: {x: 0.225, y: 0.985, z: 0.145}, orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}}"
ZERO_TWIST="{linear: {x:0,y:0,z:0}, angular: {x:0,y:0,z:0}}"
LIMO1_INIT="-x 0.225 -y 0.0   -z 0.145 -R 0 -P 0 -Y  1.5708"
LIMO2_INIT="-x 0.225 -y 0.985 -z 0.145 -R 0 -P 0 -Y -1.5708"

echo "━━━ Remote Parking Reset (MODE: $MODE) ━━━"

# FSM reset signal
ros2 topic pub --once /remote_parking/reset std_msgs/Bool "data: true" 2>/dev/null || true

# stop command
ros2 topic pub --once /limo1/cmd_vel geometry_msgs/Twist \
  "{linear:{x:0,y:0,z:0},angular:{x:0,y:0,z:0}}" 2>/dev/null || true
ros2 topic pub --once /limo2/cmd_vel geometry_msgs/Twist \
  "{linear:{x:0,y:0,z:0},angular:{x:0,y:0,z:0}}" 2>/dev/null || true
sleep 0.5

if [ "$MODE" = "teleport" ]; then
    ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
      "{state: {name: 'limo1', pose: $LIMO1_POSE, twist: $ZERO_TWIST, reference_frame: 'world'}}"
    ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState \
      "{state: {name: 'limo2', pose: $LIMO2_POSE, twist: $ZERO_TWIST, reference_frame: 'world'}}"
    echo "✅ 텔레포트 리셋 완료"

elif [ "$MODE" = "respawn" ]; then
    ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'limo1'}" 2>/dev/null || true
    ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'limo2'}" 2>/dev/null || true
    sleep 2
    ros2 run gazebo_ros spawn_entity.py -entity limo1 -topic /limo1/robot_description \
      -robot_namespace limo1 $LIMO1_INIT &
    ros2 run gazebo_ros spawn_entity.py -entity limo2 -topic /limo2/robot_description \
      -robot_namespace limo2 $LIMO2_INIT &
    wait
    echo "✅ 재스폰 리셋 완료"
fi

echo "→ 다음 테스트: ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'"
```

### 6-8. `remote_parking_manager/remote_parking_manager/mission_manager.py`

```python
#!/usr/bin/env python3
"""
mission_manager.py — 원격 주차 미션 마스터 FSM

FSM: IDLE → LIMO1_EVADE → LIMO2_EXIT_INIT → LIMO2_EXITING
       → LIMO1_SCAN → LIMO1_REPARK (APPROACH→ARC_TURN→PRECISION) → FINISH

서비스:
  /start_remote_parking  (std_srvs/Trigger)  미션 시작
  /reset_parking         (std_srvs/Trigger)  리셋

구독:
  /limo1/odom, /limo1/scan
  /limo1/rear_camera/depth/image_raw (후방 장애물)
  /limo2/odom
  /limo1/target_slot (dynamic_tracker_node 발행)
  /remote_parking/reset (Bool)

발행:
  /limo1/cmd_vel, /limo2/cmd_vel
  /remote_parking/status (String)
"""

import math
from enum import Enum, auto
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class MissionState(Enum):
    IDLE            = auto()
    LIMO1_EVADE     = auto()
    LIMO2_EXIT_INIT = auto()
    LIMO2_EXITING   = auto()
    LIMO1_SCAN      = auto()
    LIMO1_REPARK    = auto()
    FINISH          = auto()
    ABORT           = auto()


class ParkStage(Enum):
    APPROACH  = auto()
    ARC_TURN  = auto()
    PRECISION = auto()


def yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))


def pos_from_odom(odom: Odometry) -> Tuple[float, float]:
    p = odom.pose.pose.position
    return p.x, p.y


def norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def dist2d(x1, y1, x2, y2) -> float:
    return math.hypot(x2-x1, y2-y1)


class RemoteParkingManager(Node):

    CTRL_HZ          = 20.0
    ESTOP_DIST_M     = 0.18
    EVADE_SPEED      = -0.04
    EVADE_YAW_KP     = 1.8
    LIMO2_LINEAR     = 0.05
    LIMO2_ANGULAR_KP = 2.0
    MAX_LINEAR       = 0.05
    ARC_ANGULAR_MIN  = 0.35
    ARC_ANGULAR_MAX  = 1.2
    ARC_KP           = 1.5
    PREC_KP_LAT      = 1.4
    PREC_KP_YAW      = 2.2
    PREC_FINISH_DIST = 0.005
    PREC_FINISH_YAW  = 0.018
    PREC_HOLD_S      = 0.6
    SCAN_TIMEOUT_S   = 10.0

    def __init__(self):
        super().__init__('remote_parking_manager')

        self.declare_parameter('limo1_evade_x',    0.225)
        self.declare_parameter('limo1_evade_y',   -0.985)
        self.declare_parameter('limo2_exit_x',     3.0)
        self.declare_parameter('limo2_exit_y',     0.0)
        self.declare_parameter('repark_x',         0.225)
        self.declare_parameter('repark_y',         0.985)
        self.declare_parameter('repark_yaw',      -1.5708)
        self.declare_parameter('max_linear_speed', 0.05)

        self.evade_goal  = (self.get_parameter('limo1_evade_x').value,
                            self.get_parameter('limo1_evade_y').value)
        self.limo2_exit  = (self.get_parameter('limo2_exit_x').value,
                            self.get_parameter('limo2_exit_y').value)
        self.repark_goal = (self.get_parameter('repark_x').value,
                            self.get_parameter('repark_y').value)
        self.repark_yaw_goal = self.get_parameter('repark_yaw').value

        self.state      = MissionState.IDLE
        self.park_stage = ParkStage.APPROACH

        self.limo1_odom: Optional[Odometry] = None
        self.limo1_scan: Optional[LaserScan] = None
        self.limo2_odom: Optional[Odometry] = None
        self.target_slot: Optional[PoseStamped] = None

        self.front_obstacle_m = 999.0
        self.rear_obstacle_m  = 999.0
        self.limo2_waypoints  = []
        self.limo2_wp_idx     = 0
        self.scan_start_time: Optional[float] = None
        self.prec_ok_since:   Optional[float] = None
        self.approach_target: Optional[Tuple[float, float]] = None

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        self.create_subscription(Odometry,    '/limo1/odom',  self._l1_odom_cb,  sensor_qos)
        self.create_subscription(LaserScan,   '/limo1/scan',  self._l1_scan_cb,  sensor_qos)
        self.create_subscription(Image, '/limo1/rear_camera/depth/image_raw',
                                 self._rear_depth_cb, sensor_qos)
        self.create_subscription(Odometry,    '/limo2/odom',  self._l2_odom_cb,  sensor_qos)
        self.create_subscription(PoseStamped, '/limo1/target_slot', self._slot_cb, 1)
        self.create_subscription(Bool, '/remote_parking/reset', self._reset_cb, 1)

        self.limo1_cmd  = self.create_publisher(Twist,  '/limo1/cmd_vel', 1)
        self.limo2_cmd  = self.create_publisher(Twist,  '/limo2/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/remote_parking/status', 1)

        self.create_service(Trigger, '/start_remote_parking', self._start_cb)
        self.create_service(Trigger, '/reset_parking',        self._reset_srv_cb)
        self.create_timer(1.0 / self.CTRL_HZ, self._control_loop)

        self._build_limo2_waypoints()
        self.get_logger().info('🚗 RemoteParkingManager 준비 — /start_remote_parking 대기')

# ── Waypoint ────────────────────────────────────────────────────────
    def _build_limo2_waypoints(self):
        self.limo2_waypoints = [
            (0.225,  0.2,  -math.pi/2),   # 슬롯 → 차로 진입
            (0.225,  0.0,  -math.pi/2),   # 차로 중앙
            (0.50,   0.0,   0.0),          # 우회전 준비
            (self.limo2_exit[0], self.limo2_exit[1], 0.0),  # 출구
        ]
        self.limo2_wp_idx = 0

# ── callback ────────────────────────────────────────────────────────────
    def _l1_odom_cb(self, m): self.limo1_odom = m
    def _l2_odom_cb(self, m): self.limo2_odom = m
    def _slot_cb(self, m):    self.target_slot = m

    def _l1_scan_cb(self, msg: LaserScan):
        self.limo1_scan = msg
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges) & (ranges > msg.range_min)
                          & (ranges < msg.range_max), ranges, np.inf)
        n = len(ranges)
        if n == 0: return
# ±30° (ESTOP_FRONT_START /END_DEG =±30 based on limo_valet_parking_node.py)
        span = int(30 / math.degrees(msg.angle_increment))
        fi = np.concatenate([np.arange(0, span), np.arange(n-span, n)])
        self.front_obstacle_m = float(np.min(ranges[fi]))
        c = n // 2
        self.rear_obstacle_m  = float(np.min(ranges[max(0,c-span):min(n,c+span)]))

    def _rear_depth_cb(self, msg: Image):
        try:
            if msg.encoding == '32FC1':
                d = np.frombuffer(msg.data, dtype=np.float32).copy()
                d = d[np.isfinite(d) & (d > 0.01)]
                if len(d) > 0:
                    self.rear_obstacle_m = min(self.rear_obstacle_m, float(np.percentile(d, 5)))
        except Exception: pass

    def _reset_cb(self, msg: Bool):
        if msg.data: self._do_reset()

    def _start_cb(self, req, res):
        if self.state != MissionState.IDLE:
            res.success = False; res.message = f'진행 중: {self.state.name}'
            return res
        if self.limo1_odom is None or self.limo2_odom is None:
            res.success = False; res.message = '오도메트리 미수신'
            return res
        self._transition_to(MissionState.LIMO1_EVADE)
        res.success = True; res.message = '미션 시작'
        return res

    def _reset_srv_cb(self, req, res):
        self._do_reset()
        res.success = True; res.message = 'IDLE 리셋'
        return res

    def _do_reset(self):
        self._pub(self.limo1_cmd, 0, 0); self._pub(self.limo2_cmd, 0, 0)
        self.target_slot = None; self.scan_start_time = None
        self.prec_ok_since = None; self._build_limo2_waypoints()
        self._transition_to(MissionState.IDLE)

# ── Control loop ────────────────────────────────────────────────────────
    def _control_loop(self):
        s = String(); s.data = self.state.name; self.status_pub.publish(s)

        if   self.state == MissionState.IDLE:            return
        elif self.state == MissionState.LIMO1_EVADE:     self._evade()
        elif self.state == MissionState.LIMO2_EXIT_INIT: self._exit_init()
        elif self.state == MissionState.LIMO2_EXITING:   self._exiting()
        elif self.state == MissionState.LIMO1_SCAN:      self._scan()
        elif self.state == MissionState.LIMO1_REPARK:    self._repark()
        elif self.state in (MissionState.FINISH, MissionState.ABORT):
            self._pub(self.limo1_cmd, 0, 0); self._pub(self.limo2_cmd, 0, 0)

    # ── EVADE ─────────────────────────────────────────────────────────
    def _evade(self):
        if self.limo1_odom is None: return
        if self.rear_obstacle_m < self.ESTOP_DIST_M:
            self._pub(self.limo1_cmd, 0, 0); return
        cx, cy = pos_from_odom(self.limo1_odom)
        gx, gy = self.evade_goal
        if dist2d(cx, cy, gx, gy) < 0.04:
            self._pub(self.limo1_cmd, 0, 0)
            self._transition_to(MissionState.LIMO2_EXIT_INIT); return
        cur_yaw = yaw_from_odom(self.limo1_odom)
        tgt_yaw = norm_angle(math.atan2(gy-cy, gx-cx) + math.pi)
        yaw_err = norm_angle(tgt_yaw - cur_yaw)
        remaining = dist2d(cx, cy, gx, gy)
        linear  = max(self.EVADE_SPEED, min(-0.01, -remaining * 0.08))
        angular = float(np.clip(self.EVADE_YAW_KP * yaw_err, -1.5, 1.5))
        self._pub(self.limo1_cmd, linear, angular)

# ── LIMO 2 Departure ──────────────────────────────────────────────────────
    def _exit_init(self):
        self._build_limo2_waypoints()
        self._transition_to(MissionState.LIMO2_EXITING)

    def _exiting(self):
        if self.limo2_odom is None: return
        if self.limo2_wp_idx < len(self.limo2_waypoints):
            wp = self.limo2_waypoints[self.limo2_wp_idx]
            if self._drive_limo2(wp[0], wp[1], wp[2]):
                self.limo2_wp_idx += 1
        else:
            self._pub(self.limo2_cmd, 0, 0)
            self._transition_to(MissionState.LIMO1_SCAN)

    def _drive_limo2(self, gx, gy, g_yaw) -> bool:
        cx, cy  = pos_from_odom(self.limo2_odom)
        cur_yaw = yaw_from_odom(self.limo2_odom)
        d = dist2d(cx, cy, gx, gy)
        if d < 0.05:
            ye = norm_angle(g_yaw - cur_yaw)
            if abs(ye) < 0.15:
                self._pub(self.limo2_cmd, 0, 0); return True
            self._pub(self.limo2_cmd, 0, float(np.clip(self.LIMO2_ANGULAR_KP*ye, -0.8, 0.8)))
            return False
        tgt = math.atan2(gy-cy, gx-cx)
        ye  = norm_angle(tgt - cur_yaw)
        lin = float(np.clip(self.LIMO2_LINEAR*(1-abs(ye)/math.pi), 0.01, self.LIMO2_LINEAR))
        ang = float(np.clip(self.LIMO2_ANGULAR_KP*ye, -1.0, 1.0))
        self._pub(self.limo2_cmd, lin, ang)
        return False

# ── Slot scan ─────────────────────────────────────────────────────────
    def _scan(self):
        if self.scan_start_time is None:
            self.scan_start_time = self.get_clock().now().nanoseconds * 1e-9
        if self.target_slot is not None:
            self._transition_to(MissionState.LIMO1_REPARK); return
        elapsed = self.get_clock().now().nanoseconds*1e-9 - self.scan_start_time
        if elapsed > self.SCAN_TIMEOUT_S:
            self.get_logger().warn('슬롯 스캔 타임아웃 → fallback 사용')
            self._transition_to(MissionState.LIMO1_REPARK)

# ── Re-parking ───────────────────────────────────────────────────────────
    def _repark(self):
        if self.limo1_odom is None: return
        if self.front_obstacle_m < self.ESTOP_DIST_M:
            self._pub(self.limo1_cmd, 0, 0); return
        if   self.park_stage == ParkStage.APPROACH:  self._approach()
        elif self.park_stage == ParkStage.ARC_TURN:  self._arc_turn()
        elif self.park_stage == ParkStage.PRECISION:  self._precision()

    def _approach(self):
        gx, gy = self.repark_goal
        cx, cy = pos_from_odom(self.limo1_odom)
        if self.approach_target is None:
            self.approach_target = (gx, gy - 0.40)
        ax, ay = self.approach_target
        d = dist2d(cx, cy, ax, ay)
        if d < 0.05:
            self._pub(self.limo1_cmd, 0, 0)
            self.park_stage = ParkStage.ARC_TURN; return
        cur_yaw = yaw_from_odom(self.limo1_odom)
        ye  = norm_angle(math.atan2(ay-cy, ax-cx) - cur_yaw)
        lin = float(np.clip(0.5*d, 0.01, self.MAX_LINEAR))
        ang = float(np.clip(2.0*ye, -1.2, 1.2))
        self._pub(self.limo1_cmd, lin, ang)

    def _arc_turn(self):
        cur_yaw = yaw_from_odom(self.limo1_odom)
        ye = norm_angle(self.repark_yaw_goal - cur_yaw)
        if abs(ye) < 0.08:
            self._pub(self.limo1_cmd, 0, 0)
            self.park_stage = ParkStage.PRECISION; return
        sign = 1.0 if ye > 0 else -1.0
        ang  = sign * float(np.clip(abs(self.ARC_KP*ye), self.ARC_ANGULAR_MIN, self.ARC_ANGULAR_MAX))
        self._pub(self.limo1_cmd, 0.02, ang)

    def _precision(self):
        gx, gy   = self.repark_goal
        cx, cy   = pos_from_odom(self.limo1_odom)
        cur_yaw  = yaw_from_odom(self.limo1_odom)
        remaining = dist2d(cx, cy, gx, gy)
        yaw_err  = norm_angle(self.repark_yaw_goal - cur_yaw)
        x_err    = gx - cx
        if remaining <= 0.04:
            cond = abs(yaw_err) < self.PREC_FINISH_YAW and abs(x_err) < 0.008 \
                   and remaining < self.PREC_FINISH_DIST
            if cond:
                now = self.get_clock().now().nanoseconds * 1e-9
                if self.prec_ok_since is None:
                    self.prec_ok_since = now
                elif now - self.prec_ok_since >= self.PREC_HOLD_S:
                    self._pub(self.limo1_cmd, 0, 0)
                    self.get_logger().info('🏁 재주차 완료!')
                    self._transition_to(MissionState.FINISH); return
            else:
                self.prec_ok_since = None
        desired_yaw = self.repark_yaw_goal + self.PREC_KP_LAT * x_err
        ang = float(np.clip(self.PREC_KP_YAW * norm_angle(desired_yaw - cur_yaw), -1.5, 1.5))
        lin = float(np.clip(remaining * 0.4, 0.010, 0.022))
        self._pub(self.limo1_cmd, lin, ang)

# ── Utility ────────────────────────────────────────────────────────────
    def _pub(self, pub, lin: float, ang: float):
        m = Twist(); m.linear.x = float(lin); m.angular.z = float(ang); pub.publish(m)

    def _transition_to(self, s: MissionState):
        self.get_logger().info(f'🔀 {self.state.name} → {s.name}')
        self.state = s
        if s == MissionState.LIMO1_REPARK:
            self.park_stage = ParkStage.APPROACH
            self.approach_target = None
            self.prec_ok_since = None


def main(args=None):
    rclpy.init(args=args)
    node = RemoteParkingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6-9. `remote_parking_manager/package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>remote_parking_manager</name>
  <version>0.1.0</version>
  <description>Remote parking master FSM node</description>
  <maintainer email="you@example.com">Jihyun</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <exec_depend>gazebo_msgs</exec_depend>
  <exec_depend>numpy</exec_depend>
</package>
```

### 6-10. `remote_parking_manager/setup.py`

```python
from setuptools import setup, find_packages

package_name = 'remote_parking_manager'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'mission_manager = remote_parking_manager.mission_manager:main',
        ],
    },
)
```

### 6-11. `remote_parking_manager/setup.cfg`

```ini
[develop]
script_dir=$base/lib/remote_parking_manager
[install]
install_scripts=$base/lib/remote_parking_manager
```

---

## 7. Build & Run

### Build

```bash
cd ~/remote_parking_ws
source /opt/ros/humble/setup.bash
source ~/parking_ws/install/setup.bash      # limo_car, my_valet_parking
source ~/LIMO_platform/install/setup.bash   # 필요시
colcon build --symlink-install
source install/setup.bash
```

### Execution order

```bash
# ─── Terminal 1: Gazebo server (1 only) ────────────────────────────────
ros2 launch remote_parking_world world_server.launch.py gui:=true

# ─── Terminal 2: Robot spawn + node (per test) ──────────────────────
ros2 launch remote_parking_world spawn_robots.launch.py

# ─── Terminal 3: Start mission ───────────────────────────────────────────
ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'

# ─── Retry (~0.5 seconds without Gazebo restart) ─────────────────────────────
bash ~/remote_parking_ws/src/remote_parking_world/scripts/reset_robots.sh teleport
ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'
```

### Status monitoring

```bash
# FSM status
ros2 topic echo /remote_parking/status

# Whether to detect slots
ros2 topic echo /limo1/target_slot

# robot position
ros2 topic echo /limo1/odom --no-arr
ros2 topic echo /limo2/odom --no-arr
```

---

## 8. Full list of bugs fixed

| # | file | Bug Details | Edit |
|---|------|-----------|------|
|  1 |  world_server.launch.py | `ament_python_file_backend` non-existent package import | Remove |
|  2 |  world_server.launch.py | Missing `IfCondition` import | Add |
|  3 |  spawn_robots.launch.py | Same invalid package import | Edit to `ament_index_python` |
|  4 |  spawn_robots.launch.py | `import xacro`+`make_robot_description ()`— call structure error in launch file | Remove function, keep`Command` substituent |
|  5 |  spawn_robots.launch.py | xacro argument `robot_name` → `robot_namespace` (URDF parameter name mismatch) | Edit |
|  6 → 12 |  spawn_robots.launch.py | spawn z=`0.065` → `0.145` (base_link root, wheel height calculation error) | Edit |
|  7 → 14 |  limo_with_rear_camera.urdf.xacro | include: `urdf/limo_ackerman_base.xacro` (no Gazebo plugin) | Edit to `gazebo/ackermann_with_sensor.xacro` |
|  8 |  limo_with_rear_camera.urdf.xacro | `<xacro:arg name=" robot_name ">`→` robot_namespace ` | Edit |
|  9 → 15 |  limo_with_rear_camera.urdf.xacro | Rear camera `<namespace>` hardcoding → double namespace conflict | Leave it blank and change it to be handled by spawn |
|  10 |  mission_manager.py | Obstacle detection angle `± 20° `→`± 30° `(based on` limo_valet_parking_node.py `) | Edit |
|  11 |  spawn_robots.launch.py | Missing `allow_fallback_without_baseline`in`dynamic_tracker_node` | Add |
|  12 |  reset_robots.sh | Reset position z=`0.065` → `0.145` | Edit |
|  13 |  spawn_robots.launch.py | spawn command is missing `-robot_namespace limo 1 /limo2` (Gazebo plugin namespacing fails) | Add |

---

## 9. Known limitations and precautions

### TF crash (non-fatal)
A conflict occurs in the `/tf` tree because LIMO 1 · 2 share the same URDF link name (`base_link`, `laser_link`, etc.).
**IMPACT**: Two robots may appear overlapping in RViz.
**No effect**: Odometry-based control (`/limo1/odom`, `/limo2/odom`) operates normally.
**Solution (optional)**: Add `frame_prefix` or `tf_prefix` parameter to each robot_state_publisher.

### Hardcoding topic IMU (check `sensor.xacro`)
```xml
<topicName>/limo/imu</topicName>  <!-- 절대 경로, namespace 미적용 -->
```
LIMO 1 · 2 both issue `/limo/imu` → collision.
**Impact**: Irrelevant if IMU is not used.  Topic separation is required when integrating EKF.

### base_footprint does not exist (check `ackermann.xacro`)
```xml
<!-- <link name=" base_footprint "/> --> ← Commented out
<robot_base_frame>base_footprint</robot_base_frame>  ← 플러그인 설정에는 존재
```
A TF warning may occur because the plugin references a frame that does not exist.
**Solution (optional)**: Uncomment `ackermann.xacro` and recalculate spawn z.

### Simulator speed optimization effect
| Item | Existing (start_parking_limo_master.sh) | New |
|------|--------------------------------------|------|
| Every time I run colcon build | ~10 minutes | **None** |
| gzserver sleep |  15 seconds | waiting for service |
| Physics Engine Hz |  1000 Hz | ** 500 Hz** |
| Shadow Rendering | ON | **OFF** |
| Retry time taken | ~15 minutes | **~0.5 seconds** |
