# AUV Launch & Operation Commands

## Simulation Notes

**`onnxruntime` must be installed** for the object detection node to run. Without it, `onnx_segmentation_detector` will crash on launch:
```bash
pip install onnxruntime        # CPU
pip install onnxruntime-gpu    # GPU (Jetson / CUDA)
```

**Camera topic remapping** is handled automatically by `sim_mode:=true`. In sim the object detection node remaps its inputs:
- `/rgb` → `/ogopogo/front/color/image_raw`
- `/depth` → `/ogopogo/front/depth/image_rect_raw`

**`get_pose_twist_accel` service failure** — if the navigator logs `get_pose_twist_accel service returned failure or invalid response` and the mission aborts immediately, the dead reckoning node is not providing a valid pose. Confirm the stonefish simulator is fully loaded and publishing IMU/DVL data before sending the mission start command.

---

## Launch Commands

### Simulation

```bash
# Stonefish simulator (default scenario)
ros2 launch okmr_stonefish sim.launch.py

# Stonefish simulator with specific scenario
ros2 launch okmr_stonefish sim.launch.py scenario_name:=simple.scn

# Headless simulation (no GPU rendering)
ros2 launch okmr_stonefish sim_headless.launch.py scenario_name:=simple_headless.scn
```

### Full System (Simulation Mode)

```bash
# Launch everything: sim, controls, navigation, object detection, mapping, automated planner
ros2 launch ogopogo ogopogo_system.launch.py sim_mode:=true
```

### Individual Subsystems

```bash
# Control stack
ros2 launch okmr_controls full_control_stack.launch.py folder:=default

# Navigation stack
ros2 launch okmr_navigation full_navigation_stack.launch.py

# Object detection
ros2 launch okmr_object_detection full_object_detection_system.launch.py debug:=true

# Mapping
ros2 launch okmr_mapping full_mapping_system.launch.py use_semantic_subscriber:=false

# Automated planner
ros2 launch okmr_automated_planner automated_planner.launch.py \
  root_config:='task_state_machines/finding_gate.yaml' \
  config_folder:=dev \
  debug:=true

# Automated planner (test mode, no full stack required)
ros2 launch okmr_automated_planner test_automated_planner.launch.py \
  root_config:='task_state_machines/finding_gate.yaml'
```

### Teleoperation

```bash
# Keyboard control
ros2 launch okmr_teleoperation keyboard_control.launch.py

# Xbox controller
ros2 launch okmr_teleoperation xbox_control.launch.py
```

### Hardware (Pool Tests)

```bash
# Cameras (RealSense D455 front, D405 bottom)
ros2 launch ogopogo cameras.launch.py

# Hardware interface (DVL, ESP32, temp sensor, thrust-to-PWM)
ros2 launch ogopogo pool_tests/08-11-25-hardware-interface.launch.py

# Controls stack (hardware)
ros2 launch ogopogo pool_tests/08-11-25-controls.launch.py

# Navigation stack (hardware)
ros2 launch ogopogo pool_tests/08-12-25-navigation.launch.py

# Automated planner - qualification run
ros2 launch ogopogo pool_tests/08-13-25-automated-planner-qual.launch.py
```

---

## Mission Commands

Mission commands are published to `/mission_command` (type: `okmr_msgs/msg/MissionCommand`).

```bash
# Start mission
ros2 topic pub /mission_command okmr_msgs/msg/MissionCommand "{command: 1}" --once

# Kill mission
ros2 topic pub /mission_command okmr_msgs/msg/MissionCommand "{command: 2}" --once
```

| Value | Constant        |
|-------|-----------------|
| `1`   | `START_MISSION` |
| `2`   | `KILL_MISSION`  |

---

## Movement Commands

Movement commands are sent as ROS2 actions to `/movement_command` (type: `okmr_msgs/action/Movement`).

The goal contains a `MovementCommand` with a `command` field that selects the movement type.

### Command Types

| Value | Constant               | Description |
|-------|------------------------|-------------|
| `0`   | `FREEZE`               | Hold current pose, stop all movement |
| `1`   | `MOVE_RELATIVE`        | Move relative to current pose (meters / degrees) |
| `2`   | `MOVE_ABSOLUTE`        | Move to absolute goal pose |
| `3`   | `SET_VELOCITY`         | Move at constant velocity for a duration |
| `4`   | `LOOK_AT`              | Rotate to face a specific point |
| `5`   | `SET_DEPTH`            | Dive/rise to target depth |
| `6`   | `SURFACE_PASSIVE`      | Cut motors and let buoyancy surface the AUV |
| `7`   | `BARREL_ROLL`          | Perform a barrel roll using goal_velocity |
| `8`   | `SET_ALTITUDE`         | Maintain altitude above the seafloor |
| `9`   | `SMOOTH_MOVE_ABSOLUTE` | Move absolute with cubic spline path |

### Examples

**Freeze (stop all movement):**
```bash
ros2 action send_goal /movement_command okmr_msgs/action/Movement \
  "{command_msg: {command: 0, timeout_sec: 5.0}}"
```

**Move forward 2 m (relative):**
```bash
ros2 action send_goal /movement_command okmr_msgs/action/Movement \
  "{command_msg: {command: 1, translation: {x: 2.0, y: 0.0, z: 0.0}, timeout_sec: 30.0}}"
```

**Move left 2 m (relative):**
```bash
ros2 action send_goal /movement_command okmr_msgs/action/Movement \
  "{command_msg: {command: 1, translation: {x: 0.0, y: 2.0, z: 0.0}, timeout_sec: 30.0}}"
```

**Move up 1 m (relative):**
```bash
ros2 action send_goal /movement_command okmr_msgs/action/Movement \
  "{command_msg: {command: 1, translation: {x: 0.0, y: 0.0, z: 1.0}, timeout_sec: 30.0}}"
```

**Rotate 90 degrees yaw (relative):**
```bash
ros2 action send_goal /movement_command okmr_msgs/action/Movement \
  "{command_msg: {command: 1, rotation: {x: 0.0, y: 0.0, z: 90.0}, timeout_sec: 30.0}}"
```

**Set depth to 2 m:**
```bash
ros2 action send_goal /movement_command okmr_msgs/action/Movement \
  "{command_msg: {command: 5, altitude: 2.0, timeout_sec: 30.0}}"
```

**Set velocity — surge forward at 0.5 m/s for 5 s:**
```bash
ros2 action send_goal /movement_command okmr_msgs/action/Movement \
  "{command_msg: {command: 3, goal_velocity: {integrate: false, duration: 5.0, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}, timeout_sec: 10.0}}"
```

**Surface passively:**
```bash
ros2 action send_goal /movement_command okmr_msgs/action/Movement \
  "{command_msg: {command: 6, timeout_sec: 60.0}}"
```

**Barrel roll (100 deg/s roll for 3.6 s):**
```bash
ros2 action send_goal /movement_command okmr_msgs/action/Movement \
  "{command_msg: {command: 7, goal_velocity: {integrate: true, duration: 3.6, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 100.0, y: 0.0, z: 0.0}}}, timeout_sec: 6.0}}"
```

> **Note:** `translation` values are in **meters**, `rotation` values are in **degrees**.

---

## Topic Commands

### Control Mode

Published to `/control_mode` (type: `okmr_msgs/msg/ControlMode`). Used for manual debugging/tuning.

```bash
# Pose mode (position PID)
ros2 topic pub /control_mode okmr_msgs/msg/ControlMode "{control_mode: 0}" --once

# Velocity mode
ros2 topic pub /control_mode okmr_msgs/msg/ControlMode "{control_mode: 1}" --once

# Acceleration mode (debug only)
ros2 topic pub /control_mode okmr_msgs/msg/ControlMode "{control_mode: 2}" --once

# Thrust mode (debug only)
ros2 topic pub /control_mode okmr_msgs/msg/ControlMode "{control_mode: 3}" --once

# Throttle mode (debug only)
ros2 topic pub /control_mode okmr_msgs/msg/ControlMode "{control_mode: 4}" --once

# Off (disable control system)
ros2 topic pub /control_mode okmr_msgs/msg/ControlMode "{control_mode: 127}" --once
```

### Goal Velocity (direct velocity setpoint)

Published to `/goal_velocity` (type: `okmr_msgs/msg/GoalVelocity`).

```bash
# Surge forward at 0.5 m/s for 5 s
ros2 topic pub /goal_velocity okmr_msgs/msg/GoalVelocity \
  "{integrate: false, duration: 5.0, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" --once

# Sway left at 0.5 m/s for 5 s
ros2 topic pub /goal_velocity okmr_msgs/msg/GoalVelocity \
  "{integrate: false, duration: 5.0, twist: {linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" --once

# Heave up at 0.5 m/s for 5 s
ros2 topic pub /goal_velocity okmr_msgs/msg/GoalVelocity \
  "{integrate: false, duration: 5.0, twist: {linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" --once

# Yaw at 10 deg/s for 5 s
ros2 topic pub /goal_velocity okmr_msgs/msg/GoalVelocity \
  "{integrate: false, duration: 5.0, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 10.0}}}" --once
```

### Motor Throttle (raw PWM — hardware debug only)

Published to `/motor_throttle` (type: `okmr_msgs/msg/MotorThrottle`). Range: 1100–1700 μs, neutral = 1500.

```bash
# All motors neutral
ros2 topic pub /motor_throttle okmr_msgs/msg/MotorThrottle \
  "{throttle: [1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0]}" -r 100

# Test motor 0 only
ros2 topic pub /motor_throttle okmr_msgs/msg/MotorThrottle \
  "{throttle: [1600.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0]}" -r 100
```

### Motor Thrust (force in Newtons — hardware debug only)

Published to `/motor_thrust` (type: `okmr_msgs/msg/MotorThrust`).

```bash
# All motors zero thrust
ros2 topic pub /motor_thrust okmr_msgs/msg/MotorThrust \
  "{thrust: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once
```

---

## Service Calls

### Navigation

```bash
# Enable dead reckoning
ros2 service call /set_dead_reckoning_enabled okmr_msgs/srv/SetDeadReckoningEnabled "{enable: true}"

# Disable dead reckoning
ros2 service call /set_dead_reckoning_enabled okmr_msgs/srv/SetDeadReckoningEnabled "{enable: false}"

# Get current pose, velocity, and acceleration
ros2 service call /get_pose_twist_accel okmr_msgs/srv/GetPoseTwistAccel "{}"

# Clear / reset pose estimate
ros2 service call /clear_pose okmr_msgs/srv/ClearPose "{}"

# Query distance from current navigation goal
ros2 service call /distance_from_goal okmr_msgs/srv/DistanceFromGoal "{}"
```

### Controls

```bash
# Enable thrust allocation
ros2 service call /enable_thrust_allocation okmr_msgs/srv/EnableThrustAllocation "{enable: true}"

# Disable thrust allocation
ros2 service call /enable_thrust_allocation okmr_msgs/srv/EnableThrustAllocation "{enable: false}"
```

### Object Detection

```bash
# Switch detection model
ros2 service call /change_model okmr_msgs/srv/ChangeModel "{model_id: 1}"   # GATE
ros2 service call /change_model okmr_msgs/srv/ChangeModel "{model_id: 2}"   # SHARK
ros2 service call /change_model okmr_msgs/srv/ChangeModel "{model_id: 3}"   # SWORDFISH
ros2 service call /change_model okmr_msgs/srv/ChangeModel "{model_id: 4}"   # PATH_MARKER
ros2 service call /change_model okmr_msgs/srv/ChangeModel "{model_id: 5}"   # SLALOM_CENTER
ros2 service call /change_model okmr_msgs/srv/ChangeModel "{model_id: 6}"   # SLALOM_OUTER
ros2 service call /change_model okmr_msgs/srv/ChangeModel "{model_id: 7}"   # DROPPER_BIN
ros2 service call /change_model okmr_msgs/srv/ChangeModel "{model_id: 8}"   # TORPEDO_BOARD

# Switch inference camera
ros2 service call /set_inference_camera okmr_msgs/srv/SetInferenceCamera "{camera_mode: 0}"   # DISABLED
ros2 service call /set_inference_camera okmr_msgs/srv/SetInferenceCamera "{camera_mode: 1}"   # FRONT_CAMERA
ros2 service call /set_inference_camera okmr_msgs/srv/SetInferenceCamera "{camera_mode: 2}"   # BOTTOM_CAMERA
```

---

## Monitoring / Debug

### Topic Inspection

```bash
# List all active topics
ros2 topic list

# Monitor current pose
ros2 topic echo /pose

# Monitor current velocity
ros2 topic echo /velocity

# Monitor motor thrust output
ros2 topic echo /motor_thrust

# Monitor battery voltage
ros2 topic echo /battery_voltage

# Monitor object detection mask offset
ros2 topic echo /mask_offset
```

### Node Inspection

```bash
# List all running nodes
ros2 node list

# Inspect the navigation action server
ros2 node info /navigator_action_server

# List all actions
ros2 action list

# List all services
ros2 service list
```

### Visualization

Foxglove bridge runs on port `8765` when the full system is launched. Open Foxglove Studio and connect to `ws://<robot-ip>:8765`.

---

## Complete Operation Sequences

### Simulation — Automated Mission

```bash
# Terminal 1: Launch full simulation stack
ros2 launch ogopogo ogopogo_system.launch.py sim_mode:=true

# Terminal 2: Launch automated planner with state machine
ros2 launch okmr_automated_planner automated_planner.launch.py \
  root_config:='task_state_machines/qualifying.yaml' \
  config_folder:=dev \
  debug:=true

# Terminal 3: Start the mission
ros2 topic pub /mission_command okmr_msgs/msg/MissionCommand "{command: 1}" --once

# To kill the mission at any time:
ros2 topic pub /mission_command okmr_msgs/msg/MissionCommand "{command: 2}" --once
```

### Hardware — Pool Test

```bash
# Terminal 1: Cameras
ros2 launch ogopogo cameras.launch.py

# Terminal 2: Hardware interface + controls + navigation
ros2 launch ogopogo pool_tests/08-11-25-controls.launch.py

# Terminal 3: Object detection
ros2 launch okmr_object_detection full_object_detection_system.launch.py debug:=true

# Terminal 4: Mapping
ros2 launch okmr_mapping full_mapping_system.launch.py use_semantic_subscriber:=false

# Terminal 5: Automated planner (qualification)
ros2 launch ogopogo pool_tests/08-13-25-automated-planner-qual.launch.py

# Terminal 6: Start mission
ros2 topic pub /mission_command okmr_msgs/msg/MissionCommand "{command: 1}" --once
```

### Manual Keyboard Control

```bash
# Terminal 1: Controls stack
ros2 launch ogopogo pool_tests/08-11-25-controls.launch.py

# Terminal 2: Keyboard teleoperation
ros2 launch okmr_teleoperation keyboard_control.launch.py
```

---

## Key Topics Reference

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/mission_command` | `okmr_msgs/MissionCommand` | Publish | Start / kill mission |
| `/movement_command` | `okmr_msgs/action/Movement` | Action | High-level movement goals |
| `/control_mode` | `okmr_msgs/ControlMode` | Publish | Switch control layer mode |
| `/goal_velocity` | `okmr_msgs/GoalVelocity` | Publish | Direct velocity setpoint |
| `/motor_throttle` | `okmr_msgs/MotorThrottle` | Publish | Raw PWM per motor (debug) |
| `/motor_thrust` | `okmr_msgs/MotorThrust` | Publish/Subscribe | Thrust per motor (N) |
| `/pose` | `geometry_msgs/PoseStamped` | Subscribe | Estimated AUV pose |
| `/velocity` | `geometry_msgs/TwistStamped` | Subscribe | Estimated AUV velocity |
| `/battery_voltage` | `okmr_msgs/BatteryVoltage` | Subscribe | Battery voltage reading |
| `/mask_offset` | `okmr_msgs/MaskOffset` | Subscribe | Vision-based detection offset |

## Key Services Reference

| Service | Type | Description |
|---------|------|-------------|
| `/set_dead_reckoning_enabled` | `okmr_msgs/srv/SetDeadReckoningEnabled` | Enable/disable dead reckoning |
| `/get_pose_twist_accel` | `okmr_msgs/srv/GetPoseTwistAccel` | Get current pose/velocity/accel |
| `/clear_pose` | `okmr_msgs/srv/ClearPose` | Reset pose estimate to origin |
| `/distance_from_goal` | `okmr_msgs/srv/DistanceFromGoal` | Distance remaining to nav goal |
| `/enable_thrust_allocation` | `okmr_msgs/srv/EnableThrustAllocation` | Enable/disable thrust allocator |
| `/change_model` | `okmr_msgs/srv/ChangeModel` | Switch object detection model |
| `/set_inference_camera` | `okmr_msgs/srv/SetInferenceCamera` | Switch detection camera feed |
