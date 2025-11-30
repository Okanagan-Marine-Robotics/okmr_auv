# Automated Planner using Nested State Machines

The automated planner uses the python transitions library for the state machine implementation.

Docs and Repo:  https://github.com/pytransitions/transitions

Although there is a Hierarchal State Machine in the library, its features didn't fully match up with our needs.

## Adding New State Machines
Reference the existing state machines (ex. state_machines/root_state_machine.py and state_machine_configs/dev/root.yaml) when making new state machines.

1. Create a config inside `state_machine_configs/dev/task_state_machines`.
    - Ensure you have a name for the state machine inside the actual config file.
2. Create a matching python file inside `okmr_automated_planner/state_machines`.
3. Add your new python state machine class to `state_machines/__init__.py`.
4. Connect the state machine config to the matching python class inside `state_machine_factory.py`.
    - Add an entry inside `config_to_class_dict`, format: 
`name_in_yaml : state_machines.NameInYaml`.
5. `colcon build` ros2_ws, then follow the testing instructions.

## Testing Instructions
The automated planner interfaces with the okmr_navigation navigator_action_server to send movement requests, so it must be launched to test movement commands. 

The navigator_action_server has a test mode which allows you to send movement commands that dont actually use the control systems, and instead just wait for MovementCommand.timeout_sec seconds.
(See okmr_msgs/msg/MovementCommand.msg) for more details

The launch file inside this package called "test_automated_planner.launch.py" allows you to test state machines with movement commands in isolation, so that you don't need to launch a simulation + the control systems to test your code.

Use the following command (replacing the root_config:=xyzabc portion) to launch the state machine you are testing.

This will automatically call the initialize() method on your state machine, and the on_enter_initializing() method will be called as a result.

``` bash
ros2 launch okmr_automated_planner test_automated_planner.launch.py root_config:='task_state_machines/finding_gate.yaml'
```

## Naming Conventions

 - State Names: `doing_thing`
 - Linear Transitions: `doing_thing_done`

## Code Structure

### automated_planner.py
This file is the entry and exit of the automated planner.
Defines our ros2 node AutomatedPlannerNode. Initializes the root state machine.

### base_state_machine.py
This file defines the BaseStateMachine, the superclass to all state machines used in the package.

It contains all the common methods that the state machine implementations need.

### state_machines/
This folder contains all state machine task implementations.

#### root_state_machine.py
The state machine that manages things like initializing systems, etc. First one to be called in the mission.

### state_machine_configs/
These yaml files define the states and transitions between them. They control what happens and in what order.
- Has `competition`, `dev`, and `testing` folders to be used in different scenarios.

#### root.yaml
The master file that instructs what will happen and in what order. If a task is ommitted or commented out, it will not be performed within a given run/mission.
- Corresponds with `state_machines/root_state_machine.py`

#### params.yaml
The specific values that will be used at runtime for many of the states.
- Can be overwritten by the PARAMETERS variable within each state machine.

### launch/
This folder contains run configurations. Can be used for testing or to define the configuration used on launch.
- Test files connect with okmr_navigation via creating a dead_reckoning and navigator_server node.

### Units
- Distance: Meters
- Speed: Meters per second
- Angular Velocity: Degrees per second
- Time: Seconds

## Subscribers
###### Function names below typically start with on_enter_.
All subs use [messages](#Messages) for their respective endpoints

##### `root_state_machine.py`, `qualification_state_machine.py`, `semifinal_state_machine.py`, `test_state_machine.py`
- All subscribe to the publishers:
    - `waiting_for_mission_start` subscribes to `/mission_command`
    - `enabling_subsystems` sends a service request to `/set_dead_reckoning_enabled`

##### `finding_dropper_state_machine.py`
- `detection_subscription` subscribes to `/mask_offset`

### Publisher
All pubs use [messages](#Messages) for their respective endpoints
##### `root_state_machine.py`
- `actuating_servo` publishes to `/servo_command`
    - Uses [ServoCommand](#ServoCommand) Message

## Messages
A ROS2 message, `.msg`, is simply a structured packet of data that nodes use for one-way communication with each other. No response is expected from the subscribing node.

The message type can be a built-in primitive (bool, int32, float64, string, etc) or can reference other message types.

From the ROS2 docs: 

> .msg files are simple text files that describe the fields of a ROS message.

See the [ROS2 Documentation](https://docs.ros.org/en/kilted/Concepts/Basic/About-Interfaces.html#messages) for more information on Messages.

### okmr_msgs.msg
---

#### MovementCommand
Defines a single motion command that can be sent to the navigation system, telling the motion controller what kind of movement to perform, how to move, and when to stop.
- `int8 command`: Specifies the type of movement to execute. Must match one of the following predefined constants:

| Command Name | Value | Description |
|--------------|-------|-------------|
| `FREEZE`           | 0  | Immediately stop all movement and hold current position (sets all PID controllers to position mode).  |
| `MOVE_RELATIVE`    | 1  | Move relative to the current pose. Uses translation and rotation vectors to define the motion in meters/degrees. |
| `MOVE_ABSOLUTE`    | 2  | Move to an absolute target position (goal_pose). Can optionally include a local offset using translation and rotation. |
| `SET_VELOCITY`   | 3  | Move according to a desired velocity instead of position. Uses the goal_velocity field.                     |
| `LOOK_AT`        | 4  | Rotate or orient to look at a specific point or direction defined by goal_pose.              |
| `SET_DEPTH`        | 5  | Adjust to a target depth (vertical position underwater).                                              |
| `SURFACE_PASSIVE`  | 6  | Surface by disabling all propulsion (uses buoyancy only).                                             |
| `BARREL_ROLL`      | 7  | Perform a barrel roll using angular velocity commands in goal_velocity.                               |
| `SET_ALTITUDE`     | 8  | Maintain or move to a target altitude above the seafloor.                                             |
- `geometry_msgs/Vector3 translation`: Distance to move along (x, y, z).
- `geometry_msgs/Vector3 rotation`: Rotation around (roll, pitch, yaw).
- `float32 timeout_sec`: Maximum time to complete command before considered to fail.
- `float32 radius_of_acceptance`: How close the vehicle must get to target position before completion.
- `float32 angle_threshold`: How close the vehicle's orientation must be to the target before completion.
- `float32 altitude`: Used for altitude based commands.

#### GoalPose
Used by absolute movement commands (MOVE_ABSOLUTE, LOOK_AT). Defines target position and orientation in world coordinates.
- `bool copy_orientation`: Determines if the goalPose orientation should be copied. If false, only translation is set as a goal.

#### GoalVelocity
Used by velocity-based movement commands to define the desired linear and angular velocity, duration, and whether to integrate distance over time.
- `bool integrate`: Determines if the specified velocity should be integrated to track the distance covered.
- `float32 duration`: The duration, in seconds, that the velocity should be active.
- `geometry_msgs/Twist twist`: Defines the desired linear and angular velocities.

#### MaskOffset
Used in vision-based nodes to define the detected object's positional offset in the image plane relative to the desired center.
- `float32 y_offset`: The horizontal offset in the image plane.
    * Positive values indicate a shift to the right and negative to the left.
- `float32 z_offset`: The vertical offset in the image plane.
    * Positive values indicate a shift upward and negative downward.

#### MissionCommand 
Tells us if the sub is ready to start the mission or if we need to abort.
- `int8 command`:
    - `START_MISSION=1`: Allows us to start the current mission.
    - `KILL_MISSION=2`: Tells us to kill the mission and stop.

#### ServoCommand
Defines values to be sent to the onboard servos.

- `int32 index`: Which servo to actuate (dropper, torpedo, etc.)
- `float32 pwm`: Pulse width value in microseconds controlling the servo.  1700 open 1100 close
    - Pulse-width modulation (PWM) is a way to control how much power a servo gets using a rapidly switching signal. The pulse-width is how long the signal stays on during each cycle.

### okmr_msgs.action
---

#### Movement
Used to send the [MovementCommand](#MovementCommand) messages to the action server.
- Goal: `MovementCommand command_msg`
- Result: `string debug_info`, `float32 completion_time`
- Feedback: `float32 time_elapsed`, `gemotery_msgs/Vector3 current_velocity`, `float32 completion_percentage`

### okmr_msgs.srv
---

#### Status
Status message for reporting task progress.

- `int8 status`: Current task status, one of ONGOING=0, SUCCESS=1, FAILURE=2.

#### SetInferenceCamera 
Message used when switching camera feed for inference models (like object detection).

- `int32 camera_mode`: Select which camera to use.
    - One of: `DISABLED=0`, `FRONT_CAMERA=1`, `BOTTOM_CAMERA=2`

#### ChangeModel
Used by the perception subsystem to switch the object detection model during runtime. This allows the system to dynamically load different neural network models depending on the current mission stage.

- `int32 model_id`: The ID of the model to load. Must match one of the predefined constants below:

| Constant | Value | Description |
|-----------|--------|-------------|
| `GATE` | 1 | Model for detecting gates. |
| `SHARK` | 2 | Model for detecting shark targets. |
| `SWORDFISH` | 3 | Model for detecting swordfish targets. |
| `PATH_MARKER` | 4 | Model for identifying path markers or navigation cues. |
| `SLALOM_CENTER` | 5 | Model for recognizing the center markers in the slalom task. |
| `SLALOM_OUTER` | 6 | Model for recognizing the outer markers in the slalom task. |
| `DROPPER_BIN` | 7 | Model for identifying dropper bins or containers. |
| `TORPEDO_BOARD` | 8 | Model for recognizing the torpedo board and its targets. |

- `bool success`: Indicates whether the model switch was completed successfully.
- `string message`: Additional details or error information returned by the service. 

#### SetDeadReckoningEnabled
Used by navigation to enable or disable dead reckoning mode. The service allows other nodes to toggle dead reckoning dynamically.

Request:
* `bool enable`: If dead reckoning should be true or false.

Response:
* `bool success`: Indicates whether or not the request was sucessfully executed.
* `string message`: Provides a status message.

#### GetPoseTwistAccel
Used to retrieve current estimated **pose**, **twist**, and **acceleration** data. Provides an instantaneous kinematic snapshot. (No request parameters, calling the service directly returns the latest available data.)

Response:
- `geometry_msgs/Pose pose`: Current position and orientation.
- `geometry_msgs/Twist twist`: Current linear and angular velocity.
- `geometry_msgs/Accel accel`: Current linear and angular acceleration.
- `bool success`: Whether or not the data was successfully retrived.

## Services
Services are another way for nodes to communicate, similar to publishers/subscribers, but provide information on a per-request basis instead of through continual updates.

See the [ROS2 Services Documentation](https://docs.ros.org/en/kilted/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html) for detailed information on services in general.

### send_service_request()
- The function we use to interface with services.
- Arguments:
    - `service_type`: [Message](#Messages) type as defined above
    - `service_name`: Service endpoint (ex. `/change_model`)
    - `srv_msg`: Request content
    - `done_callback`: A function to call when the request is finished. Often `None`
- If not already created, makes a service client for each request with `add_service_client`. That function creates a client and adds it to an array of all service clients.
- Async calls the service client with the `srv_msg`, calls the `done_callback` function, providing a future.

### /set_inference_camera
Change the inferencing camera (for use in inferencing tasks like object detection). Can use any of the cameras specified within the message definition. 
- Uses the [`SetInferenceCamera`](#SetInferenceCamera) message.

### /change_model
Allows us to use the [`ChangeModel`](#ChangeModel) message defined above to toggle between object detection models. Used when switching which object we need to detect, i.e. we found `GATE` and now we need to find `SHARK`.
- Uses the [`ChangeModel`](#ChangeModel) message.

### /set_dead_reckoning_enabled
Can be used to enable/disable the deadreckoning node.
- Uses the [`SetDeadReckoningEnabled`](#SetDeadReckoningEnabled) message.

### /get_pose_twist_accel
- Uses the [`GetPoseTwistAccel`](#GetPoseTwistAccel) message.
- `self._pose_callback`

## Actions

Actions are the third and final method of communication between nodes, intended for lengthy tasks. They consist of a goal, feedback, and result. Unlike services, actions can be canceled, and they provide steady feedback.

See the [ROS2 Actions Documentation](https://docs.ros.org/en/kilted/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html) for more information on actions in general.

### send_movement_command
Sends a command to the action server.
- Arguments:
    - `movement_command.type`: [MovementCommand](#MovementCommand) message
    - Optional: `on_success`, `on_failure`, `on_acceptance`, `on_rejection`, `on_feedback` as callable functions. Default None. Callback must not have required parameters.
- Returns a success/failure `bool`.
