from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.msg import MissionCommand, ServoCommand, MovementCommand
from okmr_msgs.srv import GetPoseTwistAccel, SetDeadReckoningEnabled
import time


class RootStateMachine(BaseStateMachine):

    PARAMETERS = [
        {
            "name": "initial_wait_time",
            "value": 0.0,
            "descriptor": "Time to wait in seconds before starting mission after receiving start command",
        },
        {
            "name": "gate_distance",
            "value": 5.0,
            "descriptor": "distance to move forward through gate",
        },
        {
            "name": "turn_marker_one_angle",
            "value": 90.0,
            "descriptor": "angle to turn at marker one",
        },
        {
            "name": "turn_marker_two_angle",
            "value": 90.0,
            "descriptor": "angle to turn at marker two",
        },
        {
            "name": "turn_to_octagon_angle",
            "value": 90.0,
            "descriptor": "angle to turn toward octagon",
        },
        {
            "name": "approaching_slalom_distance",
            "value": 3.0,
            "descriptor": "distance to move forward approaching slalom",
        },
        {
            "name": "approaching_dropper_distance",
            "value": 2.5,
            "descriptor": "distance to move forward approaching dropper",
        },
        {
            "name": "approaching_octagon_distance",
            "value": 4.0,
            "descriptor": "distance to move forward approaching octagon",
        },
        {
            "name": "return_home_distance",
            "value": -10.0,
            "descriptor": "distance to move backward returning home",
        },
        {
            "name": "pass_gate_distance",
            "value": 3.0,
            "descriptor": "distance to move forward passing gate",
        },
        {
            "name": "slalom_turn_one_angle",
            "value": 45.0,
            "descriptor": "angle for first slalom turn",
        },
        {
            "name": "slalom_move_forward_one_distance",
            "value": 2.0,
            "descriptor": "distance for first slalom forward movement",
        },
        {
            "name": "slalom_turn_two_angle",
            "value": -90.0,
            "descriptor": "angle for second slalom turn",
        },
        {
            "name": "return_home_rotation_angle",
            "value": -10.0,
            "descriptor": "angle for second slalom turn",
        },
        {
            "name": "slalom_move_forward_two_distance",
            "value": 2.0,
            "descriptor": "distance for second slalom forward movement",
        },
        {
            "name": "mission_altitude",
            "value": 0.75,
            "descriptor": "target altitude for mission operations",
        },
        {
            "name": "translation_timeout_factor",
            "value": 3.0,
            "descriptor": "timeout multiplier for translation movements (distance * factor = timeout)",
        },
        {
            "name": "barrel_roll_angular_velocity",
            "value": -200.0,
            "descriptor": "angular velocity for barrel roll (degrees/sec)",
        },
        {
            "name": "barrel_roll_duration",
            "value": 3.6,
            "descriptor": "duration for barrel roll (seconds)",
        },
        {
            "name": "servo_pwm",
            "value": 1500.0,
            "descriptor": "PWM value for servo actuation",
        },
        {
            "name": "servo_index",
            "value": 0,
            "descriptor": "index of servo to actuate",
        },
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.initial_wait_time = self.get_local_parameter("initial_wait_time")
        self.gate_distance = self.get_local_parameter("gate_distance")
        self.turn_marker_one_angle = self.get_local_parameter("turn_marker_one_angle")
        self.turn_marker_two_angle = self.get_local_parameter("turn_marker_two_angle")
        self.turn_to_octagon_angle = self.get_local_parameter("turn_to_octagon_angle")
        self.approaching_slalom_distance = self.get_local_parameter(
            "approaching_slalom_distance"
        )
        self.approaching_dropper_distance = self.get_local_parameter(
            "approaching_dropper_distance"
        )
        self.approaching_octagon_distance = self.get_local_parameter(
            "approaching_octagon_distance"
        )
        self.return_home_distance = self.get_local_parameter("return_home_distance")
        self.pass_gate_distance = self.get_local_parameter("pass_gate_distance")
        self.slalom_turn_one_angle = self.get_local_parameter("slalom_turn_one_angle")
        self.return_home_rotation_angle = self.get_local_parameter(
            "return_home_rotation_angle"
        )
        self.slalom_move_forward_one_distance = self.get_local_parameter(
            "slalom_move_forward_one_distance"
        )
        self.slalom_turn_two_angle = self.get_local_parameter("slalom_turn_two_angle")
        self.slalom_move_forward_two_distance = self.get_local_parameter(
            "slalom_move_forward_two_distance"
        )
        self.mission_altitude = self.get_local_parameter("mission_altitude")
        self.translation_timeout_factor = self.get_local_parameter(
            "translation_timeout_factor"
        )
        self.barrel_roll_angular_velocity = self.get_local_parameter(
            "barrel_roll_angular_velocity"
        )
        self.barrel_roll_duration = self.get_local_parameter("barrel_roll_duration")
        self.servo_pwm = self.get_local_parameter("servo_pwm")
        self.servo_index = self.get_local_parameter("servo_index")

        self.post_gate_pose = None
        
    def on_enter_waiting(self):
        self.ros_node.get_logger().info(f"Root waiting {self.initial_wait_time}s")
        if self.initial_wait_time <= 0.0:
            self.queued_method = self.waiting_done
            return
        self.add_timer("initial_wait", self.initial_wait_time, self.handle_wait_timer)
        
    def handle_wait_timer(self):
        self.ros_node.get_logger().info("Initial wait complete")
        self.remove_timer("initial_wait")
        self.waiting_done()

    def on_enter_initializing(self):
        # check system state
        # transition to waiting for mission start
        self.queued_method = self.initialized

    def _pose_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.post_gate_pose = response.pose
                self.ros_node.get_logger().info("Post-gate pose saved successfully")
            else:
                self.ros_node.get_logger().error("Failed to get current pose")
        except Exception as e:
            self.ros_node.get_logger().error(f"Exception in pose callback: {e}")

    def _send_translation_command(
        self, distance, success_callback, failure_callback, error_msg
    ):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = distance
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = (
            10.0 + abs(distance) * self.translation_timeout_factor
        )

        success = self.movement_client.send_movement_command(
            movement_msg, on_success=success_callback, on_failure=failure_callback
        )

        if not success:
            self.ros_node.get_logger().error(error_msg)
            self.queued_method = self.abort

    def _send_rotation_command(
        self, angle, success_callback, failure_callback, error_msg
    ):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = angle
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = 10.0 + abs(angle / 10.0)

        success = self.movement_client.send_movement_command(
            movement_msg, on_success=success_callback, on_failure=failure_callback
        )

        if not success:
            self.ros_node.get_logger().error(error_msg)
            self.queued_method = self.abort

    def _send_surface_command(self, success_callback, failure_callback, error_msg):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SURFACE_PASSIVE

        success = self.movement_client.send_movement_command(
            movement_msg, on_success=success_callback, on_failure=failure_callback
        )

        if not success:
            self.ros_node.get_logger().error(error_msg)
            self.queued_method = self.abort

    def _send_sink_command(self, success_callback, failure_callback, error_msg):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = 10.0

        success = self.movement_client.send_movement_command(
            movement_msg, on_success=success_callback, on_failure=failure_callback
        )

        if not success:
            self.ros_node.get_logger().error(error_msg)
            self.queued_method = self.abort

    def _send_absolute_command(
        self, pose, success_callback, failure_callback, error_msg
    ):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_ABSOLUTE
        movement_msg.goal_pose.pose = pose
        movement_msg.goal_pose.copy_orientation = False
        movement_msg.altitude = self.mission_altitude
        movement_msg.timeout_sec = 60.0

        success = self.movement_client.send_movement_command(
            movement_msg, on_success=success_callback, on_failure=failure_callback
        )

        if not success:
            self.ros_node.get_logger().error(error_msg)
            self.queued_method = self.abort

    def mission_command_callback(self, msg):
        """Handle incoming mission command messages"""
        if msg.command == MissionCommand.START_MISSION:
            if self.is_waiting_for_mission_start():
                self.ros_node.get_logger().info("Mission start command received")
                self.mission_start_received()
            else:
                self.ros_node.get_logger().warn(
                    "Mission start command received but a mission is already running"
                )
        elif msg.command == MissionCommand.KILL_MISSION:
            self.ros_node.get_logger().warn("Mission kill command received")

    def on_enter_waiting_for_mission_start(self):
        """Wait for subscription to /mission_command topic"""
        self.ros_node.get_logger().info("Waiting for mission start command...")
        self.add_subscription(
            MissionCommand, "/mission_command", self.mission_command_callback
        )

    def check_subsystem_enable_success(self, future):
        """Check response from dead reckoning service, making sure its success"""
        try:
            response = future.result()
            if response.success:
                self.ros_node.get_logger().info(
                    f"Dead reckoning enabled successfully: {response.message}"
                )
                self.enabling_subsystems_done()
            else:
                self.ros_node.get_logger().error(
                    f"Failed to enable dead reckoning: {response.message}"
                )
                self.abort()
        except Exception as e:
            self.ros_node.get_logger().error(f"Dead reckoning service call failed: {e}")
            self.abort()

    def on_enter_enabling_subsystems(self):
        request = SetDeadReckoningEnabled.Request()
        request.enable = True

        self.send_service_request(
            SetDeadReckoningEnabled,
            "/set_dead_reckoning_enabled",
            request,
            self.check_subsystem_enable_success,
        )

    def on_enter_sinking_start(self):
        self.record_initial_start_time()
        self._send_sink_command(
            self.sinking_start_done,
            self.sinking_start_done,
            "Failed to send sinking movement command",
        )

    def on_enter_gate(self):
        self._send_translation_command(
            self.gate_distance,
            self.gate_done,
            self.gate_done,
            "Failed to send gate movement command",
        )

    def on_enter_barrel_roll(self):
        request = GetPoseTwistAccel.Request()
        self.send_service_request(
            GetPoseTwistAccel,
            "/get_pose_twist_accel",
            request,
            self._pose_callback,
        )

        time.sleep(0.1)

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.BARREL_ROLL
        movement_msg.goal_velocity.twist.angular.x = self.barrel_roll_angular_velocity
        movement_msg.goal_velocity.duration = self.barrel_roll_duration
        movement_msg.goal_velocity.integrate = True

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.barrel_roll_done,
            on_failure=self.barrel_roll_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send barrel roll movement command"
            )
            self.queued_method = self.abort

    def on_enter_surfacing(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SURFACE_PASSIVE

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.surfacing_done,
            on_failure=self.surfacing_done,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send surfacing movement command"
            )
            self.queued_method = self.abort

    def on_enter_turn_marker_one(self):
        self._send_rotation_command(
            self.turn_marker_one_angle,
            self.turn_marker_one_done,
            self.turn_marker_one_done,
            "Failed to send turn marker one movement command",
        )

    def on_enter_return_home_rotation(self):
        self._send_rotation_command(
            self.return_home_rotation_angle,
            self.return_home_rotation_done,
            self.return_home_rotation_done,
            "Failed to send turn marker one movement command",
        )

    def on_enter_approaching_slalom(self):
        self._send_translation_command(
            self.approaching_slalom_distance,
            self.approaching_slalom_done,
            self.approaching_slalom_done,
            "Failed to send approaching slalom movement command",
        )

    def on_enter_slalom_turn_one(self):
        self._send_rotation_command(
            self.slalom_turn_one_angle,
            self.slalom_turn_one_done,
            self.slalom_turn_one_done,
            "Failed to send slalom turn one movement command",
        )

    def on_enter_slalom_move_forward_one(self):
        self._send_translation_command(
            self.slalom_move_forward_one_distance,
            self.slalom_move_forward_one_done,
            self.slalom_move_forward_one_done,
            "Failed to send slalom move forward one movement command",
        )

    def on_enter_slalom_turn_two(self):
        self._send_rotation_command(
            self.slalom_turn_two_angle,
            self.slalom_turn_two_done,
            self.slalom_turn_two_done,
            "Failed to send slalom turn two movement command",
        )

    def on_enter_slalom_move_forward_two(self):
        self._send_translation_command(
            self.slalom_move_forward_two_distance,
            self.slalom_move_forward_two_done,
            self.slalom_move_forward_two_done,
            "Failed to send slalom move forward two movement command",
        )

    def on_enter_turn_marker_two(self):
        self._send_rotation_command(
            self.turn_marker_two_angle,
            self.turn_marker_two_done,
            self.turn_marker_two_done,
            "Failed to send turn marker two movement command",
        )

    def on_enter_approaching_dropper(self):
        self._send_translation_command(
            self.approaching_dropper_distance,
            self.approaching_dropper_done,
            self.approaching_dropper_done,
            "Failed to send approaching dropper movement command",
        )

    def on_enter_actuating_servo(self):
        servo_msg = ServoCommand()
        servo_msg.index = self.servo_index
        servo_msg.pwm = self.servo_pwm

        servo_publisher = self.ros_node.create_publisher(
            ServoCommand, "/servo_command", 10
        )
        for i in range(10):
            servo_publisher.publish(servo_msg)
            time.sleep(0.1)

        self.ros_node.get_logger().info(
            f"Servo actuated: index={self.servo_index}, pwm={self.servo_pwm}"
        )

        self.queued_method = self.actuating_servo_done

    def on_enter_turn_to_octagon(self):
        self._send_rotation_command(
            self.turn_to_octagon_angle,
            self.turn_to_octagon_done,
            self.turn_to_octagon_done,
            "Failed to send turn to octagon movement command",
        )

    def on_enter_approaching_octagon(self):
        self._send_translation_command(
            self.approaching_octagon_distance,
            self.approaching_octagon_done,
            self.approaching_octagon_done,
            "Failed to send approaching octagon movement command",
        )

    def on_enter_surfacing_octagon(self):
        self._send_surface_command(
            self.surfacing_octagon_done,
            self.surfacing_octagon_done,
            "Failed to send surfacing octagon movement command",
        )

    def on_enter_sinking_octagon(self):
        self._send_sink_command(
            self.sinking_octagon_done,
            self.sinking_octagon_done,
            "Failed to send sinking octagon movement command",
        )

    def on_enter_return_home(self):
        if self.post_gate_pose is not None:
            self._send_absolute_command(
                self.post_gate_pose,
                self.return_home_done,
                self.return_home_done,
                "Failed to send return home absolute movement command",
            )
        else:
            self.ros_node.get_logger().warn(
                "No post-gate pose saved, using relative movement"
            )
            self._send_translation_command(
                self.return_home_distance,
                self.return_home_done,
                self.return_home_done,
                "Failed to send return home movement command",
            )

    def on_enter_pass_gate(self):
        self._send_translation_command(
            self.pass_gate_distance,
            self.pass_gate_done,
            self.pass_gate_done,
            "Failed to send pass gate movement command",
        )

    def on_enter_surface(self):
        self._send_surface_command(
            self.surface_done,
            self.surface_done,
            "Failed to send surface movement command",
        )
        
    def on_enter_stage_open(self):
        
        self.start_current_state_sub_machine(
            success_callback=self.stage_open_done, fail_callback=self.abort
        )
        
    def on_enter_stage_close(self):
        self.current_sub_machine.stage_close(
            success_callback=self.stage_close_done, fail_callback=self.abort
        )

    def on_enter_prequalification(self):
        self.start_current_state_sub_machine(
            success_callback=self.prequalification_done, fail_callback=self.abort
        )

    def on_enter_doing_gate_task(self):
        self.start_current_state_sub_machine(
            success_callback=self.doing_gate_task_done, fail_callback=self.abort
        )

    def on_completion(self):
        self.ros_node.get_logger().info(make_green_log("Root State Machine Exiting"))
