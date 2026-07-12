from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.msg import MissionCommand, MovementCommand
from okmr_msgs.srv import SetDeadReckoningEnabled


class Stage(BaseStateMachine):

    PARAMETERS = [
        {
            "name": "initial_wait_time",
            "value": 0.0,
            "descriptor": "Time to wait in seconds before starting mission after receiving start command",
        }
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.initial_wait_time = self.get_local_parameter("initial_wait_time")

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
        
    def on_enter_stage_open_done(self):
        self.success_callback()

    def on_enter_stage_close(self):
        self.queued_method = self.stage_closing

    def on_enter_initializing(self):
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

    def on_enter_sinking(self):
        self.record_initial_start_time()
        self._send_sink_command(
            self.sinking_start_done,
            self.sinking_start_done,
            "Failed to send sinking movement command",
        )

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

    def on_completion(self):
        self.ros_node.get_logger().info(make_green_log("Root State Machine Exiting"))
