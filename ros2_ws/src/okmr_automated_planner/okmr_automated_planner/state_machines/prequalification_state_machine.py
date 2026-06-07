from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.srv import SetDeadReckoningEnabled
from okmr_msgs.msg import MovementCommand, MissionCommand


class PrequalificationStateMachine(BaseStateMachine):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.distance_forward = self.get_local_parameter("distance_forward")
        self.ros_node.get_logger().info(f"Distance forward: {self.distance_forward}")

        self.distance_down = self.get_local_parameter("distance_down")
        self.ros_node.get_logger().info(f"Distance down: {self.distance_down}")

        self.rotation = self.get_local_parameter("turn")
        
        self.distance_small = self.get_local_parameter("distance_small")

    def on_enter_initializing(self):
        self.queued_method = self.initialized

    def on_enter_moving_down(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.z = -self.distance_down

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_down_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send sinking movement command")
            self.queued_method = self.abort
            
    def on_enter_moving_small(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.distance_small
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_small_done,
            on_failure=self.abort
        )
        
        if not success:
            self.ros_node.get_logger().error("Failed to send small forward movement command")
            self.queued_method = self.abort

    def on_enter_moving_forward(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.distance_forward

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_forward_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send forward movement command")
            self.queued_method = self.abort
    
    def on_enter_turn(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.rotation

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_forward_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send forward movement command")
            self.queued_method = self.abort


    def on_enter_surfacing(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SURFACE_PASSIVE

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.surfacing_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error(
                "Failed to send surfacing movement command"
            )
            self.queued_method = self.abort
        pass

    def on_completion(self):
        self.ros_node.get_logger().info(
            make_green_log("Qualification State Machine Exiting")
        )
