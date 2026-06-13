from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.srv import SetDeadReckoningEnabled
from okmr_msgs.msg import MovementCommand, MissionCommand


class PrequalificationStateMachine(BaseStateMachine):

    PARAMETERS = [
        {"name": "distance_forward",
         "value": 13.0,
         "descriptor": "E"
         },
         {"name": "distance_small",
         "value": 0.5,
         "descriptor": "E"
         },
         {"name": "distance_down",
         "value": 1.0,
         "descriptor": "Unused"
         },
         {"name": "turn",
         "value": 90.0,
         "descriptor": "E"
         }
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.distance_forward = self.get_local_parameter("distance_forward")
        self.ros_node.get_logger().info(f"Distance forward: {self.distance_forward}")

        self.distance_down = self.get_local_parameter("distance_down")
        self.ros_node.get_logger().info(f"Distance down: {self.distance_down}")

        self.rotation = self.get_local_parameter("turn")
        self.ros_node.get_logger().info(f"Each turn is: {self.rotation} degrees")
        
        self.distance_small = self.get_local_parameter("distance_small")
        self.ros_node.get_logger().info(f"Small distance: {self.distance_small}")

    def on_enter_initializing(self):
        self.ros_node.get_logger().info("Initializing prequalification state machine...")
        self.queued_method = self.initialized
        pass

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

    def on_enter_moving_forward_1(self):
        self.on_enter_moving_forward()

    def on_enter_moving_forward_2(self):
        self.on_enter_moving_forward()

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
    
    def on_enter_turn_1(self):
        self.on_enter_turn()

    def on_enter_turn_2(self):
        self.on_enter_turn()

    def on_enter_turn(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation.z = self.rotation

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.turn_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send turn command")
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
