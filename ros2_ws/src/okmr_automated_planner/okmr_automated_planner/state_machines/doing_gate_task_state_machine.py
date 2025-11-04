from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_msgs.msg import MovementCommand
from okmr_msgs.msg import GoalVelocity
from okmr_msgs.srv import Status
from geometry_msgs.msg import Vector3
from okmr_automated_planner.state_machines.sideways_scan_state_machine import SidewaysScanStateMachine


class DoingGateTaskStateMachine(BaseStateMachine):

    PARAMETERS = [
    #Copied from finding shark because shark object detection could be same as shark???????
    #       {
    #         "name": "frame_confidence_threshold",
    #         "value": 0.8,
    #         "descriptor": 'threshold that decides if a shark detection is "good enough"',
    #     },
            {
                "name": "approaching_distance",
                "value": 2.0,
                "descriptor": "distance for approaching_gate"
            },
            {
                "name": "passing_distance",
                "value": 0.2,
                "descriptor": "distance for passing gate"
            }
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.approaching_distance = self.get_local_parameter("distance_forward")
        self.passing_distance = self.get_local_parameter("passing_distance")
            
    def on_enter_initializing(self):
        self.queued_method = self.initialized
        pass

    def on_enter_approaching_gate(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.approaching_distance

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.approaching_gate_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("failed to send forward movement command")
            self.queued_method = self.abort

    def on_enter_passing_gate(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.passing_distance

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.passing_gate_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("failed to send passing movement command")
            self.queued_method = self.abort