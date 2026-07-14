from transitions import Machine
from okmr_automated_planner.state_node import StateNode
from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_msgs.msg import Dvl, MovementCommand

class CoinFlipStateMachine(BaseStateMachine):
    PARAMETERS = [
        {"name": "submerge_depth", "value": -1.0, "descriptor": "Relative depth to submerge (negative for down)"},
        {"name": "gate_forward_distance", "value": 5.0, "descriptor": "Distance to move forward through the gate"},
        
        # DVL Beam Mappings
        {"name": "front_beams", "value": [0, 1], "descriptor": "List of indices for front DVL beams"},
        {"name": "back_beams", "value": [2, 3], "descriptor": "List of indices for back DVL beams"},
        {"name": "left_beams", "value": [0, 2], "descriptor": "List of indices for left DVL beams"},
        {"name": "right_beams", "value": [1, 3], "descriptor": "List of indices for right DVL beams"},

        {"name": "min_wall_detection_delta", "value": 0.5, "descriptor": "Minimum beam distance difference (m) between an axis's two beams required to trust that axis detected the start wall; tune down for shallow test tanks"},

        {"name": "orientation_timeout_sec", "value": 15.0, "descriptor": "Max time to complete the submerge + turn movement"},
        {"name": "gate_timeout_sec", "value": 15.0, "descriptor": "Max time to complete the forward move through the gate"},
    ]

    def __init__(self, name, ros_node, success_callback=None, fail_callback=None, *args, **kwargs):
        
        states = [
            StateNode("evaluating_orientation"),
            StateNode("executing_heads"),
            StateNode("executing_tails"),
            StateNode("moving_to_gate")
        ]

        transitions = [
            {"trigger": "start_evaluation", "source": "initializing", "dest": "evaluating_orientation"},
            {"trigger": "determined_heads", "source": "evaluating_orientation", "dest": "executing_heads"},
            {"trigger": "determined_tails", "source": "evaluating_orientation", "dest": "executing_tails"},
            {"trigger": "movement_completed", "source": ["executing_heads", "executing_tails"], "dest": "moving_to_gate"},
            {"trigger": "gate_passed", "source": "moving_to_gate", "dest": "done"},
        ]

        super().__init__(
            name=name,
            ros_node=ros_node,
            states=states,
            transitions=transitions,
            success_callback=success_callback,
            fail_callback=fail_callback,
            *args, 
            **kwargs
        )

        self.orientation_determined = False

    def on_enter_initializing(self):
        self.ros_node.get_logger().info(f"[{self.machine_name}]: Initializing Coin Flip Task.")
        self.add_subscription(Dvl, "/dvl", self.dvl_callback)
        self.queued_method = self.start_evaluation

    def dvl_callback(self, msg):
        if self.state != "evaluating_orientation" or self.orientation_determined:
            return

        beams = msg.beam_distances
        if len(beams) < 4:
            self.ros_node.get_logger().warn(f"[{self.machine_name}]: DVL message missing beams!")
            return

        front_idx = self.get_local_parameter("front_beams")
        back_idx = self.get_local_parameter("back_beams")
        left_idx = self.get_local_parameter("left_beams")
        right_idx = self.get_local_parameter("right_beams")

        front_avg = sum(beams[i] for i in front_idx) / len(front_idx)
        back_avg = sum(beams[i] for i in back_idx) / len(back_idx)
        left_avg = sum(beams[i] for i in left_idx) / len(left_idx)
        right_avg = sum(beams[i] for i in right_idx) / len(right_idx)

        # The start wall sits close (~0.3m) to one side of the AUV. Whichever axis
        # (front/back or left/right) shows a large imbalance is the axis facing the
        # wall: TAILS if it's the front/back axis, HEADS if it's the left/right axis.
        front_back_delta = abs(front_avg - back_avg)
        left_right_delta = abs(left_avg - right_avg)
        min_delta = self.get_local_parameter("min_wall_detection_delta")

        self.orientation_determined = True
        self.remove_subscription("/dvl")

        if front_back_delta < min_delta and left_right_delta < min_delta:
            self.ros_node.get_logger().warn(
                f"[{self.machine_name}]: Unclear DVL readings "
                f"(front/back delta={front_back_delta:.2f}, left/right delta={left_right_delta:.2f}). "
                f"Defaulting to HEADS."
            )
            self.determined_heads()
        elif front_back_delta >= left_right_delta:
            self.ros_node.get_logger().info(f"[{self.machine_name}]: Orientation Determined -> TAILS")
            self.determined_tails()
        else:
            self.ros_node.get_logger().info(f"[{self.machine_name}]: Orientation Determined -> HEADS")
            self.determined_heads()

    def on_enter_executing_heads(self):
        self.ros_node.get_logger().info(f"[{self.machine_name}]: Executing HEADS sequence (Submerge + 90 deg turn).")
        
        cmd = MovementCommand()
        cmd.command = MovementCommand.MOVE_RELATIVE
        cmd.translation.z = self.get_local_parameter("submerge_depth")
        cmd.rotation.z = 90.0
        cmd.timeout_sec = self.get_local_parameter("orientation_timeout_sec")

        success = self.movement_client.send_movement_command(
            movement_command=cmd,
            on_success=self.movement_completed,
            on_failure=self.abort
        )

        if not success:
            self.ros_node.get_logger().error(f"[{self.machine_name}]: Failed to send HEADS movement command.")
            self.queued_method = self.abort

    def on_enter_executing_tails(self):
        self.ros_node.get_logger().info(f"[{self.machine_name}]: Executing TAILS sequence (Submerge + 180 deg turn).")
        
        cmd = MovementCommand()
        cmd.command = MovementCommand.MOVE_RELATIVE
        cmd.translation.z = self.get_local_parameter("submerge_depth")
        cmd.rotation.z = 180.0
        cmd.timeout_sec = self.get_local_parameter("orientation_timeout_sec")

        success = self.movement_client.send_movement_command(
            movement_command=cmd,
            on_success=self.movement_completed,
            on_failure=self.abort
        )

        if not success:
            self.ros_node.get_logger().error(f"[{self.machine_name}]: Failed to send TAILS movement command.")
            self.queued_method = self.abort

    def on_enter_moving_to_gate(self):
        self.ros_node.get_logger().info(f"[{self.machine_name}]: Moving forward to pass the gate.")
        
        cmd = MovementCommand()
        cmd.command = MovementCommand.MOVE_RELATIVE
        cmd.translation.x = self.get_local_parameter("gate_forward_distance")
        cmd.timeout_sec = self.get_local_parameter("gate_timeout_sec")

        success = self.movement_client.send_movement_command(
            movement_command=cmd,
            on_success=self.gate_passed,
            on_failure=self.abort
        )

        if not success:
            self.ros_node.get_logger().error(f"[{self.machine_name}]: Failed to send gate movement command.")
            self.queued_method = self.abort

    def on_completion(self):
        """Cleanup resources before handing control back to the master state machine."""
        self.ros_node.get_logger().info(f"[{self.machine_name}]: Coin Flip Sequence Successfully Completed.")
        # Ensure any active movement is stopped if the state machine completes unexpectedly
        if self.movement_client.is_movement_active():
            self.movement_client.cancel_movement()
