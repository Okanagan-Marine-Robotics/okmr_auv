from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_automated_planner.state_machines.rotating_scan_state_machine import RotatingScanStateMachine
from okmr_utils.logging import make_green_log
from okmr_msgs.msg import MissionCommand, MovementCommand


class SidewaysScanStateMachine(BaseStateMachine):
    
    PARAMETERS = [
        {
            'name': 'moving_sideways_distance',
            'value': 1,
            'descriptor': 'distance to move sideways in meters',
        }
    ]
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.scan_angle = self.get_local_parameter("moving_sideways_distance")
        
    def on_enter_initializing(self):
        self.queued_method = self.initialized
        
    def on_enter_moving_sideways(self):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = self.scan_angle
        
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.moving_sideways_done,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send sideways movement command")
            self.queued_method = self.abort
    
    def on_enter_return_to_initial(self):
        # need to decide on code for saving position and calling it back
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_ABSOLUTE

             # Create goal pose for position 0,0,0
        goal_pose = GoalPose()
        goal_pose.pose = Pose()
        goal_pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        goal_pose.copy_orientation = True

        movement_msg.goal_pose = goal_pose
        movement_msg.timeout_sec = 30.0
        movement_msg.radius_of_acceptance = 0.5

        # Send movement command
        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.your_success_callback,
            on_failure=self.abort,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send move absolute command")
            self.queued_method = self.abort

        
    
    def on_enter_rotating_scan(self):
        """Start the RotatingScanStateMachine as a sub-state."""
        self.start_sub_state_machine(
            RotatingScanStateMachine,
            success_callback=self.rotating_scan_done,
            fail_callback=self.abort,
        )

    def rotating_scan_done(self):
        # Called when the rotating scan finishes successfully
        self.ros_node.get_logger().info("Rotating scan completed.")