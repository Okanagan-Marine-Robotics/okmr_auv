"""
Shared movement helpers for all state machines.

Before, every state that needed to move the sub had to create its own MovementCommand, send it, and handle what happens if sending it failed. This ended up being the same code copied around 50 times.

Now, each state can just call a helper instead. For example:

def on_enter_moving_forward(self):
    self.move_relative(self.moving_forward_done, x=self.distance_forward)

BaseStateMachine inherits this class, so every state machine can use these helpers directly without needing extra imports.

on_success: trigger to call when the movement is done
on_failure: optional trigger if something goes wrong (defaults to self.abort)
"""

from okmr_msgs.msg import MovementCommand


class MovementCommandsMixin:

    def send_movement(self, movement_msg, on_success, on_failure=None, *, error_msg=None):
        if on_failure is None:
            on_failure = self.abort

        sent = self.movement_client.send_movement_command(
            movement_msg, on_success=on_success, on_failure=on_failure
        )

        if not sent:
            if error_msg is None:
                error_msg = (
                    f"Failed to send movement command"
                    f"\t Machine: {self.machine_name} \t State: {self.state}"
                )
            self.ros_node.get_logger().error(error_msg)
            self.queued_method = self.abort

        return sent

    def move_relative(
        self,
        on_success,
        on_failure=None,
        *,
        x=0.0,
        y=0.0,
        z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        altitude=None,
        timeout_sec=None,
        copy_orientation=None,
        error_msg=None,
    ):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.translation.x = x
        movement_msg.translation.y = y
        movement_msg.translation.z = z
        movement_msg.rotation.x = roll
        movement_msg.rotation.y = pitch
        movement_msg.rotation.z = yaw
        if altitude is not None:
            movement_msg.altitude = altitude
        if timeout_sec is not None:
            movement_msg.timeout_sec = timeout_sec
        if copy_orientation is not None:
            movement_msg.goal_pose.copy_orientation = copy_orientation

        return self.send_movement(movement_msg, on_success, on_failure, error_msg=error_msg)

    def move_absolute(
        self,
        pose,
        on_success,
        on_failure=None,
        *,
        copy_orientation=False,
        altitude=None,
        timeout_sec=None,
        error_msg=None,
    ):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_ABSOLUTE
        movement_msg.goal_pose.pose = pose
        movement_msg.goal_pose.copy_orientation = copy_orientation
        if altitude is not None:
            movement_msg.altitude = altitude
        if timeout_sec is not None:
            movement_msg.timeout_sec = timeout_sec

        return self.send_movement(movement_msg, on_success, on_failure, error_msg=error_msg)

    def rotate_at_velocity(
        self, on_success, on_failure=None, *, yaw_velocity, angle, error_msg=None
    ):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SET_VELOCITY
        movement_msg.goal_velocity.twist.angular.z = yaw_velocity
        movement_msg.goal_velocity.duration = angle / abs(yaw_velocity)
        movement_msg.goal_velocity.integrate = True
        movement_msg.timeout_sec = angle / abs(yaw_velocity) * 2

        return self.send_movement(movement_msg, on_success, on_failure, error_msg=error_msg)

    def barrel_roll(
        self, on_success, on_failure=None, *, angular_velocity, duration, error_msg=None
    ):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.BARREL_ROLL
        movement_msg.goal_velocity.twist.angular.x = angular_velocity
        movement_msg.goal_velocity.duration = duration
        movement_msg.goal_velocity.integrate = True

        return self.send_movement(movement_msg, on_success, on_failure, error_msg=error_msg)

    def surface(self, on_success, on_failure=None, *, error_msg=None):
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SURFACE_PASSIVE

        return self.send_movement(movement_msg, on_success, on_failure, error_msg=error_msg)