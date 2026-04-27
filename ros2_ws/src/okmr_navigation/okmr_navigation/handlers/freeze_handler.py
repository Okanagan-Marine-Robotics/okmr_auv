from okmr_msgs.action import Movement
from okmr_msgs.msg import GoalPose
from okmr_navigation.handlers.get_pose_twist_accel import get_current_pose, get_current_twist
from okmr_navigation.navigator_action_server import NavigatorActionServer
from okmr_navigation.handlers.set_control_mode import set_control_mode
from okmr_msgs.msg import ControlMode
import time

def handle_freeze(goal_handle):
    """Execute freeze action with monitoring"""
    node = NavigatorActionServer.get_instance()
    # Execute the common freeze functionality
    if not execute_freeze():
        goal_handle.abort()
        result = Movement.Result()
        result.debug_info = 'Failed to execute freeze command'
        return result
    
    # Wait for vehicle to come to complete stop with timeout
    start_time = node.get_clock().now()
    timeout_duration = goal_handle.request.command_msg.timeout_sec
    
    while True:
        if goal_handle.is_cancel_requested:
            # Special case: freeze action cancellation just stops without calling freeze again
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Freeze command was canceled'
            return result
        
        current_time = node.get_clock().now()
        elapsed_time = (current_time - start_time).nanoseconds / 1e9

        # Check if all velocities are under threshold
        vehicle_stopped = check_velocities_under_threshold()
        
        if vehicle_stopped:
            goal_handle.succeed()
            result = Movement.Result()
            result.completion_time = elapsed_time
            result.debug_info = f'Vehicle successfully frozen in {elapsed_time:.2f}s'
            return result
        
        # Provide feedback
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = elapsed_time
        feedback_msg.completion_percentage = min(100.0, (elapsed_time / timeout_duration) * 100.0)
        
        goal_handle.publish_feedback(feedback_msg)
        
        # Check timeout
        if elapsed_time > timeout_duration:
            goal_handle.abort()
            result = Movement.Result()
            result.debug_info = f'Freeze command timed out after {timeout_duration}s'
            return result
        time.sleep(1.0 / node.feedback_rate)


"""def execute_freeze(): Commented out to test new freeze handler hopefully fix race condition
    
    Common freeze implementation:
    1. Put vehicle into position mode
    2. Send absolute movement request at current location
    
    node = NavigatorActionServer.get_instance()
    
    set_control_mode(ControlMode.POSE)
    
    # Get current pose using service (like relative movement handler)
    current_pose = get_current_pose()
    if current_pose is None:
        node.get_logger().error("Could not get current pose for freeze command")
        return False

    # Create goal pose at current location to freeze vehicle
    freeze_goal_pose = GoalPose()
    freeze_goal_pose.header.stamp = node.get_clock().now().to_msg()
    freeze_goal_pose.pose = current_pose
    freeze_goal_pose.copy_orientation = True
    
    # Publish goal pose to freeze vehicle at current position (like absolute handler)
    goal_publisher = node.get_publisher('/current_goal_pose', GoalPose, 10)
    goal_publisher.publish(freeze_goal_pose)
    
    node.get_logger().info("Freeze command sent - vehicle will hold current position")
    return True
"""

def execute_freeze():
    
    node = NavigatorActionServer.get_instance()
    
    # Get the current pose
    current_pose = get_current_pose()
    if current_pose is None:
        node.get_logger().error("Cannot get pose for freeze command")
        return False
    
    # Publish the pose right away so there is no accumulated error to correct
    freeze_goal_pose = GoalPose()
    freeze_goal_pose.header.stamp = node.get_clock().now().to_msg()
    freeze_goal_pose.pose = current_pose
    freeze_goal_pose.copy_orientation = True
    
    goal_publisher = node.get_publisher('/current_goal_pose', GoalPose, 10)
    
    goal_publisher.publish(freeze_goal_pose)
    
    # BUffer time for PID
    time.sleep(0.05)
    
    # Enable control mode now
    set_control_mode(ControlMode.POSE)
    
    node.get_logger().info("Freeze command sent. AUV will hold position")
    
    return True


def check_velocities_under_threshold():
    """Check if all velocities are under the freeze threshold"""
    node = NavigatorActionServer.get_instance()
    
    # Get current velocity from the service
    current_twist = get_current_twist()
    if current_twist is None:
        node.get_logger().warn("Could not get current twist for velocity check")
        return False
    
    # Check linear velocities individually
    linear_vel = current_twist.linear
    linear_x_ok = abs(linear_vel.x) < node.freeze_linear_velocity_threshold
    linear_y_ok = abs(linear_vel.y) < node.freeze_linear_velocity_threshold
    linear_z_ok = abs(linear_vel.z) < node.freeze_linear_velocity_threshold
    
    # Check angular velocities individually
    angular_vel = current_twist.angular
    angular_x_ok = abs(angular_vel.x) < node.freeze_angular_velocity_threshold
    angular_y_ok = abs(angular_vel.y) < node.freeze_angular_velocity_threshold
    angular_z_ok = abs(angular_vel.z) < node.freeze_angular_velocity_threshold
    
    # Vehicle is considered stopped if all components are below their respective thresholds
    return (linear_x_ok and linear_y_ok and linear_z_ok and 
            angular_x_ok and angular_y_ok and angular_z_ok)

    
