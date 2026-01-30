import numpy as np
from scipy.interpolate import CubicSpline
import rclpy

from okmr_msgs.msg import GoalPose, ControlMode
from okmr_msgs.srv import GetPoseTwistAccel
from okmr_navigation.navigator_action_server import NavigatorActionServer
from okmr_navigation.handlers.movement_execution_common import (
    execute_movement_with_monitoring,
)
from okmr_navigation.handlers.set_control_mode import set_control_mode

# Mirror of move_absolute_handler.py (Wrapper)

def handle_smooth_move_absolute(goal_handle):
    """Execute smooth absolute movement by generating waypoints and 
    iterativley progressing to each one"""
    
    set_control_mode(ControlMode.POSE)
    node = NavigatorActionServer.get_instance()
    
    # Define target acceptance radius and goal request for consistency
    command_msg = goal_handle.request.command_msg
    total_timeout = command_msg.timeout_sec
    radius = command_msg.radius_of_acceptance
    
    

    waypoints = _generate_smooth_waypoints(goal_handle, node)
    
    last_result = Movement.Result()
    last_result.debug_info = "No waypoints generated: Sensors not ready"

    # MVP reuse logic for each point
    last_result = None
    num_wp = len(waypoints)
    
    for i, waypoint_pose in enumerate(waypoints):
        node.get_logger().info(f"Moving smoothly to waypoint {i+1}/{len(waypoints)}")

        command_msg.goal_pose.pose = waypoint_pose
        
        command_msg.timeout_sec = total_timeout / num_wp
        command_msg.radius_of_acceptance = radius

        last_result = execute_movement_with_monitoring(
            goal_handle, _publish_pose_goal, "distance_from_pose_goal"
        )

        if not goal_handle.is_active:
            node.get_logger().warn("Smooth movement disengaged")
            break

    return last_result


def _generate_smooth_waypoints(goal_handle, node, num_waypoints = 9):

    """
    Generates a cubic spline between the current position and the target

    """
    # Cubic spline now generates between the real AUV and target position
    # Use a client to get AUV position
    
    client = node.create_client(GetPoseTwistAccel, "/get_pose_twist_accel")
    
    if not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().error("Unable to get pose information, going to default origin pos")
        start_pos = [0.0, 0.0, 0.0]
    else:
        request = GetPoseTwistAccel.Request()
        future = client.call_async(request)
        
        # Apply splin until complete to avoid a deadlocked execution!
        
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        
        if future.done() and future.result().success:
            res = future.result()
            start_pos = [
                res.pose.position.x,
                res.pose.position.y,
                res.pose.position.z
            ]
        else:
            node.get_logger().error("Nav sensors not ready, aborting mission")
            return []

    target_pose = goal_handle.request.command_msg.goal_pose.pose
    end_pos = [target_pose.position.x, target_pose.position.y, target_pose.position.z]

    # Create an intermediate point in the spline

    mid_pos = [(s + e) / 2.0 for s, e in zip(start_pos, end_pos)]
    mid_pos[2] += 0.3 # Verticle curve adjustment
    
    points = np.array([start_pos, mid_pos, end_pos])
    t = np.linspace(0, 1, len(points))
    cs = CubicSpline(t, points)

    t_new = np.linspace(0, 1, num_waypoints)
    smooth_path = cs(t_new)
    
    #print(f"DEBUG: Start={start_pos}, Mid={mid_pos}, End={end_pos}") # temp

    waypoint_poses = []
    for pos in smooth_path:
        p = GoalPose().pose
        p.position.x = float(pos[0])
        p.position.y = float(pos[1])
        # Altitude constraint
        p.position.z = max(float(pos[2]), node.min_altitude)
        # Don't change orientation for MVP
        p.orientation = target_pose.orientation
        waypoint_poses.append(p)

    return waypoint_poses

def _publish_pose_goal(goal_handle):
    """Publish goal pose from the current waypoint state"""
    node = NavigatorActionServer.get_instance()
    command_msg = goal_handle.request.command_msg
    goal_pose = command_msg.goal_pose

    # Update timestamp and publish goal pose to topic
    goal_pose.header.stamp = node.get_clock().now().to_msg()

    goal_publisher = node.get_publisher("/current_goal_pose", GoalPose, 10)
    goal_publisher.publish(goal_pose)


