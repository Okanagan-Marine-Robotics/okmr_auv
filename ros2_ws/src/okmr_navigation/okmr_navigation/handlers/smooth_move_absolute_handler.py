import numpy as np
from scipy.interpolate import CubicSpline
from okmr_msgs.msg import GoalPose
from okmr_msgs.srv import DistanceFromGoal
from okmr_navigation.handlers.freeze_handler import execute_freeze
from okmr_navigation.navigator_action_server import NavigatorActionServer
from okmr_navigation.handlers.movement_execution_common import (
    execute_movement_with_monitoring,
    execute_test_movement_common,
)
from okmr_navigation.handlers.set_control_mode import set_control_mode
from okmr_msgs.msg import ControlMode

# Mirror of move_absolute_handler.py (Wrapper)

def handle_smooth_move_absolute(goal_handle):
    """Execute smooth absolute movement by generating waypoints and 
    iterativley progressing to each one"""
    
    set_control_mode(ControlMode.POSE)
    node = NavigatorActionServer.get_instance()

    waypoints = _generate_smooth_waypoints(goal_handle, node)

    # MVP reuse logic for each point
    last_result = None
    for i, waypoint_pose in enumerate(waypoints):
        node.get_logger().info(f"Moving smoothly to waypoint {i+1}/{len(waypoints)}")

        goal_handle.request.command_msg.goal_pose.pose = waypoint_pose

        last_result = execute_movement_with_monitoring(
            goal_handle, _publish_pose_goal, "distance_from_pose_goal"
        )

        if not goal_handle.is_active:
            break

        return last_result


def _generate_smooth_waypoints(goal_handle, node, num_waypoints = 10):

    """
    Generates a cubic spline between the current position and the target

    """
    from okmr_msgs.srv import GetPoseTwistAccel # Post MVP input

    # Place holder for starting coords
    start_pos = [0.0, 0.0, 0.0]

    target_pose = goal_handle.request.command_msg.goal_pose.pose
    end_pos = [target_pose.position.x, target_pose.position.y, target_pose.position.z]

    # Create an intermediate point in the spline

    mid_pos = [(s + e) / 2.0 for s, e in zip(start_pos, end_pos)]
    mid_pos[2] += 0.5 # Verticle curve adjustment
    
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


