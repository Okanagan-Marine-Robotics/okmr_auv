"""
Common execution logic for movement handlers to avoid code duplication
"""
from okmr_msgs.action import Movement
from okmr_msgs.srv import DistanceFromGoal
from okmr_navigation.navigator_action_server import NavigatorActionServer
from okmr_navigation.handlers.freeze_handler import execute_freeze

import time


def execute_movement_with_monitoring(goal_handle, publish_goal_func, service_name):
    """
    Common movement execution pattern with monitoring:
    1. Publish goal using provided function
    2. Monitor progress with distance service
    3. Provide feedback and handle completion/cancellation
    
    Args:
        goal_handle: ROS2 action goal handle
        publish_goal_func: Function to publish the goal (takes goal_handle as parameter)
        service_name: Name of the distance service to call
    """
    node = NavigatorActionServer.get_instance()
    
    # Publish the goal
    publish_goal_func(goal_handle)
    
    # Monitor execution with feedback
    start_time = node.get_clock().now()
    max_time = goal_handle.request.command_msg.timeout_sec
    if max_time == 0.0:
        max_time = 30.0
        #in case you forget to send a time limit!
    
    while True:
        if not goal_handle.is_active:
            result = Movement.Result()
            result.debug_info = 'Movement command was preempted'
            return result

        if goal_handle.is_cancel_requested:
            # Send freeze command before canceling
            execute_freeze()
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Movement command was canceled and vehicle frozen'
            return result
        
        # Provide feedback
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        feedback_msg.completion_percentage = (feedback_msg.time_elapsed / max_time) * 100.0
        
        # Check if we've reached the goal via distance service
        goal_distances = call_distance_service(service_name)
        
        if goal_distances is not None and is_goal_reached(goal_distances, goal_handle) and feedback_msg.time_elapsed > 0.1:
            # Movement completed
            execute_freeze()#used turning back into pose mode
            goal_handle.succeed()
            result = Movement.Result()
            result.completion_time = feedback_msg.time_elapsed
            result.debug_info = f'Movement completed successfully in {feedback_msg.time_elapsed:.2f}s'
            return result
        
        goal_handle.publish_feedback(feedback_msg)
        
        # Check timeout
        if feedback_msg.time_elapsed >= max_time:
            goal_handle.abort()
            result = Movement.Result()
            result.debug_info = f'Movement timed out after {max_time} seconds'
            execute_freeze()
            return result
        
        time.sleep(1.0 / node.feedback_rate)


def execute_test_movement_common(goal_handle):
    """Common test movement simulation based on estimated time"""
    node = NavigatorActionServer.get_instance()

    duration = goal_handle.request.command_msg.timeout_sec
    start_time = node.get_clock().now()
    
    while (node.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Movement.Result()
            result.debug_info = 'Test movement command was canceled'
            return result
        
        feedback_msg = Movement.Feedback()
        feedback_msg.time_elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        feedback_msg.completion_percentage = (feedback_msg.time_elapsed / duration) * 100.0
        
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1.0 / node.feedback_rate)
    
    goal_handle.succeed()
    result = Movement.Result()
    result.completion_time = (node.get_clock().now() - start_time).nanoseconds / 1e9
    result.debug_info = f'Test movement completed successfully in {result.completion_time:.2f}s'
    return result


def call_distance_service(service_name):
    """
    Common function to call distance services (pose or velocity).
    Both use the same DistanceFromGoal message format.
    """
    node = NavigatorActionServer.get_instance()
    
    try:
        client = node.create_client(DistanceFromGoal, service_name)
        
        # Wait for service to be available
        if not client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f'{service_name} service not available after 2s timeout')
            return None
        
        # Make the blocking service call
        request = DistanceFromGoal.Request()
        response = client.call(request)
        
        # Return tuple of translation and orientation differences
        if response is not None:
            return (response.translation_differences, response.orientation_differences)
        else:
            node.get_logger().error(f'{service_name} service returned invalid response')
            return None
            
    except Exception as e:
        node.get_logger().error(f'Exception in call_distance_service({service_name}): {str(e)}')
        return None


def is_translation_close_enough(translation_vector, threshold=0.1):
    """Check if translation distance is within threshold"""
    distance = (translation_vector.x**2 + translation_vector.y**2 + translation_vector.z**2)**0.5
    return distance <= threshold


def is_orientation_close_enough(rpy_diff, threshold=5.0):
    """Check if orientation difference is within threshold (degrees)"""
    roll = rpy_diff.x
    pitch = rpy_diff.y
    yaw = rpy_diff.z
    return abs(roll) < threshold and abs(pitch) < threshold and abs(yaw) < threshold


def is_goal_reached(goal_distances, goal_handle):
    """
    Common function to check if goal has been reached for both pose and velocity movements.
    Uses the same logic since both use the same message format and units.
    """
    if goal_distances is None:
        return False
    
    translation_diff, orientation_diff = goal_distances
    command_msg = goal_handle.request.command_msg
    
    # Both pose and velocity movements use the same criteria
    radius = command_msg.radius_of_acceptance if command_msg.radius_of_acceptance != 0.0 else 0.2
    angle_threshold = command_msg.angle_threshold if command_msg.angle_threshold != 0.0 else 5.0
    #hardcoded default values for radius of acceptance, please dont use outside of testing
    
    return (is_translation_close_enough(translation_diff, radius) and 
            is_orientation_close_enough(orientation_diff, angle_threshold))
