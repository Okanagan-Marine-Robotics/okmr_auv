import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from okmr_msgs.action import Movement
from okmr_msgs.msg import MovementCommand, GoalPose

# Smooth movement test script

class SmoothMoveClient(Node):
    def __init__(self):
        super().__init__('smooth_move_client')
        self._action_client = ActionClient(self, Movement, 'movement')

    def send_goal(self):
        # Make sure server client is ready
        self.get_logger().info('Waiting on Navigator Action Server')
        self._action_client.wait_for_server()

        goal_msg = Movement.Goal()
        
        # Movement command message registered as ~9
        goal_msg.command_msg.command = 9
        
        # Generate an example 10 meter long curbe
        goal_msg.command_msg.goal_pose.pose.position.x = 10.0
        goal_msg.command_msg.goal_pose.pose.position.y = 0.0
        goal_msg.command_msg.goal_pose.pose.position.z = 1.0  # Depth:= 1m
        
        # MVP constant orientation
        goal_msg.command_msg.goal_pose.pose.orientation.w = 1.0

        # Set params for testing
        goal_msg.command_msg.radius_of_acceptance = 0.4 
        goal_msg.command_msg.timeout_sec = 60.0 

        self.get_logger().info('Sending smooth 10m movement goal')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Completion: {feedback.completion_percentage:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    action_client = SmoothMoveClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()