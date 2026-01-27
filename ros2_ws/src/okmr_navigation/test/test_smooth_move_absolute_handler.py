import unittest
from unittest.mock import MagicMock
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from okmr_msgs.action import Movement
from okmr_navigation.navigator_action_server import NavigatorActionServer
from okmr_navigation.handlers.smooth_move_absolute_handler import _generate_smooth_waypoints

import launch
import launch_ros
import launch_testing.actions

def generate_test_description():
    """Generate launch description for testing smooth path lofic"""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_py',
            executabke='talker',
            name='dummy_talker_node'
        ),
        launch_testing.actions.ReadyToTest()
    ])
    
class TestSmoothMoveAbsoluteHandler(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
        # setup an MVP version of the navigation action server instance for basic node parameters
        cls.node = NavigatorActionServer.get_instance({}, {})
        
    @classmethod
    def tearDownClass(cls):
        if rclpy.ok(): 
            rclpy.shutdown()
        
    def setUp(self):
        """Set up standard goal request for a smooth path"""
        self.goal_handle = MagicMock()
        self.goal_handle.request.command_msg.goal_pose.pose.position.x = 10.0
        self.goal_handle.request.command_msg.goal_pose.pose.position.y = 0.0
        self.goal_handle.request.command_msg.goal_pose.pose.position.z = 2.0
        # Orientation stays constant
        self.goal_handle.request.command_msg.goal_pose.pose.orientation.w = 1.0
        
    def test_waypoint_count(self):
        """Test that the default number of waypoints are generated"""
        num_waypoints = 15
        waypoints = _generate_smooth_waypoints(self.goal_handle, self.node, num_waypoints=num_waypoints)
        
        self.assertEqual(len(waypoints), num_waypoints)
        
    def test_start_and_end_positions(self):
        """Verify the path starts near origin and ends at target"""
        waypoints = _generate_smooth_waypoints(self.goal_handle, self.node)
        
        # Current MVP starts at (0,0,0)
        self.assertAlmostEqual(waypoints[0].position.x, 0.0)
        
        # Want to match the goal target
        self.assertAlmostEqual(waypoints[-1].position.x, 10.0)
        self.assertAlmostEqual(waypoints[-1].position.z, 2.0)

    def test_altitude_safety_constrain(self):
        """Ensure waypoints never dip below min_altitude"""
        
        self.node.min_altitude = 1.0
        
        self.goal_handle.request.command_msg.goal_pose.pose.position.z = 0.5
        
        waypoints = _generate_smooth_waypoints(self.goal_handle, self.node)
        
        for p in waypoints:
            # Want every point to be >= to min altittude
            self.assertGreaterEqual(p.position.z, self.node.min_altitude)
            
    def test_constant_orientation(self):
        """Verify orientation remains constatn throughout the spline"""
        # Arbitrary orientation
        target_q = self.goal_handle.request.command_msg.goal_pose.pose.orientation
        target_q.z = 0.67
        target_q.w = 0.67
        
        waypoints = _generate_smooth_waypoints(self.goal_handle, self.node)
        
        for p in waypoints:
            self.assertEqual(p.orientation.z, 0.67)
            self.assertEqual(p.orientation.w, 0.67)
            
    def test_spline_smoothness(self):
        """Make sure a proper spline curve in generated"""
        waypoints = _generate_smooth_waypoints(self.goal_handle, self.node)
        x_coords = [p.position.x for p in waypoints]
        
        # In in single positive directional curve x values should be the same or increase
        for i in range(len(x_coords) - 1):
            self.assertLessEqual(x_coords[i], x_coords[i+1])