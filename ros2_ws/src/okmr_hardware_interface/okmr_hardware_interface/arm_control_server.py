#! /usr/bin/env python3

"""
Description:
    Node publishes a target coordinate for the end-effector
------
Publishing Topics:
    /servo_command - okmr_msgs/msg/ServoCommand

Subscritipn Topics:
    None
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from okmr_msgs.msg import ServoCommand
from okmr_msgs.action import Arm

class ArmActionServer(Node):

    def __init__(self):
        super.__init__('arm_action_server')
        self._action_server = ActionServer(
            self,
            Arm,
            'arm_movement',
            self.arm_callback)


    def arm_callback(self, goal):
        pass #TODO

def main(args=None):
    try:
        with rclpy.init(args=args):
                arm_action_server = ArmActionServer()
                rclpy.spin(arm_action_server)
    except KeyboardInterrupt:
         pass

if __name__ == '__main__':
     main()