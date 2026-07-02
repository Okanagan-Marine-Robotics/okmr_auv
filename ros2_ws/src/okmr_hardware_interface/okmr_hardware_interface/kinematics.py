#! /usr/bin/env python3

"""
Description:
    Service takes in a target coordinate for the end-effector and calculates the joint positions of the arm needed to reach that position.
    If the position is outside the reach of the arm an offset to the current subs position is also calculated.

Current State:
    Initial service. Setup for the 2 servo arm with linear motion and a gripper

"""

import rclpy
from rclpy.node import Node
from okmr_msgs.srv import ArmTarget

### Arm Model Constants


class KinematicsService(Node):

    def __init__(self):
        super().__init__("arm_kinematics_service")
        self.srv = self.create_service(ArmTarget, 'arm_ik_service',self.arm_target_callback)

    def arm_target_callback(self,request, response):
        pass #TODO


def main():

    try:
        with rclpy.init():
            kin_service = KinematicsService()
            rclpy.spin(kin_service)
            
    except KeyboardInterrupt:
        pass



if __name__ == "__main__":
    main()