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
import math

### Arm Model Constants
SERVO_TRAVEL = 180 #Servo range of motion
MOVEMENT_RATIO_ARM = 1.0 #mm movement in rack/deg rotation of the pinion attached to the servo, obtained from drawings
GRIPPER_FULL_OPEN = 0 #Servo position where the gripper is open, assumed to be default position
GRIPPER_FULL_CLOSE = 180 #Servo position where the gripper is closed

class KinematicsService(Node):

    def __init__(self):
        super().__init__("arm_kinematics_service")
        self.max_reach = SERVO_TRAVEL * MOVEMENT_RATIO_ARM
        self.srv = self.create_service(ArmTarget, 'arm_ik_service',self.arm_target_callback)

    def arm_target_callback(self,request, response):
        target = request.targetPose
        grip_angle = GRIPPER_FULL_OPEN
        if request.gripperState == ArmTarget.CLOSED:
            grip_angle = GRIPPER_FULL_CLOSE
        
        target_dist = math.sqrt(target.x**2+target.y**2+target.z**2) #Currently assuming millimeters for target units
        in_reach = target_dist <= self.max_reach #Arm action server sould also check this but left here if other nodes call the service
        arm_angle = 0
        if not in_reach:
            arm_angle = SERVO_TRAVEL #Put the arm at full extension
            self.get_logger.info("Requested arm position exceeds maximum reach")
        else:
            arm_angle = target_dist/MOVEMENT_RATIO_ARM
        
        response.joint_poses = [arm_angle, grip_angle]
        response.inReach = in_reach

        return response

        


def main(args=None): #TODO add entry points for client and server

    try:
        with rclpy.init(args=args):
            kin_service = KinematicsService()
            rclpy.spin(kin_service)

    except KeyboardInterrupt:
        pass



if __name__ == "__main__":
    main()