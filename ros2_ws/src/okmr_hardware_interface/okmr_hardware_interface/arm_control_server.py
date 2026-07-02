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
from okmr_msgs.srv import ArmTarget
import time

SERVO_MOVE_RATE = 60.0 #deg/s, placeholder value for now, need to find actual servo spec

class ArmActionServer(Node):

    default_poses = [0, 0]   #Default angles the arm should assume at initial power on
    current_poses = [0, 0]  #For tracking the current joint positions

    def __init__(self):
        super.__init__('arm_action_server')
        self._action_server = ActionServer(
            self,
            Arm,
            'arm_movement',
            self.arm_callback)
        self.cli = self.create_client(ArmTarget, 'arm_kinematics')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('kinematics service not available, waiting again...')
        self.req = ArmTarget.Request()


    async def arm_callback(self, goal):
        target = goal.request.targetPose
        gripper_state = goal.request.gripperState

        self.req.targetPose = target
        self.req.gripperState = gripper_state

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        joints = self.future.result().joint_poses

        max_ang_dif = 0.0
        for i in range(len(joints)):
            max_ang_dif = max(max_ang_dif,abs(self.current_poses[i]-joints[i]))
        
        movement_time = max_ang_dif/SERVO_MOVE_RATE

        feedback_msg = Arm.Feedback()
        feedback_msg.status = 0
        goal.publish_feedback(feedback_msg)

        time.sleep(movement_time)

        for i in range(len(joints)):
             self.current_poses[i] = joints[i]
        
        result = Arm.Result()
        result.exitStatus = 0
        return result
             







def main(args=None):
    try:
        with rclpy.init(args=args):
                arm_action_server = ArmActionServer()
                rclpy.spin(arm_action_server)
    except KeyboardInterrupt:
         pass

if __name__ == '__main__':
     main()