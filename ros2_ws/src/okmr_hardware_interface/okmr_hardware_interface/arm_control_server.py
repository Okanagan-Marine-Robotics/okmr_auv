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
from rclpy.action.server import GoalResponse, CancelResponse
from okmr_msgs.msg import ServoCommand
from okmr_msgs.action import Arm
from okmr_msgs.srv import ArmTarget
import time

SERVO_MOVE_RATE = 60.0 #deg/s, placeholder value for now, need to find actual servo spec
SERVO_PWM_MIN = 125
SERVO_PWM_MAX = 600
SERVO_RANGE = 180
MAX_REACH = 10 #TODO update this based on arm drawings
NUM_JOINTS = 2

class ArmActionServer(Node):

    default_poses = [0, 0]   #Default angles the arm should assume at initial power on
    current_poses = [0, 0]  #For tracking the current joint positions

    def __init__(self):
        super.__init__('arm_action_server')
        self._action_server = ActionServer(
            self,
            Arm,
            'arm_movement',
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel,
            execute_callback=self.arm_callback)
        
        self.cli = self.create_client(ArmTarget, 'arm_kinematics')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('kinematics service not available, waiting again...')
        self.req = ArmTarget.Request()
        
        self.servo_pub = self.create_publisher(
               ServoCommand, "/servo_command", 10)
        
        for i in range(len(self.default_poses)): #Move the arm to default position to ensure consistency
            serv_msg = ServoCommand()
            serv_msg.index = i
            serv_msg.pwm = self.ang_to_pwm(self.default_poses[i])
            self.servo_pub.publish(serv_msg)
    
    def handle_goal(self, goal_handle):
        dist = goal_handle.request.targetDistance
        if dist > MAX_REACH or dist < 0:
            return GoalResponse.REJECT
        else:
            return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        #For now reject all cancle requests
        #Currently cancelling mid-movement will throw off the internal tracking of joint positions
        return CancelResponse.REJECT

    def ang_to_pwm(self, angle):
        if angle < 0:
            angle = 0
        if angle > SERVO_RANGE:
            angle = SERVO_RANGE
        pwm = SERVO_PWM_MIN + (SERVO_PWM_MAX-SERVO_PWM_MIN)*(angle/SERVO_RANGE)
        return pwm
        

    async def arm_callback(self, goal_handle):
        target = goal_handle.request.targetDistance
        gripper_state = goal_handle.request.gripperState

        self.req.targetDistance = target
        self.req.gripperState = gripper_state

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        joints = self.future.result().joint_poses

        max_ang_dif = 0.0
        for i in range(len(joints)):
            max_ang_dif = max(max_ang_dif,abs(self.current_poses[i]-joints[i]))
        
        movement_time = max_ang_dif/SERVO_MOVE_RATE

        feedback_msg = Arm.Feedback()
        feedback_msg.status = Arm.JOINTS_MOVING
        goal_handle.publish_feedback(feedback_msg)

        if len(joints) != NUM_JOINTS:
            result=Arm.Result()
            result.exitStatus = Arm.UNEXPECTED_JOINT_AMOUNT
            self.get_logger.warn(f"Arm action server given incorrect amount of joint values. Recieved: {len(joints)} Expected: {NUM_JOINTS}")
            goal_handle.abort()
            return result

        for i in range(len(joints)):
             self.current_poses[i] = joints[i]
             #TODO servo index remapping may be needed
             serv_msg = ServoCommand()
             serv_msg.index = i
             serv_msg.pwm = self.ang_to_pwm(joints[i])
             self.servo_pub.publish(serv_msg)
        
        time.sleep(movement_time) #TODO non-blocking delay
             
        
        result = Arm.Result()
        result.exitStatus = Arm.TARGET_REACHED
        goal_handle.succeed()
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