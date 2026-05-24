#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from okmr_msgs.msg import ControlMode

DEADZONE = 0.05


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # Velocity scales applied to normalized joystick input [-1, 1]
        self.declare_parameter('surge_scale', 0.5)   # m/s
        self.declare_parameter('sway_scale', 0.5)    # m/s
        self.declare_parameter('heave_scale', 0.3)   # m/s
        self.declare_parameter('yaw_scale', 0.5)     # rad/s
        self.declare_parameter('roll_scale', 0.3)    # rad/s
        self.declare_parameter('pitch_scale', 0.3)   # rad/s

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/velocity_target', 10)
        self.control_mode_pub = self.create_publisher(ControlMode, '/control_mode', 10)

        self.get_logger().info("Teleop Node started")

    def joy_callback(self, msg):
        surge_scale = self.get_parameter('surge_scale').get_parameter_value().double_value
        sway_scale  = self.get_parameter('sway_scale').get_parameter_value().double_value
        heave_scale = self.get_parameter('heave_scale').get_parameter_value().double_value
        yaw_scale   = self.get_parameter('yaw_scale').get_parameter_value().double_value
        roll_scale  = self.get_parameter('roll_scale').get_parameter_value().double_value
        pitch_scale = self.get_parameter('pitch_scale').get_parameter_value().double_value

        # Xbox controller axis mapping (see params/joy_teleop.yaml)
        surge = self._deadzone(msg.axes[1])   # Left stick Y  (forward/back)
        sway  = self._deadzone(msg.axes[0])   # Left stick X  (strafe left/right)
        yaw   = self._deadzone(msg.axes[2])   # Right stick X (rotate)
        pitch = self._deadzone(msg.axes[3])   # Right stick Y (nose up/down)
        # Triggers rest at 1.0 and reach -1.0 when fully pressed
        heave_up   = msg.axes[4]              # Right trigger → ascend
        heave_down = msg.axes[5]              # Left trigger  → descend
        heave = self._deadzone((heave_down - heave_up) / 2.0)
        roll  = msg.axes[6]                   # D-pad X (binary ±1)

        # Keep the velocity control layer active
        control_mode_msg = ControlMode()
        control_mode_msg.control_mode = ControlMode.VELOCITY
        control_mode_msg.header.stamp = self.get_clock().now().to_msg()
        self.control_mode_pub.publish(control_mode_msg)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x  = surge * surge_scale
        twist.twist.linear.y  = sway * sway_scale
        twist.twist.linear.z  = heave * heave_scale
        twist.twist.angular.x = roll * roll_scale
        twist.twist.angular.y = pitch * pitch_scale
        twist.twist.angular.z = yaw * yaw_scale

        self.velocity_pub.publish(twist)

    def _deadzone(self, val, threshold=DEADZONE):
        return val if abs(val) > threshold else 0.0


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
