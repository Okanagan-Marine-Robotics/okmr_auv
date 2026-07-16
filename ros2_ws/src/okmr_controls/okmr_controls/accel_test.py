#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import AccelStamped


class AccelTest(Node):
    def __init__(self):
        super().__init__("accel_test")
        self.publisher = self.create_publisher(AccelStamped, "/accel_target", 10)

        # Publishing parameters
        self.publish_rate = 10.0  # Hz
        self.publish_duration = 5.0  # seconds

        # Acceleration variables - Linear (m/s²) and Angular (rad/s²)
        self.linear_x = 0.0  # Forward/Backward acceleration
        self.linear_y = 0.0  # Left/Right acceleration
        self.linear_z = -0.5  # Up/Down acceleration

        self.angular_x = 0.0  # Roll acceleration
        self.angular_y = 0.0  # Pitch acceleration
        self.angular_z = 0.0  # Yaw acceleration

        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_accel)
        self.start_time = self.get_clock().now()

    def publish_accel(self):
        # Check if we've exceeded the publish duration
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed >= self.publish_duration:
            self.get_logger().info(
                f"Publishing completed after {self.publish_duration} seconds"
            )
            self.timer.cancel()
            self.publish_zeros()
            return

        msg = AccelStamped()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = "base_link"

        # Set linear acceleration components
        msg.accel.linear.x = self.linear_x
        msg.accel.linear.y = self.linear_y
        msg.accel.linear.z = self.linear_z

        # Set angular acceleration components
        msg.accel.angular.x = self.angular_x
        msg.accel.angular.y = self.angular_y
        msg.accel.angular.z = self.angular_z

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published acceleration - Linear: [{self.linear_x:.1f}, {self.linear_y:.1f}, {self.linear_z:.1f}] m/s², "
            f"Angular: [{self.angular_x:.1f}, {self.angular_y:.1f}, {self.angular_z:.1f}] rad/s² (t={elapsed:.1f}s)"
        )

    def publish_zeros(self):
        msg = AccelStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Set all acceleration components to zero
        msg.accel.linear.x = 0.0
        msg.accel.linear.y = 0.0
        msg.accel.linear.z = 0.0

        msg.accel.angular.x = 0.0
        msg.accel.angular.y = 0.0
        msg.accel.angular.z = 0.0

        self.publisher.publish(msg)
        self.get_logger().info("Published zero acceleration to stop the robot")


def main(args=None):
    rclpy.init(args=args)
    accel_test = AccelTest()
    rclpy.spin(accel_test)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
