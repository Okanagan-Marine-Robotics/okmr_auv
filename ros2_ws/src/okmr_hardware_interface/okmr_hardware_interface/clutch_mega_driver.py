#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from okmr_msgs.msg import MotorThrottle, MissionCommand, ControlMode, ServoCommand, ActuatorCommand
import serial  # Import pyserial
import time


class SerialOutputNode(Node):
    def __init__(self):
        super().__init__("clutch_mega_driver")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("polling_rate", 40.0)  # Hz
        self.declare_parameter("leak_threshold", 800)
        self.declare_parameter("motor_index_remapping", [0, 1, 2, 3, 4, 5, 6, 7])
        serial_port_path = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        polling_rate = (
            self.get_parameter("polling_rate").get_parameter_value().double_value
        )
        self.leak_threshold = (
            self.get_parameter("leak_threshold").get_parameter_value().integer_value
        )
        self.motor_index_remapping = (
            self.get_parameter("motor_index_remapping")
            .get_parameter_value()
            .integer_array_value
        )

        try:
            self.serial_port = serial.Serial(serial_port_path, baud_rate, timeout=1)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

        self.subscription = self.create_subscription(
            MotorThrottle, "/motor_throttle", self.throttle_callback, 10
        )

        self.servo_subscription = self.create_subscription(
            ServoCommand, "/servo_command", self.servo_command_callback, 10
        )

        self.actuator_subscription = self.create_subscription(
            ActuatorCommand, "/actuator_command", self.actuator_command_callback, 10
        )

        self.mission_publisher = self.create_publisher(
            MissionCommand, "/mission_command", 10
        )
        self.control_mode_publisher = self.create_publisher(
            ControlMode, "/control_mode", 10
        )

        self.timer = self.create_timer(1.0 / polling_rate, self.read_serial_callback)
        self.serial_buffer = ""
        self.last_killswitch_state = False

    def _write(self, msg: str):
        self.get_logger().debug(f"TX: {msg!r}")
        self.serial_port.write(msg.encode("utf-8"))

    def servo_command_callback(self, msg):
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("Serial port is not open.")
            return

        try:
            servo_index = msg.index + 100
            serial_msg = f"{servo_index}<{round(msg.pwm, 2)}\n"
            self._write(serial_msg)
        except serial.SerialException as e:
            self.get_logger().error(f"Error writing servo command to serial port: {e}")

    def actuator_command_callback(self, msg):
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("Serial port is not open.")
            return

        try:
            actuator_index = msg.index + 200
            state_value = 1 if msg.state else 0
            serial_msg = f"{actuator_index}<{state_value}\n"
            self._write(serial_msg)
        except serial.SerialException as e:
            self.get_logger().error(f"Error writing actuator command to serial port: {e}")

    def throttle_callback(self, msg):
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error("Serial port is not open.")
            return

        try:
            for i, throttle_value in enumerate(msg.throttle):
                remapped_index = self.motor_index_remapping[i]
                serial_msg = f"{remapped_index}<{round(throttle_value,2)}\n"
                self._write(serial_msg)
        except serial.SerialException as e:
            self.get_logger().error(f"Error writing to serial port: {e}")

    def read_serial_callback(self):
        if not self.serial_port or not self.serial_port.is_open:
            return

        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.in_waiting).decode(
                    "utf-8"
                )
                self.get_logger().debug(f"RX: {data!r}")
                self.serial_buffer += data

                # Process complete lines
                while "\n" in self.serial_buffer:
                    line, self.serial_buffer = self.serial_buffer.split("\n", 1)
                    self.process_serial_line(line.strip())

        except serial.SerialException as e:
            self.get_logger().error(f"Error reading from serial port: {e}")
        except UnicodeDecodeError as e:
            self.get_logger().error(f"Error decoding serial data: {e}")
            self.serial_buffer = ""  # Clear buffer on decode error
            # Clear serial port input buffer to flush corrupted data
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.reset_input_buffer()

    def process_serial_line(self, line):
        if "<" in line:
            try:
                parts = line.split("<")
                if len(parts) == 2:
                    sensor_type = parts[0]
                    value = int(parts[1])

                    if sensor_type == "killswitch":
                        killswitch_active = (
                            value == 1
                        )  # disarmed / switch open (pin HIGH because its pullup)
                        if killswitch_active and not self.last_killswitch_state:
                            self.get_logger().warn("KILLSWITCH ACTIVATED!")

                        if not killswitch_active and self.last_killswitch_state:
                            self.get_logger().warn("MISSION START RECEIVED!")
                            mission_msg = MissionCommand()
                            for i in range(3):
                                mission_msg.command = MissionCommand.START_MISSION
                                self.mission_publisher.publish(mission_msg)
                                time.sleep(0.1)

                        if killswitch_active:
                            # Send KILL_MISSION and OFF mode
                            mission_msg = MissionCommand()
                            mission_msg.command = MissionCommand.KILL_MISSION
                            # self.mission_publisher.publish(mission_msg)

                            control_msg = ControlMode()
                            control_msg.header.stamp = self.get_clock().now().to_msg()
                            control_msg.control_mode = ControlMode.OFF
                            # self.control_mode_publisher.publish(control_msg)

                        self.last_killswitch_state = killswitch_active

                    elif sensor_type == "leak":
                        if value > self.leak_threshold:
                            self.get_logger().fatal(
                                f"LEAK DETECTED! Sensor reading: {value}, threshold: {self.leak_threshold}",
                                throttle_duration_sec=1.0,
                            )

                            # Send KILL_MISSION and OFF mode
                            mission_msg = MissionCommand()
                            mission_msg.command = MissionCommand.KILL_MISSION
                            self.mission_publisher.publish(mission_msg)

                            control_msg = ControlMode()
                            control_msg.header.stamp = self.get_clock().now().to_msg()
                            control_msg.control_mode = ControlMode.OFF
                            self.control_mode_publisher.publish(control_msg)

            except ValueError as e:
                self.get_logger().warn(f"Error parsing serial line '{line}': {e}")

    def destroy_node(self):
        # Close serial port if open
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialOutputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
