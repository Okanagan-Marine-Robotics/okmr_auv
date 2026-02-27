#!/usr/bin/env python3

import socket
import re
import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from okmr_msgs.msg import Dvl
from okmr_msgs.msg import DvlHealth  # type: ignore[attr-defined]
from sensor_msgs.msg import FluidPressure


class DvlDriverNode(Node):
    def __init__(self):
        super().__init__("dvl_driver")
        self.dvl_publisher = self.create_publisher(Dvl, "/dvl", 10)
        self.dvl_health_pub = self.create_publisher(DvlHealth, "/dvl_health", 10)

        # Declare ROS2 parameter for beam range threshold
        self.declare_parameter("beam_range_threshold", 0.8)
        self.beam_range_threshold = 0.8

        # Register the beam range threshold param callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Store previous velocity values for outlier rejection
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vz = 0.0

        # Store previous beam distances for outlier rejection
        self.last_d1 = 0.0
        self.last_d2 = 0.0
        self.last_d3 = 0.0
        self.last_d4 = 0.0

        # Health tracking
        self.last_status = 0
        self._last_health_pub_time = 0.0

    def udp_server(self):
        # Create a UDP socket
        sock_bottom = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind the socket to the host and port
        sock_bottom.bind(("0.0.0.0", 9010))
        # Create timeout so loop isn't blocked indefinately
        sock_bottom.settimeout(15.0)

        while rclpy.ok():
            try:
                # Receive data from the client
                data, _ = sock_bottom.recvfrom(1024)  # buffer size is 1024 bytes
                data = data.decode("ASCII")

                # Regular expression to match the PNORBT8 message format
                pattern = r"\$PNORBT8,TIME=(-?\d+\.?\d*),DT1=(-?\d+\.\d+),DT2=(-?\d+\.\d+),VX=(-?\d+\.\d+),VY=(-?\d+\.\d+),VZ=(-?\d+\.\d+),FOM=(-?\d+\.\d+),D1=(-?\d+\.\d+),D2=(-?\d+\.\d+),D3=(-?\d+\.\d+),D4=(-?\d+\.\d+),BATT=(-?\d+\.\d+),SS=(-?\d+\.\d+),PRESS=(-?\d+\.\d+),TEMP=(-?\d+\.\d+),STAT=(0x[0-9A-Fa-f]+)"

                # Find the matches
                matches = re.search(pattern, data)

                if matches:
                    # Extracting values from regex groups
                    vx = float(matches.group(4))
                    vy = float(matches.group(5))
                    vz = float(matches.group(6))
                    fom = float(matches.group(7))
                    d1 = float(matches.group(8))
                    d2 = float(matches.group(9))
                    d3 = float(matches.group(10))
                    d4 = float(matches.group(11))
                    battery = float(matches.group(12))
                    speed_sound = float(matches.group(13))
                    pressure = float(matches.group(14))
                    temperature = float(matches.group(15))
                    status = int(matches.group(16), 16)  # Convert hex string to int

                    self.decode_status(status)
                    self.last_status = status

                    # Note: If bit 0 is 0 then measurement is in mm/s NOT m/s so must scale down
                    if status & 0x01:
                        scale = 1.0
                    else:
                        scale = 0.001

                    vx *= scale
                    vy *= scale
                    vz *= scale

                    # DVL velocity validation
                    velocity_magnitude = (vx**2 + vy**2 + vz**2) ** 0.5
                    if velocity_magnitude > 30.0:
                        self.get_logger().warn(
                            f"DVL velocity too high ({velocity_magnitude:.3f} m/s), last velocity - VX: {self.last_vx:.3f}, VY: {self.last_vy:.3f}, VZ: {self.last_vz:.3f}",
                            throttle_duration_sec=1.0,
                        )
                        vx = self.last_vx
                        vy = self.last_vy
                        vz = 0.0
                    else:
                        self.last_vx = vx
                        self.last_vy = vy
                        self.last_vz = vz

                    # DVL beam distance validation
                    beam_distances = [d1, d2, d3, d4]
                    beam_range = max(beam_distances) - min(beam_distances)

                    if beam_range > self.beam_range_threshold:
                        self.get_logger().warn(
                            f"DVL beam range too large ({beam_range:.3f} m), using previous values - D1: {self.last_d1:.3f}, D2: {self.last_d2:.3f}, D3: {self.last_d3:.3f}, D4: {self.last_d4:.3f}",
                            throttle_duration_sec=1.0,
                        )
                        d1 = self.last_d1
                        d2 = self.last_d2
                        d3 = self.last_d3
                        d4 = self.last_d4
                    else:
                        self.last_d1 = d1
                        self.last_d2 = d2
                        self.last_d3 = d3
                        self.last_d4 = d4

                    # Create DVL message
                    dvl_msg = Dvl()
                    dvl_msg.header.stamp = self.get_clock().now().to_msg()
                    dvl_msg.header.frame_id = "dvl"

                    # Set velocity (apply coordinate transformations if needed)
                    dvl_msg.velocity.x = vx
                    dvl_msg.velocity.y = -vy  # Transform coordinate system
                    dvl_msg.velocity.z = -vz  # Transform coordinate system

                    # Set environmental data
                    dvl_msg.water_temperature = temperature

                    # Create FluidPressure message for pressure
                    pressure_msg = FluidPressure()
                    pressure_msg.header.stamp = dvl_msg.header.stamp
                    pressure_msg.header.frame_id = "dvl"
                    pressure_msg.fluid_pressure = pressure
                    pressure_msg.variance = 0.0  # Set appropriate variance if known
                    dvl_msg.pressure = pressure_msg

                    dvl_msg.figure_of_merit = fom

                    # Set beam distances
                    dvl_msg.beam_distances = [d1, d2, d3, d4]

                    # Set additional data
                    dvl_msg.battery_voltage = battery
                    dvl_msg.speed_of_sound = speed_sound
                    dvl_msg.status = status

                    # Publish DVL message
                    self.dvl_publisher.publish(dvl_msg)

                    self.get_logger().debug(
                        f"DVL data parsed successfully - VX: {vx:.3f}, VY: {vy:.3f}, VZ: {vz:.3f}, FOM: {fom:.3f}"
                    )

                    # https://support.nortekgroup.com/hc/en-us/article_attachments/19558106638620
                    # mode 358
                else:
                    self.get_logger().debug(
                        f"DVL packet parsing failed - no regex matches found"
                    )

                # Publish health at 1 Hz regardless of parse success
                now = time.monotonic()
                if now - self._last_health_pub_time >= 1.0:
                    health_msg = DvlHealth()
                    health_msg.header.stamp = self.get_clock().now().to_msg()
                    health_msg.header.frame_id = "dvl"
                    health_msg.status = self.last_status
                    self.dvl_health_pub.publish(health_msg)
                    self._last_health_pub_time = now

            except socket.timeout:
                self.get_logger().error(
                    "DVL sensor is disconnected or not sending data/data not being received"
                )

            except Exception as e:
                self.get_logger().error(f"General error: {e}")

    def parameter_callback(self, params):
        result = SetParametersResult(successful=True)

        for param in params:
            if param.name == "beam_range_threshold":
                self.beam_range_threshold = param.value
                self.get_logger().info(
                    f"Updated beam range threshold to: {self.beam_range_threshold}"
                )

        return result

    def decode_status(self, status_int):
        # Run diagnostics on dvl sensor STAT codes
        # Codes from https://support.nortekgroup.com/hc/en-us/article_attachments/19558106638620
        if status_int & 0x01:
            # Scaling: If 0, velocity is mm/s. If 1, velocity is m/s.
            self.get_logger().info("Bottom track lost! Velocity is lost")

        if status_int & 0x02:
            # Pressure: 1 if there is a pressure sensor hardware error.
            self.get_logger().info("Pressure sensor error detected")

        if status_int & 0x20:
            # Bottom Track: 0 if OK, 1 if the DVL cannot see the floor.
            self.get_logger().info("Speed of sound (acoustic calculation) error detected")

        if status_int & 0x40:
            # Status: 0 if OK, 1 if a general system fault occurred.
            self.get_logger().info("General error")


def main(args=None):
    rclpy.init(args=args)
    node = DvlDriverNode()
    node.udp_server()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
