#!/usr/bin/env python3

import socket
import re
import rclpy
from rclpy.node import Node
from okmr_msgs.msg import Dvl
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import FluidPressure


class DvlDriverNode(Node):
    def __init__(self):
        super().__init__("dvl_driver")
        self.dvl_publisher = self.create_publisher(Dvl, "/dvl", 10)

        # New implementation using beam "jump" threshold instead of "range"
        # Instead of "Spatial" filtering, this approach uses "Temporal" filtering 
        # The difference is instead of comparing the state of all 4 beams 
        # We just watch out for a jump threshold (Currently set at 1.0) from SPECIFIC beam ticks - ideal for pool testing (not open water)
        self.declare_parameter("beam_jump_threshold", 1.0)
        self.beam_jump_threshold = (
            self.get_parameter("beam_jump_threshold")
            .get_parameter_value()
            .double_value
        )

        # Store previous velocity values for outlier rejection
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vz = 0.0

        # Store previous beam distances for independent outlier rejection
        self.last_d1 = 0.0
        self.last_d2 = 0.0
        self.last_d3 = 0.0
        self.last_d4 = 0.0

    def udp_server(self):
        # Create a UDP socket
        sock_bottom = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind the socket to the host and port
        sock_bottom.bind(("0.0.0.0", 9010))

        while True:
            # Receive data from the client
            data, dvl_addr = sock_bottom.recvfrom(1024)  # buffer size is 1024 bytes
            data = data.decode("ASCII")

            # Regular expression to match the PNORBT8 message format
            pattern = r"\$PNORBT8,TIME=(-?\d+\.?\d*),DT1=(-?\d+\.\d+),DT2=(-?\d+\.\d+),VX=(-?\d+\.\d+),VY=(-?\d+\.\d+),VZ=(-?\d+\.\d+),FOM=(-?\d+\.\d+),D1=(-?\d+\.\d+),D2=(-?\d+\.\d+),D3=(-?\d+\.\d+),D4=(-?\d+\.\d+),BATT=(-?\d+\.\d+),SS=(-?\d+\.\d+),PRESS=(-?\d+\.\d+),TEMP=(-?\d+\.\d+),STAT=(0x[0-9A-Fa-f]+)"

            # Find the matches
            matches = re.search(pattern, data)

            if matches:
                # Extracting values from regex groups
                time_val = float(matches.group(1))
                dt1 = float(matches.group(2))
                dt2 = float(matches.group(3))
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

                # -- Temporal Implementation (DVL Independent Beam Distance Validation) --
                
                validated_distances = []
                current_beams = [d1, d2, d3, d4]
                last_beams = [self.last_d1, self.last_d2, self.last_d3, self.last_d4]
                beam_names = ["D1", "D2", "D3", "D4"]

                for current_d, last_d, b_name in zip(current_beams, last_beams, beam_names):
                    # Initialize first values then check for any spikes
                    if last_d != 0.0 and abs(current_d - last_d) > self.beam_jump_threshold:
                        self.get_logger().debug(f"DVL {b_name} jump too large. Revert to previous: {last_d:.3f} m")
                        validated_distances.append(last_d)
                    else:
                        validated_distances.append(current_d)

                # Unpack validated distances to class variables and message variables
                self.last_d1, self.last_d2, self.last_d3, self.last_d4 = validated_distances
                d1, d2, d3, d4 = validated_distances
                # ---------------------------------------------------------

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

                # Set validated beam distances
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

            else:
                self.get_logger().debug(
                    f"DVL packet parsing failed - no regex matches found"
                )


def main(args=None):
    rclpy.init(args=args)
    node = DvlDriverNode()
    node.udp_server()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
