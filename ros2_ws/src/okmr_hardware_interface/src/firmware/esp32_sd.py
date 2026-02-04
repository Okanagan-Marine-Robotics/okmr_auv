#!/usr/bin/env python3
"""
ESP32 Serial Driver
Interactive tool for sending commands to ESP32 via serial port.
Format: index<value
"""

import serial
import time
import sys
import argparse


class ESP32SerialDriver:
    def __init__(self, serial_port="/dev/ttyUSB0", baud_rate=115200):
        """Initialize serial connection to ESP32"""
        self.serial_port_path = serial_port
        self.baud_rate = baud_rate
        self.serial_port = None
        self.serial_buffer = ""

        try:
            self.serial_port = serial.Serial(
                self.serial_port_path,
                self.baud_rate,
                timeout=1
            )
            print(f"Serial port {self.serial_port_path} opened successfully at {self.baud_rate} baud")
            time.sleep(2)
            self.serial_port.reset_input_buffer()
            print("Ready to send commands")
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            sys.exit(1)

    def send_command(self, index, value):
        """Send command to ESP32 in format: index<value"""
        if not self.serial_port or not self.serial_port.is_open:
            print("Error: Serial port is not open")
            return False

        try:
            serial_msg = f"{index}<{value}\n"
            self.serial_port.write(serial_msg.encode("utf-8"))
            print(f"Sent: {serial_msg.strip()}")
            return True
        except serial.SerialException as e:
            print(f"Error writing to serial port: {e}")
            return False

    def read_serial(self):
        """Read and process incoming serial data"""
        if not self.serial_port or not self.serial_port.is_open:
            return

        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.in_waiting).decode("utf-8", errors="ignore")
                self.serial_buffer += data

                while "\n" in self.serial_buffer:
                    line, self.serial_buffer = self.serial_buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        self.process_line(line)

        except serial.SerialException as e:
            print(f"Error reading from serial port: {e}")
        except Exception as e:
            print(f"Unexpected error reading serial: {e}")
            self.serial_buffer = ""
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.reset_input_buffer()

    def process_line(self, line):
        """Process incoming serial line"""
        if line and "<" in line:
            try:
                parts = line.split("<")
                if len(parts) == 2:
                    index = parts[0]
                    value = parts[1]
                    print(f"Received: {index} = {value}")
            except Exception as e:
                print(f"Error parsing line '{line}': {e}")
        elif line:
            print(f"Received: {line}")

    def interactive_mode(self):
        """Interactive mode for sending commands"""
        print("\n=== ESP32 Interactive Mode ===")
        print("Commands:")
        print("  <index> <value>  - Send command (e.g., '200 1' or '100 1800')")
        print("  q                - Quit")
        print("================================\n")

        try:
            while True:
                self.read_serial()

                try:
                    cmd = input("> ").strip()
                    if not cmd:
                        continue

                    parts = cmd.split()

                    if parts[0].lower() == 'q':
                        break
                    elif len(parts) == 2:
                        index = int(parts[0])
                        value = float(parts[1])
                        self.send_command(index, value)
                    else:
                        print("Invalid command. Use: <index> <value> or q")

                except EOFError:
                    break
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"Error: {e}")

        finally:
            self.close()

    def close(self):
        """Close serial connection"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed")


def main():
    parser = argparse.ArgumentParser(description="ESP32 Serial Driver - Interactive Mode")
    parser.add_argument(
        "-p", "--port",
        default="/dev/ttyUSB0",
        help="Serial port (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "-b", "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )

    args = parser.parse_args()

    driver = ESP32SerialDriver(args.port, args.baud)

    try:
        driver.interactive_mode()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
