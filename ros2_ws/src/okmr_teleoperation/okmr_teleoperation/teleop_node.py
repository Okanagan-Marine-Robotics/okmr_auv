import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from okmr_msgs.msg import SensorReading

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.declare_parameter('yaw_multiplier', 20.0)
        self.declare_parameter('roll_multiplier', 45.0)
        self.declare_parameter('pitch_multiplier', 45.0)

        # Subscribe to the joystick topic
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publisher for motor throttle
        self.surge = self.create_publisher(SensorReading, '/PID/surge/target', 10)
        self.sway = self.create_publisher(SensorReading, '/PID/sway/target', 10)
        self.heave = self.create_publisher(SensorReading, '/PID/heave/target', 10)
        self.yaw = self.create_publisher(SensorReading, '/PID/yaw/target', 10)
        self.roll = self.create_publisher(SensorReading, '/PID/roll/target', 10)
        self.pitch = self.create_publisher(SensorReading, '/PID/pitch/target', 10)

        # Initialize motor values
        self.values = {
            "surge": 0.0,
            "sway": 0.0,
            "yaw": 0.0,
            "heave": 0.0,
            "roll": 0.0,
            "pitch": 0.0
        }

        # Initialize previous timestamp
        self.prev_time = self.get_clock().now()

    def joy_callback(self, msg):
        """Process joystick input and update motor throttle values."""

        # Calculate delta time
        current_time = self.get_clock().now()
        delta_time = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds
        self.prev_time = current_time
        
        # Get parameter values
        yaw_multiplier = self.get_parameter('yaw_multiplier').get_parameter_value().double_value
        roll_multiplier = self.get_parameter('roll_multiplier').get_parameter_value().double_value
        pitch_multiplier = self.get_parameter('pitch_multiplier').get_parameter_value().double_value

        # Joystick Axes: (normalized between -1 and 1)
        surge = msg.axes[1]   # Left stick Y-axis (up/down)
        sway  = msg.axes[0]  # Left stick X-axis (left/right)
        yaw   = msg.axes[2] * yaw_multiplier # Right stick X-axis (left/right)
        
        # Trigger for heave
        heaveUp   = msg.axes[4] # Right trigger resting at 1.0, depressed to -1.0
        heaveDown = msg.axes[5] # Left trigger resting at 1.0
        heave     = heaveDown - heaveUp 
        # D-pad for pitch, roll
        roll  = msg.axes[6] * roll_multiplier# Left is positive
        pitch = msg.axes[7] * pitch_multiplier # Up is positive

        # Button Presses: TODO kill switch button, button for going to the surface, button for leveling out roll pitch, button for firing torpedos, button for enabling arm
        dummy = msg.buttons[0] # A button
        dummy = msg.buttons[1] # B button

        
        # Compute motor values based on joystick input and delta time
        self.values["surge"] = surge
        self.values["sway"] = sway
        self.values["yaw"] += yaw * delta_time
        self.values["heave"] = heave
        self.values["roll"] = roll
        self.values["pitch"] = pitch

        # Wrap yaw angle to [-180, 180] degrees
        if self.values["yaw"] > 180:
            self.values["yaw"] -= 360
        elif self.values["yaw"] < -180:
            self.values["yaw"] += 360
        # Publish the sensor readings
        surge_msg = SensorReading()
        surge_msg.data = self.values["surge"]
        surge_msg.header = Header()
        surge_msg.header.stamp = self.get_clock().now().to_msg()
        self.surge.publish(surge_msg)

        sway_msg = SensorReading()
        sway_msg.data = self.values["sway"]
        sway_msg.header = Header()
        sway_msg.header.stamp = self.get_clock().now().to_msg()
        self.sway.publish(sway_msg)

        heave_msg = SensorReading()
        heave_msg.data = self.values["heave"]
        heave_msg.header = Header()
        heave_msg.header.stamp = self.get_clock().now().to_msg()
        self.heave.publish(heave_msg)

        yaw_msg = SensorReading()
        yaw_msg.data = self.values["yaw"]
        yaw_msg.header = Header()
        yaw_msg.header.stamp = self.get_clock().now().to_msg()
        self.yaw.publish(yaw_msg)

        roll_msg = SensorReading()
        roll_msg.data = self.values["roll"]
        roll_msg.header = Header()
        roll_msg.header.stamp = self.get_clock().now().to_msg()
        self.roll.publish(roll_msg)

        pitch_msg = SensorReading()
        pitch_msg.data = self.values["pitch"]
        pitch_msg.header = Header()
        pitch_msg.header.stamp = self.get_clock().now().to_msg()
        self.pitch.publish(pitch_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
