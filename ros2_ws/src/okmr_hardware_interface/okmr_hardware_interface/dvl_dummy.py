import socket
import re
import rclpy
import time
from rclpy.node import Node
from okmr_msgs.msg import SensorReading

class DvlDriverNode(Node):
    def __init__(self):
        super().__init__('dvl_dummy_driver')
        self.pidPublisherMap={}

        self.pidPublisherMap["surge"] = self.create_publisher(SensorReading, "/PID/surge/actual", 10)
        self.pidPublisherMap["sway"] = self.create_publisher(SensorReading, "/PID/sway/actual", 10)
        self.pidPublisherMap["heave"] = self.create_publisher(SensorReading, "/PID/heave/actual", 10)

    def udp_server(self):
        while True:
            time.sleep(0.125)
            _time=self.get_clock().now().to_msg()
            surge_msg=SensorReading()
            sway_msg=SensorReading()
            heave_msg=SensorReading()
                
            surge_msg.header.stamp = _time
            sway_msg.header.stamp = _time
            heave_msg.header.stamp = _time
                
            surge_msg.data = 0.0
            sway_msg.data=0.0
            heave_msg.data=0.0

            self.pidPublisherMap["surge"].publish(surge_msg)
            self.pidPublisherMap["sway"].publish(sway_msg)
            self.pidPublisherMap["heave"].publish(heave_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DvlDriverNode()
    node.udp_server()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
