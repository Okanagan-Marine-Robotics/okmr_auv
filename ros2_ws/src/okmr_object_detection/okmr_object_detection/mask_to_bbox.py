import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from okmr_msgs.msg import BoundingBox

class MaskToBboxNode(Node):
    def __init__(self):
        super().__init__('mask_to_bbox')
        self.subscription = self.create_subscription(Image, "/mask", self.mask_callback, 10)
        self.publisher = self.create_publisher(BoundingBox, "/bounding_box", 10)

    def mask_callback(self, msg):
        try:
            mask_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        non_zero_coords = np.nonzero(mask_image)

        #Get coords for top right corner and bottom right corner respectively
        x_min, y_min = min(non_zero_coords[1]), min(non_zero_coords[0])
        x_max,y_max = max(non_zero_coords[1]), max(non_zero_coords[0])

        width = x_max - x_min
        height = y_max - y_min

        #initialize bounding box message
        bbox_msg = BoundingBox()
        bbox_msg.x_coordinate = x_min
        bbox_msg.y_coordinate = y_min
        bbox_msg.box_width = width
        bbox_msg.box_height = height

        self.publisher.publish(bbox_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MaskToBboxNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
