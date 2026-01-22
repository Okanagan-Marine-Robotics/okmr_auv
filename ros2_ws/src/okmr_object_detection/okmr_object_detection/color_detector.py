import os
import object_detection.detector 
from okmr_object_detection.detector import ObjectDetectorNode 
import rclpy
import numpy as np
import time
import cv2
import numpy as np

class ExampleDetector(ObjectDetectorNode):
    def __init__(self):
        super().__init__(node_name="example_detector")
        #define model here if needed

    def inference(self,rgb,depth):
        tolerance = 10

        # Convert the BGR color space of the image to HSV
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

        # Define the HSV range for detecting blue
        # Lower and Upper ranges for red color in HSV
        lower_red1 = np.array([250, 70, 50])
        upper_red1 = np.array([255, 255, 255])

        lower_red2 = np.array([0, 70, 50])
        upper_red2 = np.array([10, 255, 255])


        # Create a binary mask where blue regions are set to 1 and others to 0
        # Create two masks for the two red ranges
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        

        # Combine the two masks & Normalize to 0 and 1
        binary_mask = cv2.bitwise_or(mask1, mask2) // 255

        # Optional: Display results for testing
        if os.environ.get('DISPLAY'):
            cv2.imshow('Binary Mask', binary_mask * 255)  # Display as 0 and 255
            cv2.imshow('rgbimage', rgb)
        cv2.waitKey(1)  # Display result continuously on one window

        # Return the binary mask for further processing
        # This is the old version of return statement, needs to be rewrite
        return binary_mask.astype(np.float32)

def main(args=None):
    rclpy.init(args=args)
    node = ExampleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

