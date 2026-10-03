#!/usr/bin/env python3

import cv2
import numpy as np
import time


from picamera2 import Picamera2
from std_msgs.msg import Bool, Float32


class ColourDetectionNode(Node):

    def __init__(self):
        super().__init__("colour_detection_node")

        # -----------------------------
        # Camera
        # -----------------------------

        self.picam2 = Picamera2()

        config = self.picam2.create_preview_configuration(
            main={
                "format": "RGB888",
                "size": (640, 480)
            }
        )

        self.picam2.configure(config)
        self.picam2.start()

        time.sleep(2)

        # -----------------------------
        # HSV ranges
        # -----------------------------

        self.lower_green = np.array([35, 80, 80])
        self.upper_green = np.array([85, 255, 255])

        self.lower_red1 = np.array([0, 80, 80])
        self.upper_red1 = np.array([10, 255, 255])

        self.lower_red2 = np.array([170, 80, 80])
        self.upper_red2 = np.array([179, 255, 255])

        # -----------------------------
        # ROS publishers
        # -----------------------------

        self.green_pub = self.create_publisher(
            Bool,
            "/camera/green_detected",
            10
        )

        self.red_pub = self.create_publisher(
            Bool,
            "/camera/red_detected",
            10
        )

        self.green_x_pub = self.create_publisher(
            Float32,
            "/camera/green_x",
            10
        )

        self.red_x_pub = self.create_publisher(
            Float32,
            "/camera/red_x",
            10
        )

        # -----------------------------
        # Processing timer
        # -----------------------------

        self.timer = self.create_timer(
            0.05,
            self.process_camera
        )

        self.get_logger().info(
            "Colour detection node started."
        )

    def process_camera(self):

        frame = self.picam2.capture_array()

        # RGB → HSV
        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_RGB2HSV
        )

        # -----------------------------
        # Green
        # -----------------------------

        green_mask = cv2.inRange(
            hsv,
            self.lower_green,
            self.upper_green
        )

        # -----------------------------
        # Red
        # -----------------------------

        red_mask1 = cv2.inRange(
            hsv,
            self.lower_red1,
            self.upper_red1
        )

        red_mask2 = cv2.inRange(
            hsv,
            self.lower_red2,
            self.upper_red2
        )

        red_mask = cv2.bitwise_or(
            red_mask1,
            red_mask2
        )

        # -----------------------------
        # Find largest green object
        # -----------------------------

        green_contours, _ = cv2.findContours(
            green_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        green_detected = False
        green_x = -1.0

        if green_contours:

            largest_green = max(
                green_contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(
                largest_green
            )

            if area > 500:

                x, y, w, h = cv2.boundingRect(
                    largest_green
                )

                green_detected = True
                green_x = x + (w / 2)

        # -----------------------------
        # Find largest red object
        # -----------------------------

        red_contours, _ = cv2.findContours(
            red_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        red_detected = False
        red_x = -1.0

        if red_contours:

            largest_red = max(
                red_contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(
                largest_red
            )

            if area > 500:

                x, y, w, h = cv2.boundingRect(
                    largest_red
                )

                red_detected = True
                red_x = x + (w / 2)

        # -----------------------------
        # Publish
        # -----------------------------

        green_msg = Bool()
        green_msg.data = green_detected
        self.green_pub.publish(green_msg)

        red_msg = Bool()
        red_msg.data = red_detected
        self.red_pub.publish(red_msg)

        green_x_msg = Float32()
        green_x_msg.data = green_x
        self.green_x_pub.publish(green_x_msg)

        red_x_msg = Float32()
        red_x_msg.data = red_x
        self.red_x_pub.publish(red_x_msg)

    def shutdown(self):

        self.picam2.stop()


def main(args=None):
    



if __name__ == "__main__":
    main()
