#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import numpy as np
import baxter_interface

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageCapture():
    """docstring for ImageCapture"""

    def __init__(self, arg):
        self.bridge = CvBridge()

        # for each image it will use the call back !!!! make a flag to capture
        self.image_subscriber = rospy.Subscriber("/cameras/right_hand_camera/image", Image, self.callback)
        self.flagIgnoreImages = false;

    def callback(self, data):
        if not self.flagIgnoreImages:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            (rows, cols, channels) = cv_image.shape

            if cols > 60 and rows > 60:
                # converts to grayscale
                cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY);
                self.flagIgnoreImages = true;

        cv2.imshow("Image window", cv_image_gray)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    def get_the_grey_image(self):
        return self.image_subscriber


class ShapeRecognition:
    def __init__(self, img):
        self.img = img

    def process(self):
        my_lower = np.array([0, 0, 0], dtype=np.uint8)
        my_upper = np.array([100, 100, 100], dtype=np.uint8)
        mask = cv2.inRange(self.img, my_lower, my_upper)
        # cv2.imshow('mask', mask)

        (flags, contours, h) = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # print(contours)
        return contours


def main(args):
    rospy.init_node('capture_image_for_use', anonymous=True)

    ic = ImageCapture()
    grey_image = ic.get_the_grey_image()
    shape_rec = ShapeRecognition(grey_image)
    contours = shape_rec.process()
    print(contours[0][1][0][0])
    print(contours[0][1][0][1])
    if KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)