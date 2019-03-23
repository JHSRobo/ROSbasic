#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class CrackFinder:
    def __init__(self):
        self.bridge = CvBridge()
        # figure out the video feed
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.camera_callback)

        # threshold of error (in pixels) for changing main directions
        self.threshold = 20
        self.y_err = 0
        self.x_err = 0
        self.largest_err = 0
        self.other_err = 0
        self.err_axis = ''
        self.other_axis = ''

    def camera_callback(self, data):
        try:
            # converting from ROS default rgb8 encoding to CV's standard bgr8 encoding
            video_feed = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        # downsampling to 212 x 160 to speed things up
        newsize = (212, 160)
        interpolation = cv2.INTER_NEAREST
        resized_image = cv2.resize(video_feed, newsize, 0, 0, interpolation)

        # convert to hsv for better color thresholding
        hsv_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)

        # filtering image to blue and not blue since only those colors matter
        # since our hue values wrap from 330 to 30, but that's not how being
        # in between numbers works, we gotta
        lower_blue = np.array([60, 60, 330])
        inbw_blue1 = np.array([255, 255, 0])
        inbw_blue2 = np.array([60, 60, 0])
        upper_blue = np.array([255, 255, 30])
        mask1 = cv2.inRange(hsv, lower_red, inbw_red1)
        mask2 = cv2.inRange(hsv, inbw_red2, upper_red)
        mask = mask1 + mask2

        histogram = cv2.calcHist(image=hsv_image, mask=mask)

        cv2.imshow('proceessed_video', mask)

def main():
    while not rospy.is_shutdown():
        rospy.init_node('line_follower')
        crack_finder = CrackFinder()
        rospy.spin()




if __name__ == '__main__':
    main()
