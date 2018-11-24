#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy

class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow('window', 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Apply bitmask for yellow colors captured on camera
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([20, 115, 170])
        upper_yellow = numpy.array([40, 255, 190])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(image, image, mask=mask)

        # Retain only 20 pixel rows of masked image data
        h, w, d = image.shape
        search_top = 3 * h / 4
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # Plot red circle on the center of the line
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            red = (0, 0, 255)
            cv2.circle(image, (cx, cy), 20, red, -1)
            

        cv2.imshow('window', image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
