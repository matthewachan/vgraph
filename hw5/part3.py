#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np

H_YELLOW = (20, 40)
H_BLUE = (105, 130)
H_GREEN = (40, 65)
H_RED = (0, 10)
S_LOWER = 115
S_UPPER = 255
V_LOWER = 115
V_UPPER = 190
THRESHOLD = 3000
ERR_THRESH = 100000

class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow('window', 1)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', 
                Twist, queue_size=1)

        self.twist = Twist()
        self.rate = 50
        self.r = rospy.Rate(self.rate)
        self.cmd = ''
        self.timer = 0
        self.duration = rospy.Duration(6)
        self.shutdown = False

        self.left = cv2.imread('./left.jpg')
        self.right = cv2.imread('./right.jpg')
        self.stop = cv2.imread('./stop.jpg')
        self.left = cv2.cvtColor(self.left, cv2.COLOR_BGR2GRAY)
        self.right = cv2.cvtColor(self.right, cv2.COLOR_BGR2GRAY)
        self.stop = cv2.cvtColor(self.stop, cv2.COLOR_BGR2GRAY)


    def timer_callback(self, event):
        self.cmd = ''

    def shutdown_callback(self, event):
        self.shutdown = True

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Apply bitmask for yellow colors captured on camera
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        y_mask = cv2.inRange(hsv, np.array([H_YELLOW[0], S_LOWER, V_LOWER]),
                np.array([H_YELLOW[1], S_UPPER, V_UPPER]))
        # b_mask = cv2.inRange(hsv, np.array([H_BLUE[0], S_LOWER, V_LOWER]),
        #         np.array([H_BLUE[1], S_UPPER, V_UPPER]))
        # g_mask = cv2.inRange(hsv, np.array([H_GREEN[0], S_LOWER, V_LOWER]),
        #         np.array([H_GREEN[1], S_UPPER, V_UPPER]))
        r_mask = cv2.inRange(hsv, np.array([H_RED[0], S_LOWER, V_LOWER]),
                np.array([H_RED[1], S_UPPER, V_UPPER]))
        masks = {
                'y': y_mask,
                # 'b': b_mask,
                # 'g': g_mask,
                'r': r_mask
        }

        y_masked = cv2.bitwise_and(image, image, mask=y_mask)
        # b_masked = cv2.bitwise_and(image, image, mask=b_mask)
        # g_masked = cv2.bitwise_and(image, image, mask=g_mask)
        r_masked = cv2.bitwise_and(image, image, mask=r_mask)

        # Retain only 20 pixel rows of masked image data
        h, w, d = image.shape
        search_top = 3 * h / 4
        search_bot = search_top +60

        for k, mask in masks.iteritems():
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0


        # Check for red markers
        # if np.sum(masks['r'][search_top:search_bot, 0:w]) > THRESHOLD and self.cmd == '':
        if np.sum(masks['r'][search_top:search_bot, 0:w]) / (255 * 3) > THRESHOLD and self.cmd == '':
            rdiff = np.sqrt(cv2.absdiff(self.right, r_mask))
            ldiff = np.sqrt(cv2.absdiff(self.left, r_mask))
            sdiff = np.sqrt(cv2.absdiff(self.stop, r_mask))
            rdiff = cv2.sumElems(rdiff)[0]
            ldiff = cv2.sumElems(ldiff)[0]
            sdiff = cv2.sumElems(sdiff)[0]
            diffs = [rdiff, ldiff, sdiff]
            min_diff = np.amin(diffs)
            # print "min_difff is " + min_diff
            if min_diff < ERR_THRESH:
                if ldiff == min_diff:
                    self.cmd = 'l'
                    self.timer = rospy.Timer(self.duration, self.timer_callback, oneshot=True)
                elif rdiff == min_diff:
                    self.cmd = 'r'
                    self.timer = rospy.Timer(self.duration, self.timer_callback, oneshot=True)
                else:
                    self.cmd = 's'
                    self.timer = rospy.Timer(rospy.Duration(4), self.shutdown_callback, oneshot=True)

        if self.cmd != '':
            mask_w = 5 * w / 12

            if self.cmd == 'r':
                y_mask[search_top:search_bot, 0:mask_w] = 0
            elif self.cmd == 'l':
                y_mask[search_top:search_bot, 1 - mask_w :w] = 0

        # Follow yellow line
        if not self.cmd == 's':
        # if not np.sum(masks['r'][search_top:search_bot, 0:w]) / (255 * 3) > THRESHOLD:
            if np.sum(masks['y'][search_top:search_bot, 0:w]) > THRESHOLD:
                M = cv2.moments(y_mask)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    red = (0, 0, 255)
                    cv2.circle(image, (cx, cy), 20, red, -1)

                    err = cx - w/2

                    self.twist.linear.x = 0.2

                    self.twist.angular.z = -float(err) / 100

                    self.cmd_vel_pub.publish(self.twist)
            # Blindly walk forward if no yellow line found
            else:
                self.twist.linear.x = 0.2

                self.twist.angular.z = 0 
                self.cmd_vel_pub.publish(self.twist)

        # Walk forward for a period of time
        elif self.cmd == 's' and not self.shutdown:
            self.twist.linear.x = 0.2

            self.twist.angular.z = -0.05
            self.cmd_vel_pub.publish(self.twist)


        # cv2.imshow('blue', b_mask)
        # cv2.imshow('green', g_mask)
        cv2.imshow('red', r_mask)
        cv2.imshow('yellow', y_mask)
        cv2.imshow('img', image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
