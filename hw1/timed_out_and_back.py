#!/usr/bin/env python

""" timed_out_and_back.py - Version 1.2 2014-12-14

    A basic demo of the using odometry data to move the robot along
    and out-and-back trajectory.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import rospy
from geometry_msgs.msg import Twist
from math import pi

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # How fast will we update the robot's movement?
        self.rate = 50

        # Set the forward linear speed to 0.2 meters per second 
        self.linear_speed = 0.2

        # Set the rotation speed to 1.0 radians per second
        self.angular_speed = 1.0

        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(self.rate)

        # State flag for when to quit the program
        quit = False


        while (quit != True):
            # Get user input 
            user_input = str(raw_input('(T)ranslate, (R)otate, (Q)uit: '))
            # Quit program 
            if (user_input == 'Q'):
                quit = True
            # Translate the robot a certain distance 
            elif (user_input == 'T'):
                self.translate()
            # Rotate the robot by a certain angle
            elif (user_input == 'R'):
                self.rotate()
            # Invalid input, prompt user to retry
            else:
                print('Invalid command, please try again.')

        # Stop the robot
        self.cmd_vel.publish(Twist())

    def translate(self):
        # User inputs how far the robot should travel
        goal_distance = input('Enter distance to translate: ')

        move_cmd = Twist()

        # Set the forward speed
        move_cmd.linear.x = self.linear_speed

	# Move robot backwards
	if (goal_distance < 0):
		move_cmd.linear.x *= -1

        # How long should it take us to get there?
        linear_duration = abs(goal_distance / self.linear_speed)
        
        # Move forward for a time to go the desired distance
        ticks = int(linear_duration * self.rate)

        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        # Stop the robot before the rotation
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

    def rotate(self):
        # User inputs how much the robot should rotate
        goal_angle = input('Angle to rotate by: ')

        move_cmd = Twist()

        # Set the angular speed
        move_cmd.angular.z = self.angular_speed

	if (goal_angle < 0):
		move_cmd.angular.z *= -1

        # How long should it take to rotate?
        angular_duration = abs(goal_angle / self.angular_speed)

        # Rotate for a time to go 180 degrees
        ticks = int(angular_duration * self.rate)

        for t in range(ticks):           
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        # Stop the robot before the next leg
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)    

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")

