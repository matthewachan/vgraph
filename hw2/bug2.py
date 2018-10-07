#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)
        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Init publisher and subscriber nodes
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Set the odom frame
        self.odom_frame = '/odom'

        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  

        # Init movement constants
        self.rate = 20
        self.r = rospy.Rate(self.rate)
        self.linear_speed = 0.50
        self.angular_speed = 0.5
        trace_dist = 0.2
        mline_thresh = 0.15
        self.range_ahead = 1
        self.range_right = 1

        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(1.0)


        # Get the starting position values     
        position = Point()
        (position, rotation) = self.get_odom()

        self.goal = Point()
        self.goal.x = 10
        self.goal.y = 0
        self.obstacle = Point()
        self.goal_thresh = 0.1
        rot_thresh = 0.01

        self.impossible = False

        # Loop until we reach the goal
        while (self.impossible == False and self.at_point(position, self.goal, self.goal_thresh) == False and not rospy.is_shutdown()):

            # rospy.loginfo(rotation)
            # Get new position and rotation
            (position, rotation) = self.get_odom()
            # rospy.loginfo("Robot at (" + str(position.x) + ", " + str(position.y) + ")")

            # Check for obstacles
            if (self.range_ahead < 0.8):
                rospy.loginfo("Obstacle encountered.")
                # rospy.loginfo(self.range_right)

                # Stop
                self.cmd_vel.publish(Twist())

                # Store coordinates
                self.obstacle = position

                # Turn left until the rightmost sensor can't sense the obstacle
                rightmost = 0
                for idx, reading in enumerate(self.ranges):
                    if (not math.isnan(reading)):
                        rightmost = idx
                        break
                while ((not math.isnan(self.ranges[rightmost]) and self.ranges[rightmost] < 3) and not rospy.is_shutdown()):
                    move_cmd = Twist()
                    move_cmd.angular.z = 0.5
                    self.cmd_vel.publish(move_cmd)
                    self.r.sleep()


                # Trace contour of the obstacle
                turned_left = False
                self.iterations = 0

                # Move forward a little bit to get off of the m-line
                self.translate(0.5)
                (position, rotation) = self.get_odom()
                while (self.impossible == False and (position.x < self.obstacle.x or not self.on_mline(position, self.goal, self.goal_thresh)) and not rospy.is_shutdown()):

                    self.iterations += 1
                    # rospy.loginfo("Tracing contour of obstacle...")
                    # rospy.loginfo(self.range_right)

                    # Turn right, if rightmost sensor cannot detect anything or detected object is far away
                    if (math.isnan(self.range_right) or self.range_right > 3.0):
                        # rospy.loginfo("[CONTOUR2] Turning right")
                        # rospy.loginfo(self.range_right)

                        self.rotate(-0.9)

                    # Turn left if object to close on righthand side, and move forward
                    if (self.range_right < 1.3 and not rospy.is_shutdown()):
                        # rospy.loginfo("[CONTOUR3] Turning left and moving forward")
                        # rospy.loginfo(self.range_right)

                        # Turn left 
                        turned_left = True
                        self.rotate(0.9)

                    if (turned_left):
                        turned_left = False
                        # Move forward
                        self.translate(0.3)
                        (position, rotation) = self.get_odom()
                        # if (position.x > self.obstacle.x and self.on_mline(position, self.goal, self.goal_thresh)):
                        #         break

                    # Move forward
                    self.translate(0.40)
                    # Update position and rotation
                    (position, rotation) = self.get_odom()
                    # rospy.loginfo("Robot at (" + str(position.x) + ", " + str(position.y) + ")")

            # Orient towards m-line
            # Robot rotated too far right
            while (rotation < -rot_thresh and not rospy.is_shutdown()):
                move_cmd = Twist()
                move_cmd.angular.z = 0.5
                self.cmd_vel.publish(move_cmd)
                self.r.sleep()
                (position, rotation) = self.get_odom()
            # Robot rotated too far left
            while (rotation > rot_thresh and not rospy.is_shutdown()):
                move_cmd = Twist()
                move_cmd.angular.z = -0.5
                self.cmd_vel.publish(move_cmd)
                self.r.sleep()
                (position, rotation) = self.get_odom()

            move_cmd = Twist()
            move_cmd.linear.x = self.linear_speed
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()



        # Stop the robot for good
        self.cmd_vel.publish(Twist())

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[len(msg.ranges)/2]
        self.range_right = msg.ranges[0]
        self.ranges = msg.ranges

    # Check whether the robot is on the m-line
    def on_mline(self, current, goal, threshold):
        if (current.y < goal.y + threshold and current.y > goal.y - threshold):
            return True
        return False

    # Check if the robot is at a point in space
    def at_point(self, current, checkpoint, threshold):
        if (current.x < checkpoint.x + threshold and current.x > checkpoint.x - threshold and current.y < checkpoint.y + threshold and current.y > checkpoint.y - threshold):
            return True
        return False

    # Translate the robot
    def translate(self, goal_distance):
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
            (position, rotation) = self.get_odom()

            # Check if the robot is on the m-line
            if (position.x > self.obstacle.x + 0.5 and self.on_mline(position, self.goal, self.goal_thresh)):
                # rospy.loginfo("Breaking translation")
                break
            # Check if the robot already reached this point
            elif (self.iterations > 5 and self.at_point(position, self.obstacle, 0.3)):
                rospy.loginfo("No solution possible")
                self.impossible = True
                break

            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        # Stop the robot
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

    # Rotate the robot
    def rotate(self, goal_angle):
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
