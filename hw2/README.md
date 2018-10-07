# Usage
## Bug 2 TurtleBot Navigation

Start the Bug 2 TurtleBot simulation with the following commands (executed in different terminals):

  * `roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=<path to world file>`

## bug2.py 

  Now run the Python script `bug2.py` to navigate the TurtleBot around obstacles and get from the starting point (0, 0) to the goal point (10, 0) using the Bug 2 algorithm

  * `./bug2.py cmd_vel:=cmd_vel_mux/input/teleop`

# Method

  `bug2.py` contains a class `OutAndBack` that creates a ROS node which both subscribes to the /scan topic and publishes to the /cmd\_vel topic

  This class has four methods:

   * `__init__(self)`
   * `get_odom(self)`
   * `scan_callback(self, msg)`
   * `on_mline(self, current, goal, threshold)`
   * `at_point(self, current, goal, threshold)`
   * `translate(self)`
   * `rotate(self)`
   * `shutdown(self)`

## \_\_init\_\_

   The `\_\_init\_\_` function sets up the ROS node and publishes to cmd\_vel in order to navigate the robot.

## get\_odom

   This function retrieves odometry data on the robot.

## scan\_callback

   This function retrieves messages containing sensor data from the robot's laser scanner.

## on\_mline

    This helper function checks whether the robot is currently on the m-line

## at\_point

    This helper function checks whether the robot is at a specific point in space

## translate

   This helper function translates the robot a certain distance by publishing data to /cmd\_vel. It also runs checks to see what state the robot is in.

## rotate

   This helper function rotates the robot a certain number of degrees by publishing data to /cmd\_vel.

## shutdown

   Cleans up the robot by stopping any rotation / translation and logging to the console.

# Video

   The YouTube playlist link for all of the working demos from lab 2 are vailable at: [https://www.youtube.com/playlist?list=PLci6iiGYCad9-raUSciLgkWnBQ4kDFVyc](https://www.youtube.com/playlist?list=PLci6iiGYCad9-raUSciLgkWnBQ4kDFVyc)
