# Usage
## rbx1

Start the rbx1 TurtleBot simulation with the following commands (in different terminals):

  * `roscore`
  * `roslaunch rbx1_bringup fake_turtlebot.launch`
  * ``rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz``

## timed\_out\_and\_back.py

  Now run the Python script `timed_out_and_back.py` to translate and rotate the robot.

  * `rosrun rbx1_nav timed_out_and_back.py`

# Method

  `timed_out_and_back.py` contains a class `OutAndBack` that creates a ROS node + publisher to the /cmd\_vel topic.

  This class has four methods:

   * `__init__(self)`
   * `translate(self)`
   * `rotate(self)`
   * `shutdown(self)`

## __init__

   The `__init__` function sets up the ROS node and requests user input.

## translate

   This function gets user input on how far to translate the robot and publishes the corresponding data to /cmd_vel.

## rotate

   This function gets user input on how far to rotate the robot and publishes the corresponding data to /cmd_vel.

## shutdown

   Cleans up the robot by stopping any rotation / translation and logging to the console.

# Video

   The YouTube link for a working demo of lab 1 is available at: [https://youtu.be/b9Ps7DrMPuw](https://youtu.be/b9Ps7DrMPuw)
