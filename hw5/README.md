# COMS 4733 Lab 5: Autonomous Driving (Followbot)
## Usage

There are 4 Python scripts in this repository, corresponding with each part of the lab.

Below are the commands required to run each part of the lab.

### Commands
To launch turtlebot and map for part 1
```
roslaunch followbot launch.launch

# Run this in a another terminal to control Followbot
python part1.py
```

To launch turtlebot and map for part 2
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world

# Run this in a another terminal to control Followbot
python part2.py
```

To launch turtlebot and map for part 3
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world

# Run this in a another terminal to control Followbot
python part3.py
```

To launch the map for extra credit
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=extra.world

# Run this in a another terminal to control Followbot
python part4.py
```

## Method

### Line Following

The logic for line following was derived from the Programming Robots with ROS example code. By applying a yellow color filter,
we can get a binary image of the road. Then, we can isolate the part of the image directly in front of the robot by
zeroing out pixels that are outside of the boundary we choose.

Finally, we can adjust the robot's rotational velocity proportionally to the distance between the center of the road and the
middle of it's camera to make it follow the road.

This logic is contained in the `image_callback` method of the Followbot class.

### Turning

The logic for turning in my implementation was the same for all parts. When a left turn symbol is detected, a certain fraction
of pixel values on the right half of the yellow color filter are zeroed out for a duration of time (using Timers). This 
essentially makes the robot blind to the right path of the intersection, so it follows the left path. 

The same logic applies to right turns.

The code for this is contained in `image_callback` and a few other Timer callback functions (i.e. `left_callback`, `right_callback`,
etc.)

### Detection

The logic for detection in my implementation changed depending on the map.

For the color-based turning map, I applied a blue, green and red color filter (in addition to the yellow one). When the sum
of all of the blue, green or red pixel values in the color map exceeds a certain threshold, the corresponding stop/turn
instruction is executed.

For the shape-based turning map, I saved images of the shapes on the road taken from the robot's camera. Then, when the robot
encounters a shape (i.e. if it's yellow or red color filter is above a certain threshold), it compares the current image
to the previously saved images. If the error between the images is below a certain threshold, it executes the instruction
corresponding with that image.

The code for this is contained in the `image_callback` function of the Followbot class.

## References

Followbot code used in this project was derived from examples in Programming Robots with ROS by Morgan Quigley, Brian Gerkey and William Smart.

## Video

The YouTube playlist for working demos of lab 5 is available at: [https://www.youtube.com/playlist?list=PLci6iiGYCad8RwVaezV-M4lH-F4jrUCEm](https://www.youtube.com/playlist?list=PLci6iiGYCad8RwVaezV-M4lH-F4jrUCEm)
