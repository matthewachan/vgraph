# VGRAPH
> Visibility Graph path planning for mobile base robots

VGRAPH takes two config files as input (specifying robot start and goal positions + obstacle locations) and builds a [visibility graph](https://en.wikipedia.org/wiki/Visibility_graph) which the robot uses to plan a path from the start to the goal.

The process for VGRAPH is as follows:
1. Convex hulls of each obstacle are computed
2. Each convex hull is grown by the size of the robot
3. Each corner of the grown hulls is a vertex in the graph and *n(n-1)* edges are drawn between the *n* vertices
4. Edges that intersect with the grown convex hulls (not including collinear intersections) are eliminated
5. [Djikstra's algorithm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm) is used to find the "shortest" path from the start to the goal (each of the edges in the graph represents a valid path)

![](etc/screencap.gif)

## Installation

[ROS Indigo](http://wiki.ros.org/indigo) and [rViz](http://wiki.ros.org/rviz) are required to run the simulation.

Installation instructions can be found [here](http://wiki.ros.org/indigo/Installation/Ubuntu).

## Usage

Start the VGRAPH TurtleBot simulation with the following commands (executed in different terminals):

* `roslaunch launch.launch`

This should start rViz and run src/main.py, which contains all of the logic for building the VGRAPH and moving the robot.

## Documentation 

`main.py` contains a class `Vgraph` that creates a ROS node which publishes to the /cmd\_vel topic and the /vgraph\_markerarr topic

This class has several key methods that I will mention here:

* `__init__(self)`
* `draw_hulls(self, grown_obstacles, marker_arr)`
* `draw_graph(self, marker_arr)`
* `draw_path(self, marker_arr)`

My implementation also uses some code snippets from the `odom_out_and_back.py` script in ROS By Example to get odometry information about the robot.

### \_\_init\_\_

The `__init__` function sets up the ROS node and publishes to cmd\_vel and vgraph\_markerarr topics. It uses other functions to build to VGRAPH, uses Djikstra's Algorithm to find the shortest path in the VGRAPH, and controls the robot to move it along the shortest path to the goal.

### draw\_hulls(self, grown\_obstacles, marker\_arr)

Given a 3D list of x and y coordinates corresponding to all grown obstacles, this function computes the convex hull for each obstacle and draws it in rViz using Markers.

### draw\_graph(self, marker\_arr)

This function draws the VGRAPH in rViz by taking all of the vertices of each convex hull and connecting them to each other, if the resulting edge does not intersect with the edges of a convex hull on the graph.	

Reference used in logic for checking edge intersections: [https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/](https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/)

### draw\_path(self, marker\_arr)

This function runs Djikstra's Algorithm on all of the edges in the VGRAPH to find the shortest path and draws the edges the make up the shortest path in rViz.

Return: A 2D array of vertices along the shortest path

## Meta

**Author**: [Matthew Chan](https://github.com/matthewachan)

Distributed under the GNU GPL v3.0 license. See ``LICENSE`` for more information.

## Contributing

1. [Fork](https://github.com/matthewachan/vgraph/fork) the repo
2. Create a feature branch (e.g. `git checkout -b feature/new_feature`)
3. Add & commit your changes
4. Push to your feature branch (e.g. `git push origin feature/new_feature`)
5. Create a new pull request

## Video

The YouTube video for a working demo of vgraph is available at: [https://youtu.be/9vp4PJip-Vs](https://youtu.be/9vp4PJip-Vs)
