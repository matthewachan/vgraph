# Usage
## TurtleBot VGRAPH navigation

Start the VGRAPH TurtleBot simulation with the following commands (executed in different terminals):

	* `roslaunch launch.launch`

	This should start rViz and run src/main.py, which contains all of the logic for building the VGRAPH and moving the robot.

# Method

	`main.py` contains a class `Vgraph` that creates a ROS node which publishes to the /cmd\_vel topic and the /vgraph\_markerarr topic

	This class has several key methods that I will mention here:

		* `__init__(self)`
		* `draw_hulls(self, grown_obstacles, marker_arr)`
		* `draw_graph(self, marker_arr)`
		* `draw_path(self, marker_arr)`

		My implementation also uses some code snippets from the odom\_out\_and\_back.py script in ROS By Example to get odometry information about the robot.

## \_\_init\_\_

		The `__init__` function sets up the ROS node and publishes to cmd\_vel and vgraph\_markerarr topics. It uses other functions to build to VGRAPH, uses Djikstra's Algorithm to find the shortest path in the VGRAPH, and controls the robot to move it along the shortest path to the goal.

## draw\_hulls(self, grown\_obstacles, marker\_arr)

		Given a 3d list of x and y coordinates corresponding to all grown obstacles, this function computes the convex hull for each obstacle and draws it in rViz using Markers.

## draw\_graph(self, marker\_arr)

		This function draws the VGRAPH in rViz by taking all of the vertices of each convex hull and connecting them to each other, if the resulting edge does not intersect with the edges of a convex hull on the graph.	

		Reference used in logic for checking edge intersections: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

## draw\_path(self, marker\_arr)

		This function runs Djikstra's Algorithm on all of the edges in the VGRAPH to find the shortest path and draws the edges the make up the shortest path in rViz.

		Return: A 2d array of vertices along the shortest path

# Video

The YouTube video for a working demo of lab 3 is available at: []()
