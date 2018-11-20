# Usage
## COMS 4733 Lab 4: Rapidly-exploring Random Tree 

There are two python programs, `rrt.py` (which implements a part 1 of the lab) and `bi_rrt.py` (which implements a bidirecitonal RRT for part 2). 

Both scripts accept the same input arguments in the following form:
`python rrt.py <path_to_obstacle_file> <path_to_start_and_goal_file> <step_length> <max_iters>`

* `path_to_obstacle_file` - path to a txt file containing obstacle information
* `path_to_start_and_goal_file` - path to a txt file containing start and goal position information
* `step_length` - the step length that the RRT will use to pick q-new
* `max_iters` - the maximum number of iterations that the RRT will run for before raising an exception

Here are examples for both scripts:
`python rrt.py world_obstacles.txt start_goal.txt 30 5000`
`python bi_rrt.py world_obstacles.txt start_goal.txt 30 5000`

# Method

`rrt.py` grows a rapidly-exploring random tree according to the algorithm presented in lecture. The map is stochastically sampled to find q-rand,
and the closest node in the tree is extended towards q-rand by step length to add q-new to the tree (if it is collision free). The program terminates
once the goal position has been added to the tree.

`bi_rrt.py` is similar, except that it grows two rapidly-exploring random trees simultaneously-- one from the start position and one from the goal position.
The program terminates once the two trees are connected.

Reference used in logic for checking edge intersections: [https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/](https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/)

# Video

The YouTube playlist for working demos of lab 4 is available at: [https://www.youtube.com/playlist?list=PLci6iiGYCad9ycoxPWWC22UXb-GrvrQEj](https://www.youtube.com/playlist?list=PLci6iiGYCad9ycoxPWWC22UXb-GrvrQEj)
