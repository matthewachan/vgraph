#!/usr/bin/env python

# ROS libs
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import math
from scipy.spatial import ConvexHull # Lib for computing convex hulls
import traceback # For debugging


'''
	Function to load a list of obstacles.
	The obstacle txt file show points in clockwise order

	Return:
		3d list [[[1, 2], [3, 4], [5, 6]], 
						[[7, 8], [9, 10], [10, 11]]]
'''	
def load_obstacles(object_path):
	
	obstacles = []
	obstacle = []
	with open(object_path) as f:
		numObstacles = int(f.readline())
		coordinates = int(f.readline())
		for i in range(coordinates):
			line = f.readline()
			obstacle.append(list(map(int, line.strip().split(' '))))
		for line in f:
			coordinates = list(map(int, line.strip().split(' ')))
			if len(coordinates) == 1:
				obstacles.append(obstacle)
				obstacle = []
			else:
				obstacle.append(coordinates)
	obstacles.append(obstacle)
	assert len(obstacles)==numObstacles, "number of obstacles does not match the first line"
	return obstacles

'''
	Function to load the goal position.

	Return:
		2d list [x, y]
'''	
def load_goal(goal_path):
	with open(goal_path) as f:
		line = f.readline()
		goal = list(map(int, line.strip().split(' ')))
	return goal


'''
        Function that takes an array of obstacle vertices [[[x1, y1], [x2, y2]], [[x3, y3], 
        [x4, y4]], ...] and grows them by the size of the robot.

        This assumes that the robots origin is placed at the center of a 36x36cm square.

        Return: 
                3d list of grown obstacle points
'''
def grow_obstacles(obstacles):
        grown_obstacles = []
        grown_obstacle = []

        robot_size = 36
        growth_factor = robot_size / 2
        num_obstacles = len(obstacles)
        for obstacle in obstacles:
            for coordinate in obstacle:
                c1 = [coordinate[0] - growth_factor, coordinate[1] + growth_factor]
                c2 = [coordinate[0] + growth_factor, coordinate[1] + growth_factor]
                c3 = [coordinate[0] - growth_factor, coordinate[1] - growth_factor]
                c4 = [coordinate[0] + growth_factor, coordinate[1] - growth_factor]
                grown_obstacle.append(c1)
                grown_obstacle.append(c2)
                grown_obstacle.append(c3)
                grown_obstacle.append(c4)
            grown_obstacles.append(grown_obstacle)
            grown_obstacle = []
        return np.asarray(grown_obstacles)

'''
        Utility function for initializing Markers

        Return:
                A marker with basic configuration parameters initialized
'''
def init_marker(marker_id, marker_type):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.id = marker_id
        m.ns = "ns1"
        m.action = Marker.ADD;
        m.pose.orientation.w = 1.0

        m.type = marker_type
        m.scale.x = 0.01
        m.scale.y = 0.01
        m.color.g = 1.0
        m.color.a = 1.0
        return m
        
'''
        Utility function to check if two 2D points are equal

        Return:
                True if they are equal, False otherwise
'''
def verts_equal(v1, v2):
    return (v1.x == v2.x and v1.y == v2.y) 

'''
        Utility function to check the orientation of three points.

        Return:
                0 if co-linear, 1 if CW, 2 if CCW
'''
def orientation(a, b, c):
    val = (a.y - b.y) * (c.x - b.x) - (a.x - b.x) * (c.y - b.y)
    
    if val == 0:
        return 0
    elif val > 0:
        return 1
    return 2

'''
        Utility function to check if two edges intersect

        Return:
                True if they do intersect, otherwise False
'''
def has_intersect(e1, e2):
    a1 = e1[0]
    a2 = e1[1]
    b1 = e2[0]
    b2 = e2[1]
    o1 = orientation(a1, a2, b1)
    o2 = orientation(a1, a2, b2)
    o3 = orientation(b1, b2, a1)
    o4 = orientation(b1, b2, a2)

    # If the two edges share a vertex, they are not colliding
    if (verts_equal(a1, b1) or verts_equal(a1, b2) or verts_equal(a2, b1) or verts_equal(a2, b2)):
        return False
    if (o1 != o2 and o3 != o4):
        return True
    return False

'''
        Utility function for Djikstra's algorithm to compute distance between
        two points that form an edge.

        Return:
            The distance between the ends of the edge.
'''
def compute_weight(e):
	a = e[0]
	b = e[1]
	return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)



class Vgraph():
    def __init__(self):
        # Initialize the VGRAPH node
        rospy.init_node('vgraph', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        # Create the publisher nodes
        self.marker_pub = rospy.Publisher('vgraph_markerarr', MarkerArray, queue_size=10)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
	
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

	obstacles = load_obstacles("../data/world_obstacles.txt")
        grown_obstacles = grow_obstacles(obstacles)
	self.goal = load_goal("../data/goal.txt")
        self.goal = Point(self.goal[0] / scale_factor, self.goal[1] / scale_factor, 0)

        ''' Initialize constants '''
        marker_arr = MarkerArray()
        self.marker_id = 0

        # 1m = 100cm
        scale_factor = 100.0

        self.r = rospy.Rate(30)
        
        self.hull_edges = []
        self.hull_verts = [] 
        self.edges = []

	self.start = [0.0, 0.0]
        self.start = Point(self.start[0] / scale_factor, self.start[1] / scale_factor, 0)

	''' Draw convex hulls around obstacles '''
        self.draw_hulls(grown_obstacles, marker_arr)

	''' Draw paths between vertices '''
        self.draw_graph(marker_arr)
        
        ''' Djikstra's algortihm '''
        waypoints = self.draw_path(marker_arr)

        ''' Rotate and translate the robot to reach each waypoint on the 
        shortest path to the goal '''
	# Set the rotation speed in radians per second
        self.angular_speed = 0.5
        
        # Set the angular tolerance in degrees converted to radians
        self.angular_tolerance = math.radians(0.1)

	# Set the forward linear speed to 0.15 meters per second 
        self.linear_speed = 0.15

        for i in xrange(0, len(waypoints)):
            target = waypoints[len(waypoints) - i - 1]
            (position, rotation) = self.get_odom()

            goal_angle = math.atan2(target.y - position.y, target.x - position.x)
            goal_angle = rotation - goal_angle
            
            self.rotate(goal_angle)

            # Get the starting position values     
            (position, rotation) = self.get_odom()
            
            goal_distance = math.sqrt(math.pow((target.x - position.x), 2) + 
                    math.pow((target.y - position.y), 2))
            
            self.translate(goal_distance)
        
        
        # Continue showing the VGRAPH
        while (not rospy.is_shutdown()):
            self.marker_pub.publish(marker_arr)
            self.r.sleep()

    '''
        Publish messages to the /cmd_vel topic to rotate the robot until it travels
        the goal distance.
    '''
    def translate(self, goal_distance):
    	# Initialize the movement command
	move_cmd = Twist()

	# Set the movement command to forward motion
	move_cmd.linear.x = self.linear_speed

	# Get the starting position values     
	(position, rotation) = self.get_odom()
		
	x_start = position.x
	y_start = position.y

	# Keep track of the distance traveled
	distance = 0

	# Enter the loop to move along a side
	while distance < goal_distance and not rospy.is_shutdown():
		# Publish the Twist message and sleep 1 cycle         
		self.cmd_vel.publish(move_cmd)

		self.r.sleep()

		# Get the current position
		(position, rotation) = self.get_odom()

		# Compute the Euclidean distance from the start
		distance = math.sqrt(math.pow((position.x - x_start), 2) + 
				math.pow((position.y - y_start), 2))

	# Stop the robot before the rotation
	move_cmd = Twist()
	self.cmd_vel.publish(move_cmd)
	rospy.sleep(1)

    '''
        Publish messages to the /cmd_vel topic to rotate the robot until it reaches
        the goal angle.
    '''
    def rotate(self, goal_angle):
    	move_cmd = Twist()
    	move_cmd.angular.z = self.angular_speed
        if (goal_angle > 0):
            move_cmd.angular.z *= -1
	
	# Track the last angle measured
	(position, rotation) = self.get_odom()
	last_angle = rotation

	# Track how far we have turned
	turn_angle = 0

	while abs(turn_angle + self.angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
		# Publish the Twist message and sleep 1 cycle         
		self.cmd_vel.publish(move_cmd)
		self.r.sleep()

		# Get the current rotation
		(position, rotation) = self.get_odom()

		# Compute the amount of rotation since the last loop
		delta_angle = normalize_angle(rotation - last_angle)

		# Add to the running total
		turn_angle += delta_angle
		last_angle = rotation

	# Stop the robot before the next leg
	move_cmd = Twist()
	self.cmd_vel.publish(move_cmd)
	rospy.sleep(1)

    '''
        Function get the odometry of the robot.
        
        Return:
                A tuple containing a Point() and a rotation (in radians).
    '''
    def get_odom(self):
            # Get the current transform between the odom and base frames
            try:
                (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("TF Exception")
                return

            return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    ''' 
        Add markers to rViz, drawing convex hulls around each obstacle.
    '''
    def draw_hulls(self, grown_obstacles, marker_arr):
        m = init_marker(self.marker_id, Marker.LINE_LIST)
        self.marker_id += 1
        
        points = []
        
        for idx, grown_obstacle in enumerate(grown_obstacles):
            hull = ConvexHull(grown_obstacle)
            num_points = len(hull.vertices)
            vertices = hull.vertices
            verts = []
            for i in xrange(0, num_points):
                # Append current vertex
                p1 = Point()
                p1.x = grown_obstacle[vertices[i]][0] / scale_factor
                p1.y = grown_obstacle[vertices[i]][1] / scale_factor
                p1.z = 0
                # Append next vertex (or start vertex)
                p2 = Point()
                p2.x = 0
                p2.y = 0
                if (i == num_points - 1):
                    p2.x = grown_obstacle[vertices[0]][0] / scale_factor
                    p2.y = grown_obstacle[vertices[0]][1] / scale_factor
                else:
                    p2.x = grown_obstacle[vertices[i + 1]][0] / scale_factor
                    p2.y = grown_obstacle[vertices[i + 1]][1] / scale_factor
                p2.z = 0

                points.append(p1)
                points.append(p2)

                verts.append(p1)
                self.edges.append([p1, p2])
                self.hull_edges.append([p1, p2])
            self.hull_verts.append(verts)
            m.points = points
            marker_arr.markers.append(m)

    '''
        Add markers to the MarkerArray, drawing valid paths that the robot
        can traverse--thus building our VGRAPH
    '''
    def draw_graph(self, marker_arr):
        m = init_marker(self.marker_id, Marker.LINE_LIST)
        self.marker_id += 1
        
        
        points = []
        verts = []
        self.hull_verts.append([self.goal])
        self.hull_verts.append([self.start])
        
        for i in xrange(0, len(self.hull_verts) - 1):
            for j in (xrange(i + 1, len(self.hull_verts))):
                    for v1 in self.hull_verts[i]:
                        for v2 in self.hull_verts[j]:
                            flag = True
                            for e in self.hull_edges:
                                if (has_intersect(e, [v1, v2])):
                                    flag = False
                                    break
                            if (flag):
                                points.append(v1)
                                points.append(v2)
                                verts.append(v1)
                                verts.append(v2)
                                self.edges.append([v1, v2])

        m.points = points
        marker_arr.markers.append(m)

    '''
        Use Djikstra's algorithm to find the shortest path in the graph.
        Add a marker to the MarkerArray to draw the shortest path in rViz.
        
        Return:
            A 2d array containing all vertices on the shortest path
    '''
    def draw_path(self, marker_arr):
	dist = {}
        temp_dist = []
	prev = {}
	Q = []
	
	for vertex in verts:
		dist[vertex] = (float("inf"))
                prev[vertex] = None
		Q.append(vertex)

        start_idx = verts.index(self.start)
        dist[self.start] = 0

        while len(Q) > 0:
            smallest = float("inf")
            smallest_vert = Q[0]
            for vertex in Q:
                if (dist[vertex] < smallest):
                    smallest = dist[vertex]
                    smallest_vert = vertex

            Q.remove(smallest_vert)


            for edge in self.edges:
                new_dist = smallest + compute_weight(edge)
                if (verts_equal(smallest_vert, edge[0])):
                    neighbor = edge[1]
                    if (new_dist < dist[neighbor]):
                        dist[neighbor] = new_dist
                        prev[neighbor] = smallest_vert
                elif (verts_equal(smallest_vert, edge[1])):
                    neighbor = edge[0]
                    if (new_dist < dist[neighbor]):
                        dist[neighbor] = new_dist
                        prev[neighbor] = smallest_vert

        path_edges = []
        current = goal
	path_verts = []
	
        while prev[current] is not None:
            path_edges.append(current)
            path_edges.append(prev[current])
            path_verts.append(current)
            u = prev[current]

        m = init_marker(self.marker_id, Marker.LINE_LIST)
        self.marker_id += 1
        m.points = path_edges
        m.scale.x = .03
        m.scale.y = .03
        m.color.r = 1
        m.color.g = 0
        m.color.b = 0
        marker_arr.markers.append(m)
        
        self.marker_pub.publish(marker_arr)
        self.r.sleep()

        return path_verts

    '''
        Shutdown function that stops the robot and logs info.
    '''
    def shutdown(self):
            # Always stop the robot when shutting down the node.
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel.publish(Twist())
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        Vgraph()
    except:
        rospy.loginfo("Node terminated")
        traceback.print_exc()
