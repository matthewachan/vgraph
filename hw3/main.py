import rospy
import traceback
import numpy as np
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import ConvexHull
import math


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

def load_goal(goal_path):
	with open(goal_path) as f:
		line = f.readline()
		goal = list(map(int, line.strip().split(' ')))
	return goal


def grow_obstacles(obstacles):
        grown_obstacles = []
        grown_obstacle = []

        aabb_sidelen = 36
        half = aabb_sidelen / 2
        num_obstacles = len(obstacles)
        for obstacle in obstacles:
            for coordinate in obstacle:
                c1 = [coordinate[0] - half, coordinate[1] + half]
                c2 = [coordinate[0] + half, coordinate[1] + half]
                c3 = [coordinate[0] - half, coordinate[1] - half]
                c4 = [coordinate[0] + half, coordinate[1] - half]
                grown_obstacle.append(c1)
                grown_obstacle.append(c2)
                grown_obstacle.append(c3)
                grown_obstacle.append(c4)
            grown_obstacles.append(grown_obstacle)
            grown_obstacle = []
        return np.asarray(grown_obstacles)


def init_marker(marker_id, marker_type):
        m = Marker()
        m.header.frame_id = 'base_link'
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
        
def vertsEqual(v1, v2):
    return (v1.x == v2.x and v1.y == v2.y) 

def orientation(a, b, c):
    val = (a.y - b.y) * (c.x - b.x) - (a.x - b.x) * (c.y - b.y)
    
    if val == 0:
        return 0
    elif val > 0:
        return 1
    return 2

def do_intersect(e1, e2):
    a1 = e1[0]
    a2 = e1[1]
    b1 = e2[0]
    b2 = e2[1]
    o1 = orientation(a1, a2, b1)
    o2 = orientation(a1, a2, b2)
    o3 = orientation(b1, b2, a1)
    o4 = orientation(b1, b2, a2)

    if (vertsEqual(a1, b1) or vertsEqual(a1, b2) or vertsEqual(a2, b1) or vertsEqual(a2, b2)):
        return False
    if (o1 != o2 and o3 != o4):
        return True
    return False

def compute_weight(e):
	a = e[0]
	b = e[1]
	return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)



class Vgraph():
    def __init__(self):
        rospy.init_node('vgraph', anonymous=False)
        
        self.marker_pub = rospy.Publisher('vgraph_markerarr', MarkerArray, queue_size=10)
        r = rospy.Rate(30)

        aabb_sidelen = 36
        half = aabb_sidelen / 2
        scale_factor = 100.0

	obstacles = load_obstacles("../data/world_obstacles.txt")
        grown_obstacles = grow_obstacles(obstacles)
	goal = load_goal("../data/goal.txt")
	start = [0.0, 0.0]

        marker_arr = MarkerArray()
        marker_id = 0
        
        hull_edges = []
        hull_verts = [] 
        edges = []
	
	# Draw convex hulls around obstacles
        m = init_marker(marker_id, Marker.LINE_LIST)
        marker_id += 1
        
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
                hull_edges.append([p1, p2])
            hull_verts.append(verts)


        m.points = points
        marker_arr.markers.append(m)

	# Draw paths
        m = init_marker(marker_id, Marker.LINE_LIST)
        marker_id += 1
        
        
        points = []
        verts = []
        start = Point(start[0] / scale_factor, start[1] / scale_factor, 0)
        goal = Point(goal[0] / scale_factor, goal[1] / scale_factor, 0)
        hull_verts.append([goal])
        hull_verts.append([start])
        
        for i in xrange(0, len(hull_verts) - 1):
            for j in (xrange(i + 1, len(hull_verts))):
                    for v1 in hull_verts[i]:
                        for v2 in hull_verts[j]:
                            flag = True
                            for e in hull_edges:
                                if (do_intersect(e, [v1, v2])):
                                    flag = False
                                    break
                            if (flag):
                                points.append(v1)
                                points.append(v2)
                                verts.append(v1)
                                verts.append(v2)
                                edges.append([v1, v2])
                    
                        

        m.points = points
        marker_arr.markers.append(m)
        
        
        # Compute weights on each edge
        weights = []
        edges = np.asarray(edges)
        for edge in edges:
        	weights.append(compute_weight(edge))

	dist = {}
        temp_dist = []
	prev = {}
	Q = []
	
	for vertex in verts:
		dist[vertex] = (float("inf"))
                prev[vertex] = None
		Q.append(vertex)

        start_idx = verts.index(start)
        dist[start] = 0

        while len(Q) > 0:
            smallest = float("inf")
            smallest_vert = Q[0]
            for vertex in Q:
                if (dist[vertex] < smallest):
                    smallest = dist[vertex]
                    smallest_vert = vertex

            Q.remove(smallest_vert)

            # rospy.loginfo("Visiting ")
            # rospy.loginfo(smallest_vert)

            for edge in edges:
                new_dist = smallest + compute_weight(edge)
                # rospy.loginfo(edge[0])
                # rospy.loginfo(hull_verts[min_idx])
                if (vertsEqual(smallest_vert, edge[0])):
                    neighbor = edge[1]
                    if (new_dist < dist[neighbor]):
                        dist[neighbor] = new_dist
                        prev[neighbor] = smallest_vert
                elif (vertsEqual(smallest_vert,  edge[1])):
                    neighbor = edge[0]
                    if (new_dist < dist[neighbor]):
                        dist[neighbor] = new_dist
                        prev[neighbor] = smallest_vert

            


        rospy.loginfo("Djikstra's complete!")

        S = []
        u = goal

        while prev[u] is not None:
            S.append(u)
            S.append(prev[u])
            u = prev[u]
            # if (vertsEqual(u, start)):
            #     break

        m = init_marker(marker_id, Marker.LINE_LIST)
        marker_id += 1
        m.points = S
        m.scale.x = .1
        m.scale.y = .1
        m.color.r = 1
        m.color.g = 0
        m.color.b = 0
        marker_arr.markers.append(m)
        rospy.loginfo("Retracing complete!")
        rospy.loginfo(len(S))
        while (not rospy.is_shutdown()):
            self.marker_pub.publish(marker_arr)
            r.sleep()


if __name__ == '__main__':
    try:
        Vgraph()
    except:
        rospy.loginfo("Node terminated")
        traceback.print_exc()
        
