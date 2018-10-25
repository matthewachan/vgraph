import rospy
import traceback
import numpy as np
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import ConvexHull

def load_goal(goal_path):
	with open(goal_path) as f:
		line = f.readline()
		goal = list(map(int, line.strip().split(' ')))
	return goal
def load_obstacles(object_path):
	'''
	Function to load a list of obstacles.
	The obstacle txt file show points in clockwise order

	Return:
		3d list [[[1, 2], [3, 4], [5, 6]], 
						[[7, 8], [9, 10], [10, 11]]]
	'''
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


def init_marker(marker_id):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = rospy.Time.now()
        m.id = marker_id
        m.ns = "ns1"
        m.action = Marker.ADD;
        m.pose.orientation.w = 1.0

        m.type = Marker.LINE_LIST
        m.scale.x = 0.01
        # m.scale.y = 0.10
        m.color.g = 1.0
        m.color.a = 1.0
        return m

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


def vertsEqual(v1, v2):
    return (v1.x == v2.x and v1.y == v2.y) 

def area(a, b, c):
    return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)
def on_left(a, b, c):
    return (area(a, b, c) > 0)

def on_right(a, b, c):
    return (area(a, b, c) < 0)

def is_collinear(a, b, c):
    return (area(a, b, c) < threshold and area(a, b, c) > -threshold)


def has_intersection(e1, e2):
    a1 = e1[0]
    a2 = e1[1]
    b1 = e2[0]
    b2 = e2[1]
    if (vertsEqual(a1, b1) or vertsEqual(a1, b2) or vertsEqual(a2, b1) or vertsEqual(a2, b2)):
        return False
    if ((on_left(a1, a2, b1) and on_right(a1, a2, b2)) or (on_right(a1, a2, b1) and on_left(a1, a2, b2))):
        # rospy.loginfo("INTERSECTION")
        # rospy.loginfo(e1)
        # rospy.loginfo(e2)

        return True 
    return False 

class Vgraph():
    def __init__(self):
        rospy.init_node('vgraph', anonymous=False)

        self.marker_pub = rospy.Publisher('vgraph_markerarr', MarkerArray, queue_size=10)
        r = rospy.Rate(30)

	obstacles = load_obstacles("../data/world_obstacles.txt")
	goal = load_goal("../data/goal.txt")
	start = [0, 0]

        aabb_sidelen = 36
        half = aabb_sidelen / 2
        scale_factor = 100.0

        grown_obstacles = grow_obstacles(obstacles)

        marker_arr = MarkerArray()
        hull_edges = []
        hull_verts = [] 
        marker_id = 0
        edges = []

        # while (not rospy.is_shutdown()):

        m = init_marker(marker_id)
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

        start = Point(start[0] / scale_factor, start[1] / scale_factor, 0)
        
        goal = Point(goal[0] / scale_factor, goal[1] / scale_factor, 0)
        hull_verts.append([goal])
        hull_verts.append([start])
        m.points = points
        marker_arr.markers.append(m)

        m = init_marker(marker_id)
        marker_id += 1
        points = []
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
                    
                        
        # for v1 in hull_verts:
            # for v2 in hull_verts:
                # if (not vertsEqual(v1, v2)):
                    # flag = True
                    # if (v1 in hull_verts and v2 in hull_verts):
                        # flag = False
                    # # for e in hull_edges:
                    # #     if (has_intersection(e, [v1, v2])):
                    # #         flag = False
                    # #         break
                    # if (flag):
                        # points.append(v1)
                        # points.append(v2)
        m.points = points
        marker_arr.markers.append(m)

        
        # rospy.loginfo(area(Point(1,1,0), Point(2,2,0), Point(1,2,0)))
        # rospy.loginfo(area(Point(1,1,0), Point(2,2,0), Point(2,1,0)))
        # rospy.loginfo(has_intersection([Point(1,1,0), Point(2,2,0)], [Point(1,2,0), Point(2,1,0)]))
        while (not rospy.is_shutdown()):
            self.marker_pub.publish(marker_arr)
            r.sleep()
                   


    

        # while not rospy.is_shutdown():
        #     self.publish_marker()
    # def draw(self, marker_publisher, text):
    #     marker = Marker(
    #             type=Marker.TEXT_VIEW_FACING,
    #             id=0,
    #             lifetime=rospy.Duration(1.5),
    #             pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
    #             scale=Vector3(0.06, 0.06, 0.06),
    #             header=Header(frame_id='base_link'),
    #             color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
    #             text=text)
    #     marker_publisher.publish(marker)


if __name__ == '__main__':
    try:
        Vgraph()
    except:
        rospy.loginfo("Node terminated")
        traceback.print_exc()
        
