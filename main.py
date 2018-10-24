import rospy
import traceback
import numpy as np
import tf
from geometry_msgs.msg import Twist, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import ConvexHull

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

class Vgraph():
    def __init__(self):
        rospy.init_node('vgraph', anonymous=False)

        self.marker_pub = rospy.Publisher('vgraph_markerarr', MarkerArray, queue_size=10)
        r = rospy.Rate(30)

	obstacles = load_obstacles("../data/world_obstacles.txt")
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

        grown_obstacles = np.asarray(grown_obstacles)
        # hull = ConvexHull(grown_obstacles[0])
        # rospy.loginfo(hull.vertices)
        scale_factor = 100.0
        marker_arr = MarkerArray()
                

        while (not rospy.is_shutdown()):

            for idx, grown_obstacle in enumerate(grown_obstacles):
                m = Marker()
                m.header.frame_id = 'base_link'
                m.header.stamp = rospy.Time.now()
                m.id = idx
                m.ns = "ns1"
                m.action = Marker.ADD;
                m.pose.orientation.w = 1.0

                m.type = Marker.LINE_LIST
                m.scale.x = 0.10
                m.scale.y = 0.10
                m.color.g = 1.0
                m.color.a = 1.0
                points = []
                hull = ConvexHull(grown_obstacle)
                num_points = len(hull.vertices)
                vertices = hull.vertices
                for i in xrange(0, num_points):
                    # Append current vertex
                    p1 = Point()
                    p1.x = grown_obstacle[vertices[i]][0] / scale_factor
                    p1.y = grown_obstacle[vertices[i]][1] / scale_factor
                    p1.z = 0
                    points.append(p1)
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
                    points.append(p2)

                m.points = points
                marker_arr.markers.append(m)

        
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
        
