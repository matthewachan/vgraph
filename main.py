import rospy
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

        self.marker_pub = rospy.Publisher('vgraph_markers', Marker, queue_size=10)
        r = rospy.Rate(30)

	obstacles = load_obstacles("../data/world_obstacles.txt")
        grown_obstacles = []
        grown_obstacle = []

        aabb_sidelen = 36
        half = aabb_sidelen / 2
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
                

        while (not rospy.is_shutdown()):
            # rospy.loginfo("WOO")
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.ns = "ns1"
            m.action = Marker.ADD;
            m.pose.orientation.w = 1.0

            m.type = Marker.POINTS
            m.scale.x = 0.10
            m.scale.y = 0.10
            m.color.g = 1.0
            m.color.a = 1.0
            points = []

            for grown_obstacle in grown_obstacles:
                hull = ConvexHull(grown_obstacle)
                for vertex in hull.vertices:
                    p = Point()
                    p.x = grown_obstacle[vertex][0] / scale_factor
                    p.y = grown_obstacle[vertex][1] / scale_factor
                    p.z = 0
                    points.append(p)
            m.points = points

        
            self.marker_pub.publish(m)
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
