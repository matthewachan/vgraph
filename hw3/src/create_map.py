#!/usr/bin/env python

import numpy as np # I update the numpy to 1.15.1 using sudo pip install --upgrade numpy
import cv2
		
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

def load_goal(goal_path):
	with open(goal_path) as f:
		line = f.readline()
		goal = list(map(int, line.strip().split(' ')))
	return goal
""" 
------------------------------------------
(0, 0) is at the center of the map;
(0, 0) is at the top left of the image

Thus, we need some transformation
------------------------------------------
""" 

def map2img(ob):
	""" transform an obstacle in map frame to img frame """
	ob_tranformed = []
	t = np.array([[1, 0, 0, 300], 
					[0, -1, 0, 300], 
					[0, 0, -1, 0],
					[0, 0, 0, 1]])
	for p in ob:
		p = np.array(p) # in case it is not already numpy array
		p = np.hstack((p, [0, 1]))
		p = t.dot(p).astype(int)
		ob_tranformed.append(p[:2])
	return np.array(ob_tranformed)

def img2map(ob):
	""" transform an obstacle in img frame to map frame """
	ob_tranformed = []
	t = np.array([[1, 0, 0, -300], 
					[0, -1, 0, 300],
					[0, 0, -1, 0],
					[0, 0, 0, 1]])
	for p in ob:
		p = np.array(p) # in case it is not already numpy array
		p = np.hstack((p, [0, 1]))
		p = t.dot(p).astype(int)
		ob_tranformed.append(p[:2])
	return np.array(ob_tranformed)

if __name__ == "__main__":

	# Create a black image
	img = np.full((600,1200,3), 255, np.uint8)

	obstacles = load_obstacles("../data/world_obstacles.txt")
	goal = load_goal("../data/goal.txt")
	start = [0, 0]

	# draw obstacles
	for ob in obstacles:
		ob = map2img(ob)
		# print(len(ob))
		# print(ob)
		cv2.fillConvexPoly(img, ob.reshape(-1, 1, 2), (255,255,0))

	# draw start and goal point
	goal = tuple(map2img([goal])[0])
	start = tuple(map2img([start])[0])
	cv2.circle(img, goal, 7, (100, 0, 0), -1)
	cv2.circle(img, start, 7, (0, 0, 100), -1)

	# import pdb; pdb.set_trace()
	# cv2.polylines(img,[pts],True,(0,255,255), thickness=1)

	cv2.imshow( "Display window", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	cv2.imwrite('../maps/map.png',img)