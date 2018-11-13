# Program to load obstacle course for Lab 4 - RRT

# usage:  python rrt.py obstacles_file start_goal_file


from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import random, math
import time

# Node class
class Node:
    def __init__(self, point, parent):
        self.point = point
        self.parent = parent

def getDistance(a, b):
    return np.sqrt(np.power(a[0] - b[0], 2) + np.power(a[1] - b[1], 2))

# Points are tuples
# class Point:
#     def __init__(self, x, y):
#         self.x = x
#         self.y = y

#     def getX(self):
#         return self.x

#     def getY(self):
#         return self.y

def build_obstacle_course(obstacle_path, ax):
    vertices = list()
    codes = [Path.MOVETO]
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        lines = 0
        for line in f:
            coordinates = tuple(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                codes += [Path.MOVETO] + [Path.LINETO]*(coordinates[0]-1) + [Path.CLOSEPOLY]
                vertices.append((0,0)) #Always ignored by closepoly command
            else:
                vertices.append(coordinates)
    vertices.append((0,0))
    vertices = np.array(vertices, float)
    path = Path(vertices, codes)
    pathpatch = patches.PathPatch(path, facecolor='None', edgecolor='xkcd:violet')

    ax.add_patch(pathpatch)
    ax.set_title('Rapidly-exploring Random Tree')

    ax.dataLim.update_from_data_xy(vertices)
    ax.autoscale_view()
    ax.invert_yaxis()

    return path

def add_start_and_goal(start_goal_path, ax):
    start, goal = None, None
    with open(start_goal_path) as f:
        start = tuple(map(int, f.readline().strip().split(' ')))
        goal  = tuple(map(int, f.readline().strip().split(' ')))

    ax.add_patch(patches.Circle(start, facecolor='xkcd:bright green'))
    ax.add_patch(patches.Circle(goal, facecolor='xkcd:fuchsia'))

    return start, goal

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path',
                        help="File path for obstacle set")
    parser.add_argument('start_goal_path',
                        help="File path for obstacle set")
    args = parser.parse_args()

    fig, ax = plt.subplots()
    path = build_obstacle_course(args.obstacle_path, ax)
    start, goal = add_start_and_goal(args.start_goal_path, ax)
    
    # Initialize RRT
    rrt = Node(start, None)
    nodeList = [start]

    # Initialize constants
    stepLen = 10 # Default step length
    bounds = (600, 600) # Default bounds of 2D world space
    random.seed(time.time())

    # Generate q-rand
    qRand = (random.randint(0, bounds[0]), random.randint(0, bounds[1]))

    # Locate the closet node to q-rand in the RRT
    closestNode = nodeList[0]
    minDist = getDistance(qRand, closestNode)

    for node in nodeList:
        if getDistance(qRand, node) < minDist:
            closestNode = node
            minDist = getDistance(qRand, node)

    # Locate q-new
    qNew = ()
    if getDistance(qRand, closestNode) < stepLen:
        qNew = qRand
    else:
        # Linear algebra to find a point a given distance away on a line
        v = (qRand[0] - closestNode[0], qRand[1] - closestNode[1])
        norm = np.sqrt(np.power(v[0], 2) + np.power(v[1], 2))
        u = (v[0] / norm, v[1] / norm)
        qNew = (closestNode[0] + stepLen * u[0], closestNode[1] + stepLen * u[1])

    # Check q-new for collisions
    
    plt.show()
