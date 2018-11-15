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

def orientation(a, b, c):
    val = (a[1] - b[1]) * (c[0] - b[0]) - (a[0] - b[0]) * (c[1] - b[1])
    
    if val == 0:
        return 0
    elif val > 0:
        return 1
    return 2

def areEqual(v1, v2):
    return (v1[0] == v2[0] and v1[1] == v2[1]) 

def isIntersecting(e1, e2):
    a1 = e1[0]
    a2 = e1[1]
    b1 = e2[0]
    b2 = e2[1]
    o1 = orientation(a1, a2, b1)
    o2 = orientation(a1, a2, b2)
    o3 = orientation(b1, b2, a1)
    o4 = orientation(b1, b2, a2)

    if (areEqual(a1, b1) or areEqual(a1, b2) or areEqual(a2, b1) or areEqual(a2, b2)):
        return True
    if (o1 != o2 and o3 != o4):
        return True
    return False

# def growRRT(rrt, nodeList):

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
    verts = path.vertices[1:-1] # Ignore first (0, 0) coordinate
    codes = path.codes[1:-1]

    # Construct a list containing all obstacle edges
    start = verts[0]
    edges = []
    for i in range(1, len(verts)):
        if codes[i] == [Path.MOVETO]:
            start = verts[i]
            continue

        prev = verts[i - 1]
        if codes[i] == [Path.CLOSEPOLY]:
            edges.append((prev, start))
        else: 
            edges.append((prev, verts[i]))

    start, goal = add_start_and_goal(args.start_goal_path, ax)
    
    # Initialize RRT
    rrt = Node(start, None)
    nodeList = [rrt]

    # Initialize constants
    stepLen = 20 # Default step length
    bounds = (600, 600) # Default bounds of 2D world space
    random.seed(time.time())

    for i in range(0,5):
        # Generate q-rand
        qRand = (random.randint(0, bounds[0]), random.randint(0, bounds[1]))

        # Locate the closet node to q-rand in the RRT
        closestNode = nodeList[0]
        minDist = getDistance(qRand, closestNode.point)

        for node in nodeList:
            if getDistance(qRand, node.point) < minDist:
                closestNode = node
                minDist = getDistance(qRand, node.point)

        # Locate q-new
        qNew = ()

        # Set q-new to equal q-rand if the distance between them is shorter 
        # than the step length
        if getDistance(qRand, closestNode.point) < stepLen:
            qNew = qRand
        # Linear algebra to find a point a given distance away on a line
        else:
            v = (qRand[0] - closestNode.point[0], qRand[1] - closestNode.point[1])
            norm = np.sqrt(np.power(v[0], 2) + np.power(v[1], 2))
            u = (v[0] / norm, v[1] / norm)
            qNew = (closestNode.point[0] + stepLen * u[0], closestNode.point[1] + stepLen * u[1])

        # Plot q-new on the map
        ax.add_patch(patches.Circle(qNew, facecolor='xkcd:bright green'))
        # Plot the edge between q-new and its parent on the map
        path = Path([closestNode.point, qNew], [Path.MOVETO, Path.LINETO])
        pathpatch = patches.PathPatch(path, facecolor='None', edgecolor='xkcd:violet')
        ax.add_patch(pathpatch)

        # Check q-new for collisions
        for edge in edges:
            if isIntersecting(edge, (closestNode.point, qNew)):
                raise Exception("qnew intersects with an obstacle") 
            else:
                nodeList.append(Node(qNew, closestNode))

    plt.show()
