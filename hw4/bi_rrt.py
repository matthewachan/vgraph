# Program to load obstacle course for Lab 4 - RRT

# usage:  python bi_rrt.py obstacles_file start_goal_file

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

def tracePath(nodeList):
    curr = nodeList[-1]
    while curr.parent != None:
        ax.add_patch(patches.Circle(curr.point, facecolor='xkcd:red', linewidth=5))
        curr = curr.parent
        plt.pause(pauseTime)

    ax.add_patch(patches.Circle(curr.point, facecolor='xkcd:red', linewidth=5))
    plt.pause(pauseTime)


def expandRRT(qRand, nodeList, edges, pointColor, edgeColor):
    # Locate the closet node to q-rand in the RRT
    closestNode = nodeList[0]
    minDist = getDistance(qRand, closestNode.point)

    for node in nodeList:
        if getDistance(qRand, node.point) < minDist:
            closestNode = node
            minDist = getDistance(qRand, node.point)

    # Locate q-new
    qNew = ()

    # Find a new q-rand if the distance between the closest point and
    # q-rand is too small
    if getDistance(qRand, closestNode.point) < stepLen:
        return
    # Linear algebra to find a point a given distance away on a line
    else:
        v = (qRand[0] - closestNode.point[0], qRand[1] - closestNode.point[1])
        norm = np.sqrt(np.power(v[0], 2) + np.power(v[1], 2))
        u = (v[0] / norm, v[1] / norm)
        qNew = (closestNode.point[0] + stepLen * u[0], closestNode.point[1] + stepLen * u[1])

    # Check q-new for collisions
    hasCollision = isColliding(edges, (closestNode.point, qNew))

    # Add q-new to the RRT if no collisions are detected
    if not hasCollision:
        nodeList.append(Node(qNew, closestNode))
        lastAdded = qNew

        # Plot q-new on the map
        ax.add_patch(patches.Circle(qNew, facecolor=pointColor))
        # Plot the edge between q-new and its parent on the map
        path = Path([closestNode.point, qNew], [Path.MOVETO, Path.LINETO])
        pathpatch = patches.PathPatch(path, facecolor='None', edgecolor=edgeColor)
        ax.add_patch(pathpatch)

        plt.pause(pauseTime)
        return qNew
    return
        
def connect(qNew, goal, nodeList, edges, pointColor, edgeColor):
    if getDistance(qNew, goal) < stepLen and isColliding(edges, (qNew, goal)) == False:
        nodeList.append(Node(goal, nodeList[-1]))
        lastAdded = goal
        # Plot q-new on the map
        ax.add_patch(patches.Circle(goal, facecolor=pointColor))
        # Plot the edge between q-new and its parent on the map
        path = Path([goal, qNew], [Path.MOVETO, Path.LINETO])
        pathpatch = patches.PathPatch(path, facecolor='None', edgecolor=edgeColor)
        ax.add_patch(pathpatch)
        plt.pause(pauseTime)
        return True
    return False


def isColliding(obstacles, newEdge): 
    for edge in obstacles:
        if isIntersecting(edge, newEdge):
            return True
    return False

def getDistance(a, b):
    return np.sqrt(np.power(a[0] - b[0], 2) + np.power(a[1] - b[1], 2))

def orientation(a, b, c):
    val = (a[1] - b[1]) * (c[0] - b[0]) - (a[0] - b[0]) * (c[1] - b[1])
    
    if val == 0:
        return 0
    elif val > 0:
        return 1
    return 2

def onEdge(p1, p2, p3):
    if p2[0] <= max(p1[0], p3[0]) and p2[0] >= min(p1[0], p3[0] and p2[1]) <= max(p1[1], p3[1]) and p2[1] >= min(p1[1], p3[1]):
        return True
    return False

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

    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and onEdge(a1, b1, a2):
        return True
    if o2 == 0 and onEdge(a1, b2, a2):
        return True
    if o3 == 0 and onEdge(b1, a1, b2):
        return True
    if o4 == 0 and onEdge(b1, a2, b2):
        return True
    if (areEqual(a1, b1) or areEqual(a1, b2) or areEqual(a2, b1) or areEqual(a2, b2)):
        return True
    return False


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

    ax.add_patch(patches.Circle(start, facecolor='xkcd:royal blue'))
    ax.add_patch(patches.Circle(goal, facecolor='xkcd:fuchsia'))

    return start, goal

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path',
                        help="File path for obstacle set")
    parser.add_argument('start_goal_path',
                        help="File path for obstacle set")
    parser.add_argument('step_length',
                        help="Step length not specified")
    parser.add_argument('max_iters',
                        help="Max number of iterations not specified")
    args = parser.parse_args()

    fig, ax = plt.subplots()
    path = build_obstacle_course(args.obstacle_path, ax)
    verts = path.vertices[1:] # Ignore first (0, 0) coordinate
    codes = path.codes[1:]

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
    
    # Initialize an RRT at the start position
    rrtStart = Node(start, None)
    startList = [rrtStart]
    lastAdded = rrtStart.point 

    # Initialize another RRT at the goal position
    rrtGoal = Node(goal, None)
    goalList = [rrtGoal]

    # Keep track of iterations
    numIter = 0

    # Initialize constants
    stepLen = int(args.step_length) # Default step length
    bounds = (600, 600) # Default bounds of 2D world space
    random.seed(time.time())
    maxIters = int(args.max_iters)
    threshold = 20 # Threshold before trying to reach the goal in one step

    startPointColor = 'xkcd:bright green'
    startEdgeColor = 'xkcd:violet'
    goalPointColor = 'xkcd:olive green'
    goalEdgeColor = 'xkcd:maroon'

    pauseTime = 0.01 # Pause duration between plot refreshes

    while True:
        if numIter >= maxIters:
            raise Exception("Maximum number of iterations reached and goal still not found")
        # Generate q-rand
        qRand = (random.randint(0, bounds[0]), random.randint(0, bounds[1]))
        # Grow start RRT
        qNew = expandRRT(qRand, startList, edges, startPointColor, startEdgeColor)
        # Grow goal RRT towards qnew if it exists, otherwise grow it towards a random point
        if not qNew:
            qRand = (random.randint(0, bounds[0]), random.randint(0, bounds[1]))
            expandRRT(qRand, goalList, edges, goalPointColor, goalEdgeColor)
        else:
            # qNew successfully added, check if we can connect
            lastNode = goalList[-1].point
            if connect(qNew, lastNode, goalList, edges, goalPointColor, goalEdgeColor):
                break
            else:
                qNew = expandRRT(qNew, goalList, edges, goalPointColor, goalEdgeColor)
                if qNew:
                    lastNode = startList[-1].point
                    if connect(qNew, lastNode, startList, edges, startPointColor, startEdgeColor):
                        break


        numIter += 1
            

    # Trace the path from goal to start in red
    tracePath(startList)
    tracePath(goalList)

    plt.show()
