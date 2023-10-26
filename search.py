import matplotlib.pyplot as plt
import matplotlib.path as mplP
import numpy as np
import time
import matplotlib.animation as animation

from utils import *
from grid import *
import sys

sys.setrecursionlimit(5000)
depth = 0
breadth = 0
greedy = 0
star = 0

def points_to_tuples(points):
    return [(p.x, p.y) for p in points]

def setVisited(visited):
    for i in range(50):
        for j in range(50):
            visited[i][j] = 0


def isObstructed(visited, point):
    u = len(epolygons)
    for k in range(len(epolygons)):
        shape = points_to_tuples(epolygons[k])
        for o in range(len(shape)):
            visited[shape[o][0]][shape[o][1]] = 1
        path = mplP.Path(shape)
        x = point.x
        y = point.y
        inside = path.contains_points([[x,y]], radius=-.01)
        for i in range(len(shape)):
            if inside:
                visited[point.x][point.y] = 1
                return True
    return False

def isTurf(visited, point):
    u = len(tpolygons)
    for k in range(len(tpolygons)):
        shape = points_to_tuples(tpolygons[k])
        for o in range(len(shape)):
            visited[shape[o][0]][shape[o][1]] = 1
        path = mplP.Path(shape)
        x = point.x
        y = point.y
        inside = path.contains_points([[x,y]], radius=-.01)
        for i in range(len(shape)):
            if inside:
                visited[point.x][point.y] = 1
                return True
    return False


def get_neighbors(node):
    neighbors = []
    # Check neighbor above
    if node.y < 50 - 1:
        neighbor = Point(node.x, node.y + 1)
        neighbors.append(neighbor)
    # Check neighbor right
    if node.x < 50 - 1:
        neighbor = Point(node.x + 1, node.y)
        neighbors.append(neighbor)
    # Check neighbor below
    if node.y > 0:
        neighbor = Point(node.x, node.y - 1)
        neighbors.append(neighbor)
    # Check neighbor to the left
    if node.x > 0:
        neighbor = Point(node.x - 1, node.y)
        neighbors.append(neighbor)
    return neighbors

class Node:
    def __init__(self, state, parent=None):
        self.state = state
        self.parent = parent

def expand(visited, node):
    x, y = node.state.x, node.state.y
    children = []

    if y < 50 - 1:
        child = Node(Point(x, y + 1), node)
        if (child.state.x, child.state.y) not in visited:
            children.append(child)
            visited.add((child.state.x, child.state.y))
    if x < 50 - 1:
        child = Node(Point(x + 1, y), node)
        if (child.state.x, child.state.y) not in visited:
            children.append(child)
            visited.add((child.state.x, child.state.y))
    if y > 0:
        child = Node(Point(x, y - 1), node)
        if (child.state.x, child.state.y) not in visited:
            children.append(child)
            visited.add((child.state.x, child.state.y))
    if x > 0:
        child = Node(Point(x - 1, y), node)
        if (child.state.x, child.state.y) not in visited:
            children.append(child)
            visited.add((child.state.x, child.state.y))
    return children, visited

def DFS(start, goal):
    frontier = [Node(start)]
    visited = set()
    global depth
    while frontier:
        node = frontier.pop()
        if node.state == goal:
            path = [goal]
            while path[-1] != start:
                path.append(node.parent.state)
                node = node.parent
            return path[::-1]
        children, visited = expand(visited, node)
        for child in children:
            if obstructions[child.state.x][child.state.y] == 0:
                depth = depth + 1
                frontier.append(child)
    return None





def BFS(visited, queue, start, goal):
    global breadth
    visitedBFS = set()
    visitedBFS.add((start.x, start.y))
    parent = {}
    parent[(start.x, start.y)] = None
    while queue:
        current = queue.pop(0)
        if current == goal:
            path = [(goal.x, goal.y)]
            while path[-1] != (start.x, start.y):
                path.append(parent[path[-1]])
            return path[::-1]
        for neighbor in get_neighbors(current):
            if visited[neighbor.x][neighbor.y] == 0:
                if obstructions[neighbor.x][neighbor.y] == 0:
                    if (neighbor.x, neighbor.y) not in visitedBFS:
                        breadth = breadth + 1
                        visitedBFS.add((neighbor.x, neighbor.y))
                        parent[(neighbor.x, neighbor.y)] = (current.x, current.y)
                        queue.append(neighbor)
    return None



import heapq
import math

def heuristic_func(point1, point2):
    x1, y1 = point1.x, point1.y
    x2, y2 = point2.x, point2.y
    return (math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2), x1, y1)

def GBFS(start, goal):
    start.parent = None
    global greedy
    visited = set()
    heap = [(heuristic_func(start, goal), start)]
    while heap:
        _, current = heapq.heappop(heap)
        if (current.x, current.y) in visited:
            continue
        visited.add((current.x, current.y))
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = current.parent
            return path[::-1]
        for neighbor in get_neighbors(current):
            if obstructions[neighbor.x][neighbor.y] == 0:
                if (neighbor.x, neighbor.y) not in visited:
                    greedy = greedy + 1
                    neighbor.parent = current
                    heapq.heappush(heap, (heuristic_func(neighbor, goal), neighbor))
    return None


def Astar(start, goal):
    visited = set()
    global star
    heap = [(heuristic_func(start, goal), (0, 0), id(start), start)]
    while heap:
        _, (cost, prev_h_cost), _, current = heapq.heappop(heap)
        if (current.x, current.y) in visited:
            continue
        visited.add((current.x, current.y))
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = current.parent
            return path[::-1]
        for neighbor in get_neighbors(current):
            if obstructions[neighbor.x][neighbor.y] == 0:
                if (neighbor.x, neighbor.y) not in visited:
                    neighbor.parent = current
                    star = star + 1
                    if turf[current.x][current.y] == 1:
                        g_cost = cost + 1.5
                    else:
                        g_cost = cost + 1.0
                    prev_g_cost_h_cost = prev_h_cost  # for unpacking
                    h_cost = heuristic_func(neighbor, goal)[0]
                    f_cost = g_cost + h_cost
                    heapq.heappush(heap, (f_cost, (g_cost, h_cost), id(neighbor), neighbor))
    return None


def gen_polygons(worldfilepath):
    polygons = []
    with open(worldfilepath, "r") as f:
        lines = f.readlines()
        lines = [line[:-1] for line in lines]
        for line in lines:
            polygon = []
            pts = line.split(';')
            for pt in pts:
                xy = pt.split(',')
                polygon.append(Point(int(xy[0]), int(xy[1])))
            polygons.append(polygon)
    return polygons


if __name__ == "__main__":
    epolygons = gen_polygons('TestingGrid/world1_enclosures.txt')
    tpolygons = gen_polygons('TestingGrid/world1_turfs.txt')

    source = Point(8, 10)
    dest = Point(43, 45)
    #dest = Point(7, 9)

    fig, ax = draw_board()
    draw_grids(ax)
    draw_source(ax, source.x, source.y)  # source point
    draw_dest(ax, dest.x, dest.y)  # destination point

    # Draw enclosure polygons
    for polygon in epolygons:
        for p in polygon:
            draw_point(ax, p.x, p.y)
    for polygon in epolygons:
        for i in range(0, len(polygon)):
            draw_line(ax, [polygon[i].x, polygon[(i + 1) % len(polygon)].x],
                      [polygon[i].y, polygon[(i + 1) % len(polygon)].y])

    # Draw turf polygons
    for polygon in tpolygons:
        for p in polygon:
            draw_green_point(ax, p.x, p.y)
    for polygon in tpolygons:
        for i in range(0, len(polygon)):
            draw_green_line(ax, [polygon[i].x, polygon[(i + 1) % len(polygon)].x],
                            [polygon[i].y, polygon[(i + 1) % len(polygon)].y])

    #### Here call your search to compute and collect res_path

    start = source
    res_path = [source]

    visited = []
    obstructions = []
    turf = []

    for i in range(50):
        inner_list = []
        inner_list2 = []
        inner_list3 = []
        for j in range(50):
            inner_list.append(0)  # or any other initial value
            inner_list2.append(0)  # or any other initial value
            inner_list3.append(0)  # or any other initial value
        visited.append(inner_list)
        obstructions.append(inner_list2)
        turf.append(inner_list3)

    for i in range(50):
        for j in range(50):
            isObstructed(obstructions, Point(i, j))
            if obstructions[i][j] == 1:
                draw_point(ax, i, j)

    for i in range(50):
        for j in range(50):
            isTurf(turf, Point(i, j))
            if turf[i][j] == 1:
                draw_green_point(ax, i, j)

    f = open("Summary.txt", "w")
    f.write("Search Alg                   Cost   Nodes Expanded\n")

    while(1>0):
        print("Which drawing would you like to see?")
        print("1. Depth First Search")
        print("2. Breadth First Search")
        print("3. Greedy Best First Search")
        print("4. A*")
        val = input("Enter: ")
        try:
            choice = int(val)
            if choice == 1 or choice == 2 or choice == 3 or choice == 4:
                break;
            print("\nInvalid input\n")
        except:
            print("\nInvalid input\n")




########################### DFS ##########################
    res_path = DFS(source, dest)
    count = 0.0
    if res_path is not None:
        for i in range(1, len( res_path)):
            if turf[res_path[i].x][res_path[i].y] == 1:
                count = count + .5
            count = count + 1.0
    f.write("Depth First Search           {0}   {1}\n".format(count, depth))


    if choice == 1:
        if res_path is not None:
            for i in range(len(res_path) - 1):
                draw_result_line(ax, [res_path[i].x, res_path[i + 1].x], [res_path[i].y, res_path[i + 1].y])
                plt.pause(0.1)
        else:
            print("No path found")
        plt.show()

########################### BFS ##########################

    res_path = [source]
    setVisited(visited)
    count = 0.0
    tuple_path = BFS(visited, res_path, start, dest)
    res_path = [source]
    for i in range(len(tuple_path)):
        res_path.append(Point(tuple_path[i][0], tuple_path[i][1]))

    if res_path is not None:
        for i in range(1, len( res_path)):
            if turf[res_path[i].x][res_path[i].y] == 1:
                count = count + .5
            count = count + 1.0

    f.write("Breadth First Search         {0}   {1}\n".format(count, breadth))

    if choice == 2:
        if res_path is not None:
            for i in range(len(res_path) - 1):
                draw_result_line(ax, [res_path[i].x, res_path[i + 1].x], [res_path[i].y, res_path[i + 1].y])
                plt.pause(0.1)
        else:
            print("No path found")
        plt.show()

########################### GBFS ##########################

    res_path = [source]
    count = 0.0
    res_path = GBFS(source, dest)

    if res_path is not None:
        for i in range(1, len( res_path)):
            if turf[res_path[i].x][res_path[i].y] == 1:
                count = count + .5
            count = count + 1.0

    f.write("Greedy Best First Search     {0}   {1}\n".format(count, greedy))

    if choice == 3:
        if res_path is not None:
            for i in range(len(res_path) - 1):
                draw_result_line(ax, [res_path[i].x, res_path[i + 1].x], [res_path[i].y, res_path[i + 1].y])
                plt.pause(0.1)
        else:
            print("No path found")
        plt.show()

########################### A* ##########################

    res_path = [source]
    count = 0.0
    res_path = Astar(source, dest)

    if res_path is not None:
        for i in range(1, len( res_path)):
            if turf[res_path[i].x][res_path[i].y] == 1:
                count = count + .5
            count = count + 1.0

    f.write("A*                           {0}   {1}\n".format(count, star))

    if choice == 4:
        if res_path is not None:
            for i in range(len(res_path) - 1):
                draw_result_line(ax, [res_path[i].x, res_path[i + 1].x], [res_path[i].y, res_path[i + 1].y])
                plt.pause(0.1)
        else:
            print("No path found")
        plt.show()


