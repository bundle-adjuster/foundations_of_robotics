#!/usr/bin/env python3
import heapq

def neighbors(current): 
    # define the list of 4 neighbors

    neighbors = [(0,1),(0,-1),(1,0),(-1,0)]
    return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ]

def heuristic_distance(candidate, goal):
    distance = abs((candidate[0]-goal[0])) + abs((candidate[1]-goal[1])) # manhatton distance
    return abs((candidate[0]-goal[0])) + abs((candidate[1]-goal[1]))

def get_path_from_A_star(start, goal, obstacles):
    """
    Implements A* algorithm to find an optimal path from start to goal while avoiding obstacles.
    
    :param start: Tuple (x, y) representing the start node.
    :param goal: Tuple (x, y) representing the goal node.
    :param obstacles: List of tuples [(x1, y1), (x2, y2), ...] representing obstacle locations.
    :return: List of tuples [(x1, y1), (x2, y2), ...] representing the path.
    """
    
    if goal in obstacles:
        raise Exception('An exception: Goal node is an obstacle and cannot be reached.')

    # Priority queue (min-heap) for A* (cost, node)
    open_list = []
    heapq.heappush(open_list, (0, start))  # (f-score, node)
    
    came_from = {}  # To reconstruct path
    g_score = {start: 0}  # Cost from start to node
    f_score = {start: heuristic_distance(start, goal)}  # Estimated cost to goal

    while open_list:
        _, current = heapq.heappop(open_list)  # Get the node with the lowest f-score

        if current == goal:  # If goal is reached, reconstruct and return the path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # Reverse path to get correct order

        for neighbor in neighbors(current):
            if neighbor in obstacles:  # Ignore obstacles
                continue

            tentative_g_score = g_score[current] + 1  # Assume uniform cost (1 per move)

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic_distance(neighbor, goal)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return []  # Return empty path if no path is found

