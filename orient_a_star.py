
import heapq

actions = [-1, 0, 1]  # Right turn and move, move forward, Left turn and move
directions = {
    0: (-1, 0),  # North
    1: (0, 1),   # East
    2: (1, 0),   # South
    3: (0, -1)   # West
}

def neighbors_orient(current, cost, costs):
    neighbors = []
    x, y, theta = current  
    
    for i, action in enumerate(actions):
        new_theta = (theta + action) % 4 
        dx, dy = directions[new_theta]

        new_x = x + dx
        new_y = y + dy
        cell_cost = cost + costs[i] 

        neighbors.append(((new_x, new_y, new_theta), cell_cost))  
    
    return neighbors

def heuristic_distance(candidate,costs, goal):
    x1, y1, theta1 = candidate
    x2, y2, theta2 = goal
    manhattan_dist = abs(x1 - x2) + abs(y1 - y2)
    orientation_penalty = costs[2] if theta1 != theta2 else 0 
    return manhattan_dist + orientation_penalty  

def get_path_from_A_star_orient(start, goal, obstacles, costs):
    open_set = []
    heapq.heappush(open_set, (0, heuristic_distance(start,costs, goal), start, []))  # (g, h, node, path)

    visited = set()

    while open_set:
        g, h, current, path = heapq.heappop(open_set)

        if current in visited:
            continue
        visited.add(current)

        new_path = path + [current]

        if current == goal: 
            return new_path[1:]

        for neighbor, step_cost in neighbors_orient(current, g, costs):
            if neighbor[:2] not in obstacles and neighbor not in visited:
                f = step_cost + heuristic_distance(neighbor,costs, goal)
                new_g = g + step_cost
                heapq.heappush(open_set, (new_g, f, neighbor, new_path))

    return []
