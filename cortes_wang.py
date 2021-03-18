# ENPM 661 - Planning for Autonomous Robots: 
# Project 3 - Point Robot Dijkstra on Mobile Robot
# Shon Cortes Bo-Shiang Wang

import numpy as np
import copy
import cv2

map = 255*np.ones([301, 401, 3], dtype = np.uint8)

def obstacles_chk(node): # Check if position is in obstacle space.
   
    # Rectangle 
    if node[1] >= (node[0] * 0.7) + 74.39 - 15 and node[1] <= (node[0] * 0.7) + 98.568 + 15 and node[1] >= (node[0] * -1.428) + 176.554 - 15 and node[1] <= (node[0] * -1.428) + 438.068 + 15:
        return True

    # Circle
    elif (node[0] - 90)**2 + (node[1] - 70)**2 <= (35+15)**2:
        return True

    # Elipse
    elif ((node[0] - 246)**2)/((60 + 15)**2) + ((node[1] - 145)**2)/((30 + 15)**2) <= 1:
        return True
    
    # 3 section Rectangular Area
    elif node[0] >= 200 - 15 and node[0] <= 230 + 15 and node[1] >= 230 - 15 and node[1] <= 280 + 15: # First section
        return True
 
    else:
        return False

def move_check(child_node, move): # Check if the move is allowed. If it is, add the child node to the visited_list and children_nodes list
    child_node = copy.deepcopy(child_node)

    # Check if out of puzzle boundary
    if child_node[0] < 0 or child_node[1] < 0 or child_node[0] >= map.shape[1] or child_node[1] >= map.shape[0]:
        return

    # Check if obstacle
    elif obstacles_chk(child_node):
        return

    # Check if child node has been visited before
    try:
        visited_list.index([child_node[0], child_node[1]])

    except ValueError:
        
        # Add child to visited and parent nodes list
        visited_list.append([child_node[0], child_node[1]])
        
        parent_nodes.append(child_node)
        child_node[2] = parent_index
        children_nodes.append(child_node)
        # print(move) # For debugging
    else:
        return
 
def begin(): # Ask for user input of start and goal pos. Start and goal much be positive integers
    while True:
        
        start_x, start_y = input("Enter starting x and y coordinates separated with a space: ").split()
        goal_x, goal_y = input("Enter goal x and y coordinates separated with a space: ").split()
        
        start_node = [int(start_x), int(start_y), parent_index]
        goal_node = [int(goal_x), int(goal_y)]

        # Check if obstacle
        if obstacles_chk(start_node):
            print("Start position is in an obstacle.")
        elif obstacles_chk(goal_node):
            print("Goal position is in an obstacle.")

        # Check if values are positive and within the map
        elif start_node[0] < 0 or start_node[1] < 0 or start_node[0] > map.shape[1] or start_node[1] > map.shape[0]:
            print("Please enter positive integer values (0 <= x <= 400, 0 <= y <= 300).")
        elif goal_node[0] < 0 or goal_node[1] < 0 or goal_node[0] > map.shape[1] or goal_node[1] > map.shape[0]:
            print("Please enter positive integer values (0 <= x <= 400, 0 <= y <= 300).")

        else:
            break
    return int(start_x), int(start_y), int(goal_x), int(goal_y)

def visualize_BFS(node):
    
    for i in range(len(node)):
        point = node[i]
        map[point[1]][point[0]] = [0, 255, 0]

def visualize_path(node): 

    while len(node) != 0:
        point = node.pop()
        map[point[1]][point[0]] = [0, 0, 255]

    cv2.imshow("Map", map)

class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

def motion_model():
    model = [[1, 0, 1],
             [0, 1, 1],
             [-1, 0, 1],
             [0, -1, 1],
             [-1, -1, np.sqrt(2)],
             [-1, 1, np.sqrt(2)],
             [1, -1, np.sqrt(2)],
             [1, 1, np.sqrt(2)]]

    return model

def dijkstra(start_x, start_y, goal_x, goal_y):
    start_node = Node(start_x, start_y, 0, -1)
    goal_node = Node(goal_x, goal_y, 0, -1)

    # obstacle_map = ObstacleMap((start_x, start_y), (goal_x, goal_y))
    path, distance, queue, visited = dict(), dict(), dict(), dict()

    motion = motion_model()
    explored_map = []
    queue[(start_node.x, start_node.y)] = start_node
    distance[(start_node.x, start_node.y)] = 0

    while True:
        cur_index = min(queue, key=lambda o: queue[o].cost)
        cur = queue[cur_index]
        if cur.x == goal_node.x and cur.y == goal_node.y:
            # print("Goal!!!")
            goal_node.parent_index = cur.parent_index
            goal_node.cost = cur.cost
            break

        del queue[cur_index]
        visited[cur_index] = cur

        explored_map.append((cur.x, cur.y))

        for i in range(len(motion)):
            node = Node(cur.x + motion[i][0], cur.y + motion[i][1], cur.cost + motion[i][2], cur_index)
            node_index = (node.x, node.y)

            # Check if the next node is valid (Out of boundary)
            # if not obstacle_map.is_valid(node.x, node.y):
            #     continue

            # Check if the next node is a obstacle
            # if is_obstacle(node.x, node.y):
            #     continue

            # If the next node is already visited, skip it
            if node_index in visited:
                continue

            if node_index in queue:
                if queue[node_index].cost > node.cost:
                    queue[node_index].cost = node.cost
                    queue[node_index].parent_index = cur_index
            else:
                queue[node_index] = node

    # Backtrack the path from Goal to Start
    path = []
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = visited[parent_index]
        path.append((n.x, n.y))
        parent_index = n.parent_index

    path = list(reversed(path))
    path.append((goal_x, goal_y))
    return explored_map, path

if __name__ == "__main__":

    # Define BFS parameters for storing the queue, parent/children information, and initialize map
    open_nodes = []
    children_nodes = []
    parent_nodes = []
    parent_index = 0
    for i in range(len(map)): # Initialize map obstacles
        for j in range(len(map[i])):
            point = j, i 
            if obstacles_chk(point):
                map[i][j] = 0
    
    # Ask for user input of start and goal pos. Start and goal much be positive integers
    start_x, start_y, goal_x, goal_y = begin()

    explored_map, path = dijkstra(start_x, start_y, goal_x, goal_y)

    print(path)
