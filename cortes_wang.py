# ENPM 661 - Planning for Autonomous Robots: 
# Project 3 - Point Robot Dijkstra on Mobile Robot
# Shon Cortes, Bo-Shiang Wang

import numpy as np
import copy
import cv2

def obstacles_chk(node): # Check if position is in Robot Adjusted obstacle space. Obstacle space was expanded by a radius of 10 + 5 for clearance for a total of 15. Warning obstacles appear larger than they are.
   
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

def obstacle_visulaize(node): # Check if point is an obstacle. Original, un-modified obstacle space.
    # Rectangle 
    if node[1] >= (node[0] * 0.7) + 74.39 and node[1] <= (node[0] * 0.7) + 98.568 and node[1] >= (node[0] * -1.428) + 176.554 and node[1] <= (node[0] * -1.428) + 438.068:
        return True

    # Circle
    elif (node[0] - 90)**2 + (node[1] - 70)**2 <= (35)**2:
        return True

    # Elipse
    elif ((node[0] - 246)**2)/((60)**2) + ((node[1] - 145)**2)/((30)**2) <= 1:
        return True
    
    # 3 section Rectangular Area
    elif node[0] >= 200 and node[0] <= 230 and node[1] >= 230 and node[1] <= 280: # First section
        return True
 
    else:
        return False

def move_check(child_node): # Check if the move is allowed. 

    # Check if out of puzzle boundary
    if child_node[0] < 0 or child_node[1] < 0 or child_node[0] >= map.shape[1] or child_node[1] >= map.shape[0]:
        return False

    # Check if obstacle
    elif obstacles_chk(child_node):
        return False

    else:
        return True
 
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

class Node: # Class for storing node position, cost to come, and parent index.
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

def motion_model(): # Defines action set and cost [move in x, move in y, cost].
    model = [[1, 0, 1], # Right
             [0, 1, 1], # Up
             [-1, 0, 1], # Left
             [0, -1, 1], # Down
             [-1, -1, np.sqrt(2)], # Down Left
             [-1, 1, np.sqrt(2)], # Up Left
             [1, -1, np.sqrt(2)], # Down Right
             [1, 1, np.sqrt(2)]] # Up Right

    return model

def dijkstra(start_x, start_y, goal_x, goal_y):
    
    # Initialize start and goal nodes from node class
    start_node = Node(start_x, start_y, 0, -1)
    goal_node = Node(goal_x, goal_y, 0, -1)

    # obstacle_map = ObstacleMap((start_x, start_y), (goal_x, goal_y))

    # Initialize dictionaries
    path, distance, queue, visited = dict(), dict(), dict(), dict()

    motion = motion_model() # Initialize action set
    explored_map = [] # Initialize list of explored positions
    queue[(start_node.x, start_node.y)] = start_node # Initialize queue with startnode for Dijkstra algorithm.
    distance[(start_node.x, start_node.y)] = 0 # Initialize distance traveled.

    while True: # Start of Dijkstra Algorithm.

        cur_index = min(queue, key=lambda o: queue[o].cost) # Find the node in queue with the minimum cost.
        cur = queue[cur_index] # Assign node in queue with minimum cost to be the current node to be tested.
        
        if cur.x == goal_node.x and cur.y == goal_node.y: # If goal node is reached, Break the while loop.
            # print("Goal!!!")
            goal_node.parent_index = cur.parent_index
            goal_node.cost = cur.cost
            break

        del queue[cur_index] # Remove the current node from the queue.
        visited[cur_index] = cur # Add current node to visited list.

        explored_map.append((cur.x, cur.y)) # Add current node to explored map.

        for i in range(len(motion)): # Generate childeren of current node based on the action set.

            node = Node(cur.x + motion[i][0], cur.y + motion[i][1], cur.cost + motion[i][2], cur_index) # Generate child node
            node_index = (node.x, node.y) # Assign child node position

            # Check if the next node is valid (Out of boundary)
            # if not obstacle_map.is_valid(node.x, node.y):
            #     continue

            # Check if the next node is a obstacle
            # if is_obstacle(node.x, node.y):
            #     continue

            if move_check(node_index): # Check if child is within the map or in an obstacle.
                pass

            if node_index in visited: # If the next node is already visited, skip it
                continue

            if node_index in queue: # If the child node is already in the queue, compare and update the node's cost and parent as needed.
                if queue[node_index].cost > node.cost:
                    queue[node_index].cost = node.cost
                    queue[node_index].parent_index = cur_index
            else: # Else add child to the queue.
                queue[node_index] = node

    # Backtrack the path from Goal to Start
    path = []
    parent_index = goal_node.parent_index
    while parent_index != -1: # Follow the parents from the goal node to the start node and add them to the path list.
        n = visited[parent_index]
        path.append((n.x, n.y))
        parent_index = n.parent_index

    path = list(reversed(path)) # Reverse the path list to get the path from start node to goal node.
    path.append((goal_x, goal_y)) # Add Goal node to the end of the path list
    return explored_map, path

if __name__ == "__main__":

    # Define Dijkstra parameters for storing the queue, parent/children information, and initialize map
    open_nodes = []
    children_nodes = []
    parent_nodes = []
    parent_index = 0
    map = 255*np.ones([301, 401, 3], dtype = np.uint8) # Define a map of ones for OpenCV Visualization.
    for i in range(len(map)): # Initialize map obstacles by setting the bostacle points to 0. OpenCV displays values of 0 as black.
        for j in range(len(map[i])):
            point = j, i 
            if obstacles_chk(point):
                map[i][j] = 0
    
    # Ask for user input of start and goal pos. Start and goal much be positive integers
    start_x, start_y, goal_x, goal_y = begin()

    explored_map, path = dijkstra(start_x, start_y, goal_x, goal_y) # Call Dijkstra algorithm

    print(path)
