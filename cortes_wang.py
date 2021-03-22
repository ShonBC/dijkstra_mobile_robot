# ENPM 661 - Planning for Autonomous Robots: 
# Project 3 - Point Robot Dijkstra on Mobile Robot
# Shon Cortes, Bo-Shiang Wang

import numpy as np
import cv2

def obstacles_chk(NODE): # Check if position is in Robot Adjusted obstacle space. Obstacle space was expanded by a radius of 10 + 5 for clearance for a total of 15. Warning obstacles appear larger than they are.
 
    node = [NODE.x, NODE.y, NODE.radius]

    # Rectangle 
    if node[1] >= (node[0] * 0.7) + 74.39 - node[2] and node[1] <= (node[0] * 0.7) + 98.568 + node[2] and node[1] >= (node[0] * -1.428) + 176.554 - node[2] and node[1] <= (node[0] * -1.428) + 438.068 + node[2]:
        return True

    # Circle
    elif (node[0] - 90)**2 + (node[1] - 70)**2 <= (35 + node[2])**2:
        return True

    # Elipse
    elif ((node[0] - 246)**2)/((60 + node[2])**2) + ((node[1] - 145)**2)/((30 + node[2])**2) <= 1:
        return True
    
    # 3 section Rectangular Area    
    elif node[0] >= 200 - node[2] and node[0] <= 210 + node[2] and node[1] >= 230 - node[2] and node[1] <= 280 + node[2]: # First section
        return True
    elif node[0] >= 210 - node[2] and node[0] <= 230 + node[2] and node[1] >= 270 - node[2] and node[1] <= 280 + node[2]: # Second section
        return True
    elif node[0] >= 210 - node[2] and node[0] <= 230 + node[2] and node[1] >= 230 - node[2] and node[1] <= 240 + node[2]: # Third section
        return True
 
    else:
        return False

def move_check(child_node): # Check if the move is allowed. 

    # Check if out of puzzle boundary
    if child_node.x < 0 or child_node.y < 0 or child_node.x >= obstacle_map.shape[1] or child_node.y >= obstacle_map.shape[0]:
        return False

    # Check if obstacle
    elif obstacles_chk(child_node):
        return False

    else:
        return True
 
def begin(obstacle_map): # Ask for user input of start and goal pos. Start and goal much be positive integers
    while True:
        
        start_x, start_y = input("Enter starting x and y coordinates separated with a space: ").split()
        goal_x, goal_y = input("Enter goal x and y coordinates separated with a space: ").split()
        start_x = int(start_x)
        start_y = int(start_y)
        goal_x = int(goal_x)
        goal_y = int(goal_y)

        # Initialize start and goal nodes from node class
        start_node = Node(start_x, start_y, 15, 0, -1)
        goal_node = Node(goal_x, goal_y, 15, 0, -1)

        # Check if obstacle
        if obstacles_chk(start_node):
            print("Start position is in an obstacle.")
        elif obstacles_chk(goal_node):
            print("Goal position is in an obstacle.")

        # Check if values are positive and within the map
        elif start_node.x < 0 or start_node.y < 0 or start_node.x > obstacle_map.shape[1] or start_node.y > obstacle_map.shape[0]:
            print("Please enter positive integer values (0 <= x <= 400, 0 <= y <= 300).")
        elif goal_node.x < 0 or goal_node.y < 0 or goal_node.x > obstacle_map.shape[1] or goal_node.y > obstacle_map.shape[0]:
            print("Please enter positive integer values (0 <= x <= 400, 0 <= y <= 300).")

        else:
            break

    return start_node, goal_node

def visualize_Dij(node): # Visualize 
    global obstacle_map
    for i in range(len(node)):
        # point = node[i]
        obstacle_map[node[1]][node[0]] = [0, 255, 0]

def visualize_path(node): # Visualize path and wait for ESC before closing

    for i in range(len(node)): # Trace path from start to goal
        point = node[i]
        obstacle_map[point[1]][point[0]] = [0, 0, 255]

        cv2.imshow("Map", obstacle_map)
        out.write(obstacle_map) # Save output as video
    cv2.waitKey(0)


    # while len(node) != 0: # Trace path form goal to start
    #     point = node.pop()
    #     obstacle_map[point[1]][point[0]] = [0, 0, 255]

    #     cv2.imshow("Map", obstacle_map)
    #     out.write(obstacle_map) # Save output as video
    #     cv2.waitKey(10)

class Node: # Class for storing node position, cost to come, and parent index.
    def __init__(self, x, y, radius, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index
        self.radius = radius

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

def dijkstra(start_node, goal_node):

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

            node = Node(cur.x + motion[i][0], cur.y + motion[i][1], 15, cur.cost + motion[i][2], cur_index) # Generate child node
            node_index = (node.x, node.y) # Assign child node position

            if move_check(node): # Check if child is within the map or in an obstacle.
                pass
            else: # If out of bounds or an obstacle, restart loop and choose new node.
                continue

            visualize_Dij(node_index)

            if node_index in visited: # If the next node is already visited, skip it
                continue

            if node_index in queue: # If the child node is already in the queue, compare and update the node's cost and parent as needed.
                if queue[node_index].cost > node.cost:
                    queue[node_index].cost = node.cost
                    queue[node_index].parent_index = cur_index
            else: # Else add child to the queue.
                queue[node_index] = node

            # cv2.imshow("Map", obstacle_map)
            
        cv2.imshow("Map", obstacle_map)

        out.write(obstacle_map) # Save output as video

        cv2.waitKey(1) # Refresh rate
        # Condition to break the while loop
        if i == 27:
            break
    cv2.imshow("Map", obstacle_map)

    

    # Backtrack the path from Goal to Start
    path = []
    parent_index = goal_node.parent_index
    while parent_index != -1: # Follow the parents from the goal node to the start node and add them to the path list.
        n = visited[parent_index]
        path.append((n.x, n.y))
        parent_index = n.parent_index

    path = list(reversed(path)) # Reverse the path list to get the path from start node to goal node.
    path.append((goal_node.x, goal_node.y)) # Add Goal node to the end of the path list

    return explored_map, path

if __name__ == "__main__":

    obstacle_map = 255*np.ones([301, 401, 3], dtype = np.uint8) # Define a map of ones for OpenCV Visualization.

    # Video output variables
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('Dijkstra.mp4', fourcc, 20.0, (400, 300))

    for i in range(len(obstacle_map)): # Initialize map obstacles by setting the bostacle points to 0. OpenCV displays values of 0 as black.
        for j in range(len(obstacle_map[i])):
            # point = j, i, 0
            obstacle = Node(j, i, 0, 0, 0)
            if obstacles_chk(obstacle):
                obstacle_map[i][j] = 0
    
    # Ask for user input of start and goal pos. Start and goal much be positive integers
    start_node, goal_node = begin(obstacle_map)
    

    explored_map, path = dijkstra(start_node, goal_node) # Call Dijkstra algorithm

    # print(path)
    visualize_path(path)
