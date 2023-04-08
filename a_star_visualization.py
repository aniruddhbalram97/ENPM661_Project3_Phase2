# import libraries
import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
from queue import PriorityQueue

# All measurements are in cm (including map)
############ Constants - Robot Dimensions #############
wheel_radius = 3.3 # from documentation 
robot_radius = 10.5 # from documentation
wheel_distance = 3.54 # from documentation
####################################

############# Global Variables #########
open_list = PriorityQueue() # unvisited list
#close_list = [] # visited list
close_list = np.zeros((400, 1200), np.uint8)
visual_list = []
tree = []
# using maps to store costs and parent nodes
cost_of_node = {} # map containing cost of each node
parent_node = {} # map containing parent node of each node
#######################################

# input user data
def enter_coordinates(obstacle_map):
    x_i = int(input("Enter initial x-coordinate"))
    y_i = int(input("Enter initial y-coordinate"))
    x_g = int(input("Enter goal x-coordinate"))
    y_g = int(input("Enter goal y-coordinate"))
    
    orientation_s = int(input("Enter start orientation "))
    
    rpm_1 = int(input("Enter RPM 1: "))
    rpm_2 = int(input("Enter RPM 2: "))
    
    rpm = (rpm_1, rpm_2)
    i = (x_i, obstacle_map.shape[0] - y_i - 1, orientation_s)
    g = (x_g, obstacle_map.shape[0] - y_g - 1)
    
    return i, g, rpm


# check for valid input coordinates
def check_valid_entry(start, goal, obstacle_map, robot_radius, clearance):
    net_clearance = robot_radius + clearance
    net_clearance = int(round(net_clearance))
    
    if(obstacle_map[start[1]][start[0]][0]==255 or obstacle_map[goal[1]][goal[0]][0]==255 or 
      (start[0] - net_clearance) < 0 or (start[0] + net_clearance) > 599 or (start[1] - net_clearance) < 0 or (start[1] + net_clearance) > 199
       or sum(obstacle_map[goal[1]][goal[0]])==765 or sum(obstacle_map[start[1]][start[0]])==765
      ):
        print("The start point or end point is invalid. Either it is out of bounds or within obstacle space. Try Again!\n")
        return False
    else:
        print("Great! The start and goal points are valid")
        return True
    
    # Function to generate map using half plane equations
def generate_map(canvas, clearance):
    canvas_ = canvas.copy()
    for j in range(canvas.shape[0]):
        for i in range(canvas.shape[1]):
            
            # create rectangles with clearance.
            if(i>=150 - clearance and i < 165 + clearance and j<=125 + clearance):
                canvas_[j,i] = [255, 0 ,0]
            
            if(i>=250 - clearance and i < 265 + clearance and j>=75 - clearance):
                canvas_[j,i] = [255, 0 ,0]
                
            # create rectangles without clearance
            if(i>=150 and i < 165 and j<=125):
                canvas_[j,i] = [255, 255 ,255]
            if(i>=250 and i < 265 and j>=75):
                canvas_[j,i] = [255, 255 ,255]
            
            # create circle with clearance
            if((i - 400)**2 + (j - 90)**2 <= (50 + clearance)**2):
                canvas_[j,i] = [255, 0 ,0]   
            # create circle without clearance                         
            if((i - 400)**2 + (j - 90)**2 <= (50)**2):
                canvas_[j,i] = [255, 255 ,255]  
                            
            # clearance for walls
            if(i<clearance or j<clearance or i>canvas.shape[1] - 1 - clearance or j>canvas.shape[0] - 1 - clearance):
                canvas_[j, i] = [255, 0 ,0]
            
    return canvas_

# cost function which considers robot dynamics and outputs the next position, orientation and cost to reach that position if valid
def cost_function(x_i,y_i,theta_i,u_l,u_r, clearance, obstacle_map):
    t = 0
    dt = 0.1
    x_n = x_i
    y_n = y_i
    theta_n = 3.14 * theta_i / 180
    D = 0
    while t < 0.5:
        t = t + dt
        delta_x_n = 0.5 * wheel_radius * (u_l + u_r) * math.cos(theta_n) * dt
        delta_y_n = 0.5 * wheel_radius * (u_l + u_r) * math.sin(theta_n) * dt
        theta_n += (wheel_radius / wheel_distance) * (u_r - u_l) * dt
        D = D + math.sqrt((0.5 * wheel_radius * (u_l + u_r) * math.cos(theta_n) * dt)**2 + (0.5 * wheel_radius * (u_l + u_r) * math.sin(theta_n) * dt)**2)
        x_n+=delta_x_n
        y_n+=delta_y_n
        x_n = round(x_n)
        y_n = round(y_n)
        next_point = (x_n, y_n)
        if(not check_valid_point(next_point, obstacle_map, robot_radius, clearance)):
            return None, (None, None, None)
        tree.append(((x_i,y_i),(x_n,y_n)))
        x_i = x_n
        y_i = y_n
    theta_n = 180 * (theta_n) / 3.14
    if(theta_n < 0):
        theta_n+=360
    theta_n%=360
    theta_n = int(round(theta_n))
    return D, (x_n, y_n, theta_n)


# check valid neighbor points
def check_valid_point(neighbor, obstacle_map, robot_radius, clearance):
    net_clearance = robot_radius + clearance
    net_clearance = int(round(net_clearance))
    if(obstacle_map[neighbor[1]][neighbor[0]][0]==255 or  
      (neighbor[0] - net_clearance) < 0 or (neighbor[0] + net_clearance) > 599 or (neighbor[1] - net_clearance) < 0 or (neighbor[1] + net_clearance) > 199
       or  sum(obstacle_map[neighbor[1]][neighbor[0]])==765):
        return False
    else:
        return True
    
# Function which generates valid neighbors from a node point
def valid_next_steps(current_node, obstacle_map, action_set):
    valid_neighbors = []
    for action in action_set:
        neighbor = cost_function(current_node[0],current_node[1],current_node[2],action[0],action[1], clearance, obstacle_map)
        if(neighbor[1][0] and neighbor[1][1]):
            valid_neighbors.append(neighbor)
    return valid_neighbors  