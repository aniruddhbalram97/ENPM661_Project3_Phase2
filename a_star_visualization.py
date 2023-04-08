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