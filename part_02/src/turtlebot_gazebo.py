#!/usr/bin/env python3
import sys
import rospy
import cv2
from queue import PriorityQueue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
from tf import transformations


# All measurements are in cm (including map)
############ Constants - Robot Dimensions #############
wheel_radius = 3.3 # from documentation 
robot_radius = 15 # from documentation
wheel_distance = 35.4 # from documentation
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
    x_g = int(input("Enter goal x-coordinate"))
    y_g = int(input("Enter goal y-coordinate"))
    
    g = (x_g, obstacle_map.shape[0] - y_g - 1)
    
    return g

# check for valid input coordinates
def check_valid_entry(start, goal, obstacle_map, robot_radius, clearance):
    net_clearance = robot_radius + clearance
    net_clearance = int(round(net_clearance))
    
    if(obstacle_map[start[1]][start[0]][0]==255 or obstacle_map[goal[1]][goal[0]][0]==255 or 
      (start[0] - net_clearance) < 0 or (start[0] + net_clearance) > 599 or (start[1] - net_clearance) < 0 or (start[1] + net_clearance) > 199
       or sum(obstacle_map[goal[1]][goal[0]])==765 or sum(obstacle_map[start[1]][start[0]])==765
      ):
        rospy.logerr("The start point or end point is invalid. Either it is out of bounds or within obstacle space. Try Again!\n")
        return False
    else:
        rospy.loginfo("Great! The start and goal points are valid")
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
def valid_next_steps(current_node, obstacle_map, action_set, clearance):
    valid_neighbors = []
    for action in action_set:
        neighbor = cost_function(current_node[0],current_node[1],current_node[2],action[0],action[1], clearance, obstacle_map)
        if(neighbor[1][0] and neighbor[1][1]):
            valid_neighbors.append(neighbor)
    return valid_neighbors  

# main a-star search function
def astar_search(obstacle_map, start, goal, rpm_1, rpm_2, clearance):
    action_set = [(0, rpm_1),(rpm_1, 0),(rpm_1, rpm_1),(0, rpm_2),(rpm_2, 0),(rpm_2, rpm_2),(rpm_1, rpm_2),(rpm_2, rpm_1)]
    while open_list:
        current_node = open_list.get()[1]
        close_list[int(round(current_node[1] * 2)),int(round(current_node[0] * 2))] = 2
        visual_list.append(current_node)
        goal_reached = False
        if(math.sqrt((current_node[0] - goal[0])**2 + (current_node[1] - goal[1])**2) < 0.5):
            goal_reached = True
        else:
            goal_reached = False
            
        if (goal_reached):
            rospy.loginfo("Reached the given goal")
            found_path = back_track_path(current_node)
            return found_path
        else:
            valid_neighbors = valid_next_steps(current_node, obstacle_map, action_set, clearance)
            for next_node in valid_neighbors:
                closed = close_list[int(round(next_node[1][1] * 2)),int(round(next_node[1][0] * 2))]
                # if the node is already present, ignore it
                if(closed == 2):
                    continue    
                else:
                    cost_to_next_node = next_node[0]
                    total_cost_to_node = cost_of_node[current_node] + cost_to_next_node
                    if(next_node[1] not in cost_of_node or total_cost_to_node < cost_of_node[next_node[1]]):
                        cost_of_node[next_node[1]] = total_cost_to_node
                        # Adding Euclidean heuristic
                        euc_dist = math.sqrt((next_node[1][0] - goal[0])**2 + (next_node[1][1] - goal[1])**2)
                        cost_with_heuristic = total_cost_to_node + euc_dist       
                        open_list.put((cost_with_heuristic, next_node[1]))
                        parent_node[next_node[1]] = current_node   
                        
# Function to back track and obtain the shortest path possible
def back_track_path(current_node):
    found_path = []
    final_node = current_node
    while final_node != None:
        found_path.append(final_node)
        final_node = parent_node[final_node]
    found_path.reverse()
    rospy.loginfo("The path traversed:\n")
    for node in found_path:
      rospy.loginfo(str((node[0], node[1])))
    return found_path

x = 0.0
y = 0.0 
theta = 0.0

# Odometry callback to know current position of the robot
def current_position(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    orientation = msg.pose.pose.orientation
    (m, n, theta) = transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])  #Getting yaw 

def turtlebot_simulation(publisher, planned_path, goal):
    vel = Twist()
    rate = rospy.Rate(4)
    for idx, node in enumerate(planned_path): #For each node in the path
        next_xpos = node[0]
        next_ypos = node[1]
        while True:
          # if the next position is reached with a tolerance of 0.07(user defined)
            if(math.sqrt((next_xpos - x)**2 + (next_ypos - y)**2)<0.1):
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                publisher.publish(vel)
                rospy.loginfo("Reached next point!")
                break
            else:
              # if there is some error in orientation of the robot
                if abs(math.atan2( next_ypos - y, next_xpos - x) - theta) > 0.2:
                    vel.linear.x = 0.0
                    # rotate in place till next point is reached
                    vel.angular.z = (math.atan2(next_ypos - y, next_xpos - x)-theta)*0.3
                else:
                    # move in straight line
                    vel.angular.z = 0.0
                    vel.linear.x = math.sqrt((next_xpos-x)**2 + (next_ypos-y)**2)*0.3
            publisher.publish(vel)
            rate.sleep()
            if ((idx == len(planned_path)-1) and math.sqrt((next_xpos - goal[0])**2 + (next_ypos - goal[1])**2)<0.1):
               vel.linear.x =0
               vel.angular.z = 0
               publisher.publish(vel)
               rospy.loginfo("Goal reached") 
               exit()
               
############################# MAIN FUNCTION ####################
def main_function(args):
  # initialize node
  rospy.init_node('tb_sim')
  
  # creating publisher
  publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  
  # create empty canvas
  canvas = np.zeros((200, 600, 3), dtype='uint8')
  clearance = 15
  obstacle_map = generate_map(canvas, clearance)

  odom_sub = rospy.Subscriber('/odom', Odometry, current_position)  #Odometry subscriber
  x_i, y_i, orientation_s = 50, 100, 0 #Starting point
  
  g = enter_coordinates(obstacle_map)
  i = (x_i, obstacle_map.shape[0] - y_i - 1, orientation_s)
  
  while(not check_valid_entry(i, g, obstacle_map, robot_radius, clearance)):
    g = enter_coordinates(obstacle_map)

  if(check_valid_entry(i, g, obstacle_map, robot_radius, clearance)):
    open_list.put((0, i))
    parent_node[i] = None # start node has no parent
    cost_of_node[i] = 0 # start node has no cost
  
  rospy.loginfo("Goal coordinates: " + str(g))
  # fixed rpms
  rpm1, rpm2 = 10, 10 #RPM's
  found_path = astar_search(obstacle_map, i, g, rpm1, rpm2, clearance)

  transformed_path = []
  # Transforming 2D coordinates to Gazebo coordinates
  # in 2D measures were in cm and y-axis was inverted, so fixing that here and generating a transformed path
  # Also shifting origin because start point is (50, 100)cm
  for node in found_path: 
    node_x = node[0]/100.0 - 0.5
    node_y = (200 - node[1])/100.0 - 1
    node_tup = (node_x, node_y)
    transformed_path.append(node_tup)

  transformed_goal = (g[0]/100.0 - 0.5, (200 - g[1])/100 - 1)

  while (not rospy.is_shutdown()):
    try:
        turtlebot_simulation(publisher, transformed_path, transformed_goal) 
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main_function(sys.argv)