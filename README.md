## ENPM661 - Project 3 - Phase2

### Team:
Name: Aniruddh Balram
UID: 119206416
DirectoryID: abalram1

Name: Rishabh Namdev 
UID: 119480977
DirectoryID: rnamdev

### Dependencies:
1) Python 3
2) Numpy
3) OpenCV
4) geometry_msgs
5) roslaunch
6) ROS - Noetic(Ran on WSL)
7) nav_msgs

### part_01: 
- Contains the code for part_01 visualization
- Create a folder called "videos" for the videos to be written otherwise code will no-folder-found error

#### Build and Run:
```
# cd into project directory
cd <project_directory>/part_01

# create videos folder
mkdir videos

# run the python script
python3 a_star_visualization.py
```

### part_02:
- Contains rospy package for turtlebot simulation

#### Build and Run: 
```
# cd into project directory
cd <project_directory>/part_02

# add the part_02 to your catkin workspace
cd workspace/src
cp <project_directory>/part_02 workspace/src/.

# Rename part_02 to "project_3_phase_2_aniruddh_rishabh"

# set turtlebot3 to burger
export TURTLEBOT3_MODEL="BURGER"

# build catkin workspace
cd ../
catkin_make

# source workspace
source ./devel/setup.bash

# Make the python script executable
chmod +x src/turtlebot_gazebo.py

# launch the main file
roslaunch project_3_phase_2_aniruddh_rishabh final.launch

# Enter x and y coordinates of the goal (Spawn Position is default) after all the rospy logs are finished
```

### Videos:

- Part 01: https://drive.google.com/drive/folders/1bkmg_n7KPHb0Z_g0tp25p3hR1wPJGNMb

  - Start Position: (50, 100, 0)
  - Goal Position: (550, 100)
  - Clearance: 5
  - RPM1: 5
  - RPM2: 6

- Part 02(gazebo.mkv): https://drive.google.com/drive/folders/1bkmg_n7KPHb0Z_g0tp25p3hR1wPJGNMb

  - Start Position: (50, 100, 0) # Default spawn point on gazebo
  - Goal Position: (550, 100)
  - Clearance: 15

### Github Repository:
https://github.com/aniruddhbalram97/ENPM661_Project3_Phase2
