# Turtlebot3 Robot Motion in ROS environment

# Run Autonomous Robot step by step
## 1. Input goal(x,y) in line 15 in file execute.sh
```rosrun automotive_robot RobotGlobalVisionUpdate.py -gx x -gy y```
## 2. Open 1 terminal and run roscore
```$ roscore```
## 3. Open 1 terminal and ssh to Turtlebot3 Burger, run node Bringup
```$ roslaunch turtlebot3_bringup turtlebot3_robot.launch```
## 4. Open 1 terminal and run node SLAM
```$ roslaunch turtlebot3_slam turtlebot3_slam.launch```
## 5. Open 1 terminal and go to direction store automotive_robot packages, then run execute.sh
```$ execute.sh```
