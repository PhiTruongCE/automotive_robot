cmake_minimum_required(VERSION 3.16.3)
project(automotive_robot)

find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  nav_msgs
)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(FILES Path.msg Point.msg) 

#add_service_files(FILES )

catkin_python_setup()
generate_messages(DEPENDENCIES std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime nav_msgs 
)

## add include files
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})

###########
## Build ##
###########

## Declare a C++ executable
add_executable(robot_motion_node src/automotive_robot/robot_motion_node.cpp)
target_link_libraries(robot_motion_node ${catkin_LIBRARIES})
add_dependencies(robot_motion_node automotive_robot_generate_messages_cpp)

catkin_install_python(PROGRAMS src/automotive_robot/RobotGlobalVisionUpdate.py src/automotive_robot/trigger_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#src/automotive_robot/RobotGlobalVisionUpdate.py src/automotive_robot/Program_config.py src/automotive_robot/Robot_lib.py src/automotive_robot/Robot_control_panel.py src/automotive_robot/Robot_csv_lib.py src/automotive_robot/Robot_draw_lib.py src/automotive_robot/Robot_goal_lib.py src/automotive_robot/Robot_map_lib.py src/automotive_robot/Robot_paths_lib.py src/automotive_robot/Robot_sight_lib.py src/automotive_robot/Robot_world_lib.py

# add_executable(publisher src/automotive_robot/publisher.cpp)
# target_link_libraries(publisher ${catkin_LIBRARIES})
# add_dependencies(publisher automotive_robot_generate_messages_cpp)

#############
## Install ##
#############

install(DIRECTORY filter launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

