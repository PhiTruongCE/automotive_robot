#!/bin/bash

echo "========== Begin catkin make =======!";
cd ../../
catkin_make
cd -
echo "========== Catkin make done =======!";

echo "========== Add sources =======!";
source ../../devel/setup.bash

rosrun automotive_robot robot_motion_node