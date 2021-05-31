#!/bin/bash

#export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/map/small_office_layout.world";
xterm  -e  "source devel/setup.bash; roslaunch my_robot world.launch" & 
sleep 10
#export TURTLEBOT_GAZEBO_MAP_FILE="$(pwd)/src/map/small_world.yaml";
xterm  -e "source devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 5
xterm  -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm  -e "source devel/setup.bash; rosrun visible_markers main_node"