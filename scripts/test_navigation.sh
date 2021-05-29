#!/bin/bash

#xterm  -e  "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" & 
#sleep 10
#xterm  -e "source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch" &

#launch my world.launch
xterm  -e  "source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 10
# launch my amcl.launch
xterm  -e "source devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 5
xterm  -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &

