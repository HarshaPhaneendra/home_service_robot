#!/bin/bash

# exporting my world file
xterm  -e  "source devel/setup.bash; roslaunch my_robot world.launch" &
sleep 10
# exporting yaml file
xterm  -e "source devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 10
xterm  -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e "source devel/setup.bash; rosrun add_markers add_markers" &
sleep 5
xterm -e "source devel/setup.bash; rosrun pick_objects pick_objects" 

#xterm  -e "source devel/setup.bash; export TURTLEBOT_GAZEBO_MAP_FILE="$(pwd)/src/my_robot/maps/test_2.yaml"; roslaunch turtlebot_gazebo amcl_demo.launch" 
#xterm  -e  "source devel/setup.bash; export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/my_robot/worlds/test_2.world"; roslaunch turtlebot_gazebo turtlebot_world.launch" 
#xterm -e "source devel/setup.bash; rosrun teleop_twist_keyboard teleop_twist_keyboard.py" &
#sleep 5