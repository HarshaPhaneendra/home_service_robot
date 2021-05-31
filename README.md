# home_service_robot
This Project is of "Robot Path Planning and Navigation", part of Udacity Nano Degree Program "Robotics Engineer". 

## Objective 
* Robot has to navigate on its own to a specific pose and pick up virtual marker. 
* Later it has to navigate to a goal pose and drop off the virtual marker. 

## Packages 
"home_service_robot" project consists of serveral custum built along with pre-existed packages from ros community. 
* Localization is achieved using AMCL algorithm. Its parameters are customized to enhance the localization ability. 
* Environmnet Mapping is done by 'pgm_map_cearter' package, which creates a '.pgm' map from pre-build build world environment.
* To achieve Robot Navigation, Dijkstra's algorithm (a variant of the Uniform Cost Search algorithm) is used. Which results in, ROS navigation stack creates a path for the robot while avoiding obstacles on its path. 
* 'pick_objects' node - Here multple destinations will be provided to robot. 
* 'add_marker' node - It subscribes to the destinations topic published by previous node and takes care of visualization of marker.


## Build and Run
### Terminal 1
* Download and Build the project
```
cd /home/<username>/catkin_ws/src
git clone git@github.com:HarshaPhaneendra/home_service_robot.git
cd ..
catkin_make
source devel/setup.bash
```
* Make sure to give 'execuite' permission to all the script files 
```
chmod +x src/script/home_service.sh
```
* Run 'home_service.sh' shell file 
```
./src/script/home_service.sh
```

