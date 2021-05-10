#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  int No_of_goals = 2;
  // creating 2*3 array
  double goal_coordinates[No_of_goals][3] = {{2.25, 2.25, 1.0}, {1, -1.25, 1.0}};
  /* ROS_INFO("goal coordinates " + std::to_string(No_of_goals[0][0]) ); */

  for (int i = 0; i < No_of_goals; i++)
  {
    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = goal_coordinates[i][0];
    goal.target_pose.pose.position.y = goal_coordinates[i][1];
    goal.target_pose.pose.position.z =  0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = goal_coordinates[i][2];
   

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, robot travelled to goal position");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");

    ros::Duration(5).sleep(); // sleep for 5 sec
  }


  return 0;
}