#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include "pick_objects/RobotStatusUpdate.h"

#include <iostream>
#include <string.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void RobotSetGoal(move_base_msgs::MoveBaseGoal& goal, 
  const float& pose_x,
    const float& pose_y,
      const float& pose_orientation)
{
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = pose_x;
  goal.target_pose.pose.position.y = pose_y;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_orientation);

}

bool RobotSendGoal(MoveBaseClient& ac, 
  move_base_msgs::MoveBaseGoal& goal )
{
  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    return true;
  else
    return false;
}

int main(int argc, char** argv)
{
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher movebase_pub = n.advertise<move_base_msgs::MoveBaseGoal>("/pick_up/move_base/goal", 100);
  ros::Publisher robotstatus_pub = n.advertise<pick_objects::RobotStatusUpdate>("/pick_up/robot_status", 100);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal; // movebase ptr

  pick_objects::RobotStatusUpdate msg; // custom msg ptr

  // initialse the msg variables 
  msg.robot_at_pickup_pose = false;
  msg.robot_at_dropoff_pose = false;

  // No of goals coordinates
  int No_of_goals = 2;
  bool condition;

  // creating struct for pose x,y and orientation w
  struct robot_goal
  {
    float pose_x;
    float pose_y;
    float orientation;
  };

  // marker pick_up and drop_off x,y & w coorinates 
  robot_goal marker_pick_up = {+1.5, +4.0, -1.57};
  robot_goal marker_drop_off = {-1.5, +11.0, -1.57};

  for (int i = 0; i < No_of_goals; i++)
  {
      
    switch (i)
    {
      case 0:
        RobotSetGoal(goal, marker_pick_up.pose_x, marker_pick_up.pose_y, marker_drop_off.orientation);
        movebase_pub.publish(goal);
        condition = RobotSendGoal(ac, goal);
        
        if (condition)
        {
          ROS_INFO("Robot has reached PICK_UP pose ...");
          msg.robot_at_pickup_pose = true;
        }
        else
        {
          ROS_INFO("Robot couldn't reach PICK_UP pose !!!");
          msg.robot_at_pickup_pose = false;
        }
        break;

      case 1: 
        RobotSetGoal(goal, marker_drop_off.pose_x, marker_drop_off.pose_y, marker_drop_off.orientation);
        movebase_pub.publish(goal);
        condition = RobotSendGoal(ac, goal);
        
        if (condition)
        {
          ROS_INFO("Robot has reached DROP_OFF pose ...");
          msg.robot_at_dropoff_pose = true;
        }
        else
        {
          ROS_INFO("Robot couldn't reach DROP_OFF pose !!!");
          msg.robot_at_dropoff_pose = false;
        }
        break;
      
      default:
        break;
    }    
    
    robotstatus_pub.publish(msg);
    ros::spinOnce();
    ros::Duration(5).sleep(); // sleep for 5 sec
  }
  
  return 0;
}