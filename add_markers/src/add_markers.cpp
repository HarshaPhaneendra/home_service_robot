#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseGoal.h>
//#include "add_markers/RobotStatusInfo.h" 
#include "pick_objects/RobotStatusUpdate.h"

#include <iostream>
#include <string>

// marker pose struct 
struct pose
{
    float x;
    float y;
};

//robot status info struct 
struct robot_info
{
    bool robot_at_pickup_pose;
    bool robot_at_dropoff_pose;
};

pose move_base;
robot_info robot_status;

void movebase_callback(const move_base_msgs::MoveBaseGoal::ConstPtr& msg)
{
    move_base.x = msg->target_pose.pose.position.x;
    move_base.y = msg->target_pose.pose.position.y;
}

void robotstatus_callback(const pick_objects::RobotStatusUpdate::ConstPtr& msg)
{
    robot_status.robot_at_pickup_pose = msg->robot_at_pickup_pose;
    robot_status.robot_at_dropoff_pose = msg->robot_at_dropoff_pose;
}
 
int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    
    ros::Subscriber movebase_sub = n.subscribe("/pick_up/move_base/goal", 1000, movebase_callback);
    ros::Subscriber robotstatus_sub = n.subscribe("/pick_up/robot_status", 1000, robotstatus_callback);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
      
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;
    
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    while (ros::ok())
    {

        if( robot_status.robot_at_pickup_pose == false &&
         robot_status.robot_at_dropoff_pose == false)
        {
            marker.pose.position.x = move_base.x; 
            marker.pose.position.y = move_base.y;
            marker.pose.orientation.w = 1.0; 
            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker);
            ROS_INFO("Virtual Marker at Pick_up is being published. "); 
            std::cout << "marker_pickup_pose_x: " << move_base.x << " marker_pickup_pose_y: " << move_base.y << std::endl;       
        }
        
        if (robot_status.robot_at_pickup_pose == true &&
         robot_status.robot_at_dropoff_pose == false)
        {
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);
            ROS_INFO("Marker is Hidden."); 
        }

        if (robot_status.robot_at_dropoff_pose == true)
        {
            marker.pose.position.x = move_base.x; 
            marker.pose.position.y = move_base.y;
            marker.pose.orientation.w = 1.0; 
            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker);
            ROS_INFO("Virtual Marker is Droped_Off. ");
            std::cout << "marker_pickup_pose_x: " << move_base.x << " marker_pickup_pose_y: " << move_base.y << std::endl;
        }
        
        ros::spinOnce();
        r.sleep();

    }
    return 0;

}