#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
/* #include <iostream>
#include <string> */

int main(int argc, char** argv)
{

    ros::init(argc, argv, "visible_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    // causion don't execute both marker nodes at once, since both has smae pub topic name

    visualization_msgs::Marker marker; // local ptr 

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "visible_markers";
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

    // creating struct for pose x,y and orientation w
    struct robot_goal
    {
        float pose_x;
        float pose_y;
        float orientation_w;
    };

    // marker pick_up and drop_off x,y coorinates & orientation.w
    robot_goal marker_pick_up = {+1.5, +4.0, +1.0};
    robot_goal marker_drop_off = {-1.5, +11.0, +1.0};

    /* while(1)
    { */
        ROS_INFO("Loading pose info...") ;
        ros::Duration(5).sleep(); 
        marker.pose.position.x = marker_pick_up.pose_x;
        marker.pose.position.y = marker_pick_up.pose_y;
        marker.pose.orientation.w = marker_pick_up.orientation_w;
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
        ROS_INFO("Marker being published at PICK_UP pose.");
        ros::Duration(5).sleep(); // Retain published marker @ pick_up pose for 5 seconds

        //Hidding marker for next 5 seconds
        marker.action = visualization_msgs::Marker::DELETE; 
        marker_pub.publish(marker);
        ROS_INFO("Marker is being Hidden.");
        ros::Duration(5).sleep(); 

        marker.pose.position.x = marker_drop_off.pose_x;
        marker.pose.position.y = marker_drop_off.pose_y;
        marker.pose.orientation.w = marker_drop_off.orientation_w;
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
        ROS_INFO("Marker being published at DROP_OFF pose.");
        //ros::Duration(15).sleep();


        ros::spinOnce();
        ros::Duration(5).sleep();


    //}
    return 0;
}

