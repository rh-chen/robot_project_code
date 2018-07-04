#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

main (int argc, char **argv)
{
    ros::init (argc, argv, "showpath");

    ros::NodeHandle ph;
    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory",1, true);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    nav_msgs::Path path;
    //nav_msgs::Path path;
    path.header.stamp=current_time;
    path.header.frame_id="map";




    ros::Rate loop_rate(1);


        current_time = ros::Time::now();

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0);

        geometry_msgs::PoseStamped pose1,pose2,pose3,pose4,pose5,pose6,pose7;
        pose1.pose.position.x = 1;
        pose1.pose.position.y = 0;
        pose1.pose.orientation = goal_quat;
        pose1.header.stamp=current_time;
        pose1.header.frame_id="map";

        pose2.pose.position.x = 1.2;
        pose2.pose.position.y = 0.2;
        pose2.pose.orientation = goal_quat;
        pose2.header.stamp=current_time;
        pose2.header.frame_id="map";


        pose3.pose.position.x = 1.2;
        pose3.pose.position.y = 0.4;
        pose3.pose.orientation = goal_quat;
        pose3.header.stamp=current_time;
        pose3.header.frame_id="map";

        pose4.pose.position.x = 1.2;
        pose4.pose.position.y = 0.6;
        pose4.pose.orientation = goal_quat;
        pose4.header.stamp=current_time;
        pose4.header.frame_id="map";

        pose5.pose.position.x = 1;
        pose5.pose.position.y = 1;
        pose5.pose.orientation = goal_quat;
        pose5.header.stamp=current_time;
        pose5.header.frame_id="map";


        pose6.pose.position.x = 0;
        pose6.pose.position.y = 1;
        pose6.pose.orientation = goal_quat;
        pose6.header.stamp=current_time;
        pose6.header.frame_id="map";



        pose7.pose.position.x = 2;
        pose7.pose.position.y = 0;
        pose7.pose.orientation = goal_quat;
        pose7.header.stamp=current_time;
        pose7.header.frame_id="map";
        path.poses.push_back(pose1);

        path.poses.push_back(pose2);

        path.poses.push_back(pose3);
        path.poses.push_back(pose4);
        path.poses.push_back(pose5);

        path.poses.push_back(pose6);

//        path.poses.push_back(pose7);

        path_pub.publish(path);
        ros::spin();               // check for incoming messages

        last_time = current_time;
        loop_rate.sleep();


    return 0;
}
