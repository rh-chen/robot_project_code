#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kobuki_msgs/CliffEvent.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <kobuki_msgs/WheelDropEvent.h>
nav_msgs::Odometry odom_msg_old,vio_msg_old;

double odom_dx,odom_dy,odom_dth,vio_dx,vio_dy,vio_dth,yaw_new,yaw_old,odom_x,odom_y;


geometry_msgs::PoseStamped vio_pose;



tf::Pose  pose_new,pose_old,pose_diff,pose_raw;
tf::Transform transform,transform_old;


void odomReceived(const nav_msgs::OdometryConstPtr &msg)

{


static int ddd = 0;
if (ddd == 0){
    ddd++;
    odom_msg_old = *msg;

    return;
}


    tf::Pose pose,pose_old,pose_diff,pose_raw;
    tf::poseMsgToTF(msg->pose.pose,pose);
    tf::poseMsgToTF(odom_msg_old.pose.pose,pose_old);

    pose_diff =pose_old.inverse() *  pose;

    pose_raw = pose_old * pose_diff;






  // std::cout << pose.getOrigin().getX()-pose_old.getOrigin().getX() << " " << pose.getOrigin().getY() - pose_old.getOrigin().getY() <<std::endl;
   //std::cout << odom_dx << " " << odom_dy <<std::endl;
      std::cout << pose_raw.getOrigin().getX() << " " << pose_raw.getOrigin().getY() <<std::endl;

    odom_msg_old = *msg;
}






int main(int argc, char** argv){

    ros::init(argc, argv, "odometry_sub");

    ros::NodeHandle n;

    ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odomReceived);



    ros::Time current_time;
    current_time = ros::Time::now();

    ros::spin();

      return 0;
}

