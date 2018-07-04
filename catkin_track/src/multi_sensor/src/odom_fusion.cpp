#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kobuki_msgs/CliffEvent.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <kobuki_msgs/WheelDropEvent.h>
nav_msgs::Odometry vio_msg_old;
geometry_msgs::PoseWithCovarianceStamped odom_msg_old;
kobuki_msgs::CliffEvent cliff;
tf::Transform odom_pose,odom_pose_old,vio_pose,vio_pose_old,vio_diff,pose_fusion,odom_diff;

tf::Transform static_tf_odom,static_tf_vio,static_tf;


//pose1.inverse() * pose2 的意思是pose2-pose1，    pose1 * pose2的意思是pose1+pose2 ,  .inverseTimes  = .inverse() * 取逆 然後相承
void odomReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{



    tf::poseMsgToTF(msg->pose.pose,odom_pose);

    static int i =0;
    if(i == 0)
    {
        odom_msg_old = *msg;
        pose_fusion = odom_pose;
        i++;
        return;
    }

    tf::poseMsgToTF(odom_msg_old.pose.pose,odom_pose_old);

    odom_pose = static_tf_odom.inverse() * odom_pose;
    odom_pose_old = static_tf_odom.inverse() * odom_pose_old;

            odom_diff =odom_pose_old.inverse() *  odom_pose;

            if(cliff.state == 0)
            {
            pose_fusion =  pose_fusion * odom_diff    ;
            std::cout << pose_fusion.getOrigin().getX() << " " << pose_fusion.getOrigin().getY() <<std::endl;
            }



            odom_msg_old = *msg;




}


void vioOdomReceived(const nav_msgs::OdometryConstPtr &msg)
{
    tf::poseMsgToTF(msg->pose.pose,vio_pose);

    static int i =0;
    if(i == 0)
    {
        vio_msg_old = *msg;
        pose_fusion = odom_pose;
        i++;
        return;
    }

    tf::poseMsgToTF(vio_msg_old.pose.pose,vio_pose_old);

    vio_pose = static_tf_vio * vio_pose;
    vio_pose_old = static_tf_vio * vio_pose_old;



            vio_diff =vio_pose_old.inverse() *  vio_pose;

            if(cliff.state == 1)
            {
            pose_fusion =  pose_fusion * vio_diff    ;
            std::cout << pose_fusion.getOrigin().getX() << " " << pose_fusion.getOrigin().getY() <<std::endl;
            }



            vio_msg_old = *msg;

}



void cliffEventsReceived(const kobuki_msgs::CliffEventConstPtr &msg)
{
    cliff = *msg;
}

int main(int argc, char** argv){



    ros::init(argc, argv, "odometry_sub");
    ros::NodeHandle n;

    tf::Quaternion q;

    static_tf_odom = tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0));
    static_tf_vio.setOrigin(tf::Vector3(0.17,0,0));
    q.setRPY(-3.1416,0,0);
    static_tf_vio.setRotation(q);


    ros::Subscriber cliff_sub = n.subscribe("/mobile_base/events/cliff", 1000, cliffEventsReceived);
    ros::Subscriber odom_sub = n.subscribe("/robot_pose_ekf/odom_combined", 1000, odomReceived);
    ros::Subscriber vio_sub = n.subscribe("/vins_estimator/odometry_shadow", 1000, vioOdomReceived);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_fusion", 50);
    tf::TransformBroadcaster odom_broadcaster;



    ros::Time current_time;
    current_time = ros::Time::now();

    ros::Rate r(30.0);


    while(n.ok()){
        current_time = ros::Time::now();

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat;
        odom_quat.x = pose_fusion.getRotation().getX();
        odom_quat.y = pose_fusion.getRotation().getY();
        odom_quat.z = pose_fusion.getRotation().getZ();
        odom_quat.w = pose_fusion.getRotation().getW();
        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom_ekf";
        odom_trans.child_frame_id = "odom_combined";


        odom_trans.transform.translation.x = pose_fusion.getOrigin().getX();
        odom_trans.transform.translation.y = pose_fusion.getOrigin().getY();
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;


          odom.header.stamp = current_time;
          odom.header.frame_id = "odom_ekf";

          odom.pose.pose.position.x = pose_fusion.getOrigin().getX();
          odom.pose.pose.position.y = pose_fusion.getOrigin().getY();
          odom.pose.pose.position.z = 0.0;
          odom.pose.pose.orientation = odom_quat;
          odom.child_frame_id = "odom_combined";
          odom.twist.twist.linear.x = 0;
          odom.twist.twist.linear.y = 0;
          odom.twist.twist.angular.z = 0;

          odom_pub.publish(odom);


    ros::spinOnce();
    r.sleep();
    }

    ros::spin();

      return 0;
}
