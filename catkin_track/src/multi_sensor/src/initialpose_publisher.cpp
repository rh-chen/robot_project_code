#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

geometry_msgs::PoseWithCovarianceStamped pose ;
bool poseInitialed = false;
bool hasPoseInitialed = false;

double offsetX, offsetY, offsetYaw;

tf::StampedTransform tf_odom_base;

void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    pose = *msg;
    poseInitialed = true;
   



        //got current offset, and inverse it
        tf::TransformListener listener;

	listener.waitForTransform("base_footprint", "odom_combined", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base_footprint", "odom_combined", ros::Time(0), tf_odom_base);


}


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;

  ros::Rate r(100);

  ros::Subscriber sub = node.subscribe("initialpose", 1, initialPoseReceived);


  tf::Quaternion q;
  //q.setRPY(0,0,0);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0,0,0));
  q.setRPY(0,0,0);


  transform.setRotation(q);

  static tf::TransformBroadcaster br;

  while(ros::ok()){
    if (false == poseInitialed){
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom_ekf"));
    }else{
	    tf::Transform transform;
	    tf::Pose  pose_new;
	    tf::Quaternion q;

	    tf::poseMsgToTF(pose.pose.pose, pose_new);

	    transform.setOrigin( tf::Vector3(pose.pose.pose.position.x, pose.pose.pose.position.y , 0.0) );
	    q.setRPY(0,0,tf::getYaw(pose_new.getRotation()));

	    transform.setRotation(q);

	    static tf::TransformBroadcaster br;

	    br.sendTransform(tf::StampedTransform(transform * tf_odom_base, ros::Time::now(), "map", "odom_ekf"));
  	std::cout << tf_odom_base.getOrigin().x() <<"  " << tf_odom_base.getOrigin().y()<<std::endl;
    }
    ros::spinOnce();

  }




  return 0;
}
