#include <ros/ros.h>
#include <ros/topic.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>


#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <kobuki_msgs/BumperEvent.h>

#include <opencv2/core/core.hpp>

#include <string>
#include <iostream>
#include <algorithm> 

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define N_MP 20

using namespace tf;
using namespace std;

tf::TransformListener *tf_listener;

typedef struct mp{
	double x;
	double y;
	double z;
	tf::Transform t;

	mp(double x_,double y_,double z_,tf::Transform t_)
	{x = x_;y = y_;z = z_;t = t_;}

}marker_pose;

std::vector<marker_pose> temp_marker_pose;


double wz_ = 0.5;
double vx_ = 0.12;

Eigen::Matrix3d R;
Eigen::Vector3d t;

std::string world_frame = "/base_link";
std::string marker_msg_frame_id = "ar_marker";

class calculate_global_pose{
	public:
	ros::NodeHandle nh;
	ros::Subscriber sub_robot_global_pose_;
	ros::Subscriber sub_marker_msg_;
	ros::Publisher pub_marker_global_pose_;
	ros::Publisher pub_vel_;

    boost::mutex mtx;

	calculate_global_pose(ros::NodeHandle n):nh(n)
	{
		sub_marker_msg_ = nh.subscribe("/ar_pose_marker", 30,&calculate_global_pose::getMarkerMsgCallback,this);
        sub_robot_global_pose_ = nh.subscribe("/odom",30,&calculate_global_pose::getRobotGlobalPoseCallback,this);
		pub_marker_global_pose_ = nh.advertise<geometry_msgs::Pose>("/marker_global_pose",30);
		pub_vel_ = nh.advertise<geometry_msgs::Twist>("/mynt_cmd_vel_mux/input/keyop",30);
	}

	void getMarkerMsgCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
    void getRobotGlobalPoseCallback(const nav_msgs::Odometry& msg);
};


	void calculate_global_pose::getRobotGlobalPoseCallback(const nav_msgs::Odometry& msg ){
		Eigen::Quaterniond q(msg.pose.pose.orientation.x,\
							 msg.pose.pose.orientation.y,\
							 msg.pose.pose.orientation.z,\
							 msg.pose.pose.orientation.w);

		R = q.toRotationMatrix();

		t(0,0) = msg.pose.pose.position.x;
		t(1,0) = msg.pose.pose.position.y;
		t(2,0) = msg.pose.pose.position.z;
	}

	void calculate_global_pose::getMarkerMsgCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
	{	
        mtx.lock();
		
		if((msg->header.frame_id == marker_msg_frame_id) && (msg->markers.size() > 0)){
			ar_track_alvar_msgs::AlvarMarker marker = msg->markers[0];

			std::string marker_frame = marker.pose.header.frame_id;
			ROS_INFO_STREAM("marker_frame_id:" << marker_frame);
			tf::StampedTransform localPoseTransform;;
    		try{
					tf_listener->waitForTransform(world_frame,marker_frame, ros::Time(0), ros::Duration(1.0));
					tf_listener->lookupTransform(world_frame,marker_frame, ros::Time(0), localPoseTransform);	
   			}
    		catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				pub_vel_.publish(geometry_msgs::Twist());
				ros::shutdown();
    		}


			tf::Transform CToM = localPoseTransform.inverse();
	
			
			double marker_x_ = CToM.getOrigin().x();
			double marker_y_ = CToM.getOrigin().y();
			double marker_z_ = CToM.getOrigin().z();

			double roll_,pitch_,yaw_;
			CToM.getBasis().getRPY(roll_,pitch_,yaw_);

			ROS_INFO_STREAM("marker_x_:" << marker_x_);
			ROS_INFO_STREAM("marker_y_:" << marker_y_);
			ROS_INFO_STREAM("marker_z_:" << marker_z_);
			ROS_INFO_STREAM("marker_roll_:" << roll_);
			ROS_INFO_STREAM("marker_pitch_:" << pitch_);
			ROS_INFO_STREAM("marker_yaw_:" << yaw_);
					
			if(temp_marker_pose.size() < N_MP){
				temp_marker_pose.push_back(marker_pose(marker_x_,marker_y_,marker_z_,CToM));
				ROS_INFO_STREAM("store marker pose");
				mtx.unlock();
				return;
			}
			else{
				Eigen::Matrix3d R_res;
				Eigen::Vector3d t_res;

				for(int i = 0;i < temp_marker_pose.size();i++){
					t_res(0,0) += temp_marker_pose[i].t.getOrigin().x();
					t_res(1,0) += temp_marker_pose[i].t.getOrigin().y();
					t_res(2,0) += temp_marker_pose[i].t.getOrigin().z();

					tf::Matrix3x3 rotation_matrix = temp_marker_pose[i].t.getBasis();
					R_res(0,0) += rotation_matrix[0][0];
					R_res(0,1) += rotation_matrix[0][1];
					R_res(0,2) += rotation_matrix[0][2];
					R_res(1,0) += rotation_matrix[1][0];
					R_res(1,1) += rotation_matrix[1][1];
					R_res(1,2) += rotation_matrix[1][2];
					R_res(2,0) += rotation_matrix[2][0];
					R_res(2,1) += rotation_matrix[2][1];
					R_res(2,2) += rotation_matrix[2][2];
				}
				
				t_res *= (1.0/N_MP);
				R_res *= (1.0/N_MP);
				
				Eigen::Matrix3d Rwc(R*R_res);
				Eigen::Vector3d twc(R*t_res+t);
				ROS_INFO_STREAM("Rwc:" << Rwc);
				ROS_INFO_STREAM("twc:" << twc);
			}
			mtx.unlock();
		}
		else{
			geometry_msgs::Twist velocity;
			velocity.angular.z = wz_;

			pub_vel_.publish(velocity);
		mtx.unlock();
		}
	}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calculate_global_pose_node");
	ros::NodeHandle n;

	calculate_global_pose cgp(n);
	tf_listener = new tf::TransformListener(n);
	
	ros::spin();
  	return 0;
}
