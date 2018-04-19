#include <ros/ros.h>
#include <ros/topic.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <ret_chargeable_pile/FindMarkerPoseAction.h>
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

#include <opencv2/core/core.hpp>

#include <string>
#include <iostream>
#include <algorithm> 

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>


#define PI 3.1415926

using namespace tf;
using namespace std;

tf::TransformListener *tf_listener;
std::string camera_frame;
std::string world_frame;
double max_frequency;

bool marker_is_visible = false;
bool is_forward_marker = false;
geometry_msgs::Twist rotate_move_cmd;
geometry_msgs::Twist straight_move_cmd;

double marker_x;
double marker_y;
double marker_z;
double marker_yaw;
double marker_roll;
double marker_pitch;

class robot_control{
	public:
	ros::NodeHandle nh;
	ros::Subscriber msg_sub;
	ros::Publisher cmd_vel_pub;

	geometry_msgs::Twist move_cmd;

	robot_control(ros::NodeHandle n):nh(n)
	{
		msg_sub = nh.subscribe("/ar_pose_marker", 1,&robot_control::poseCallback,this);
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",1);
	}
	void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
	void robot_move_base();

};

	void robot_control::poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
	{
		ROS_INFO("world_frame:%s\n",world_frame.c_str());
#if 1

		if(msg->header.frame_id == world_frame)
		{
			ROS_INFO("world_frame:%s\n",world_frame.c_str());
#if 1
			ar_track_alvar_msgs::AlvarMarker marker = msg->markers[0];
			if(marker.header.frame_id == world_frame+"_2_pose")
			{
				ROS_INFO("world_frame:%s\n",(world_frame+"_2_pose").c_str());
#if 1
				if(marker.pose.header.frame_id == world_frame+"_2")
				{
					ROS_INFO("world_frame:%s\n",(world_frame+"_2").c_str());
					marker_is_visible = true;
#if 1

					tf::StampedTransform CameraToWorld;
    			try{
						tf_listener->waitForTransform(world_frame+"_2", camera_frame, msg->header.stamp, ros::Duration(1.0));
						tf_listener->lookupTransform(world_frame+"_2", camera_frame, msg->header.stamp, CameraToWorld);
   				}
    			catch (tf::TransformException ex){
      				ROS_ERROR("%s",ex.what());
    			}

					marker_x = CameraToWorld.getOrigin().x();
					marker_y = CameraToWorld.getOrigin().y();
					marker_z = CameraToWorld.getOrigin().z();

					CameraToWorld.getBasis().getRPY(marker_roll,marker_pitch,marker_yaw);

					//is_forward_marker
					ROS_INFO("marker_x:%f\n",marker_x);
					ROS_INFO("marker_y:%f\n",marker_y);
					ROS_INFO("marker_z:%f\n",marker_z);
					ROS_INFO("marker_yaw:%f\n",marker_yaw);

					if(fabs(marker_yaw+PI) < 0.02)
						if(fabs(marker_y) < 0.01)
							is_forward_marker = true;
						else
							is_forward_marker = false;
					else
							is_forward_marker = false;
#endif
			}
			else
				marker_is_visible = false;
#endif
		}
		else
			marker_is_visible = false;
#endif
		}
		else
			marker_is_visible = false;
#endif
	}
void robot_control::robot_move_base()
{
#if 0
		if(!first_marker){
			ar_track_alvar_msgs::AlvarMarker marker = msg->markers[0];

			ROS_INFO("marker_id:%d",marker.id);
			ROS_INFO("marker_confidence:%d",marker.confidence);
			ROS_INFO("marker_yaw:%f",marker_yaw);

			first_marker_x = marker.pose.pose.position.x;
			first_marker_y = marker.pose.pose.position.y;
			first_marker_z = marker.pose.pose.position.z;

			geometry_msgs::Quaternion odom_quat = marker.pose.pose.orientation;
			first_marker_yaw = tf::getYaw(odom_quat);

			//stop robot
			//geometry_msgs::Twist move_cmd;
			move_cmd.linear.x = 0;
			move_cmd.linear.y = 0;
			move_cmd.linear.z = 0;
			move_cmd.angular.x = 0;
			move_cmd.angular.y = 0;
			move_cmd.angular.z = 0;

			cmd_vel_pub.publish(move_cmd);

			//turn robot
			double robot_rotation_angle = atan2(first_marker_z-1,first_marker_y);
			double rotation_theta = PI-first_marker_yaw-robot_rotation_angle;

			ROS_INFO("robot_rotation_angle:%e",robot_rotation_angle);
			ROS_INFO("rotation_theta:%e",rotation_theta);

			double rotation_theta = 90*PI/180;

			//geometry_msgs::Twist turn_cmd;
			move_cmd.linear.x = 0;
			move_cmd.linear.y = 0;
			move_cmd.linear.z = 0;
			move_cmd.angular.x = 0;
			move_cmd.angular.y = 0;
			move_cmd.angular.z = rotation_theta/2.0;

			ros::Rate turn_r(5);
			for(int i = 0;i< 10;i++)
			{
				cmd_vel_pub.publish(move_cmd);
				turn_r.sleep();
			}

			//straight robot
			double robot_straight_distance = first_marker_y;
			double vx = 0.2;
			double robot_straight_time = robot_straight_distance/vx;
			int straight_time = (int)(robot_straight_time + 0.05);
	
			ROS_INFO("straight_time:%e",straight_time);
			double vx = 0.2;

			//geometry_msgs::Twist straight_cmd;
			move_cmd.linear.x = vx;
			move_cmd.linear.y = 0;
			move_cmd.linear.z = 0;
			move_cmd.angular.x = 0;
			move_cmd.angular.y = 0;
			move_cmd.angular.z = 0;

			ros::Rate straight_r(5);
			for(int i = 0;i< 10;i++)
			{
				cmd_vel_pub.publish(move_cmd);
				straight_r.sleep();
			}

			//turn cmd
			move_cmd.linear.x = 0;
			move_cmd.linear.y = 0;
			move_cmd.linear.z = 0;
			move_cmd.angular.x = 0;
			move_cmd.angular.y = 0;
			move_cmd.angular.z = rotation_theta/2.0;

			ros::Rate turn_rr(5);
			for(int i = 0;i < 10;i++)
			{
				cmd_vel_pub.publish(move_cmd);
				turn_rr.sleep();
			}
			
			//first_marker = true;
			
			return;
		}	
#endif
	
	}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ret_chargeable_pile");
	ros::NodeHandle n,pn("~");

	robot_control rc(n);

	pn.param<string>("camera_frame", camera_frame, "mynt_left_frame");
  pn.param<string>("world_frame", world_frame, "ar_marker");
	pn.param<double>("max_freq", max_frequency, 10.0);

	//rotate
	rotate_move_cmd.linear.x = 0;
	rotate_move_cmd.linear.y = 0;
	rotate_move_cmd.linear.z = 0;
	rotate_move_cmd.angular.x = 0;
	rotate_move_cmd.angular.y = 0;
	rotate_move_cmd.angular.z = 0.8;

	//straight
	straight_move_cmd.linear.x = 0.2;
	straight_move_cmd.linear.y = 0;
	straight_move_cmd.linear.z = 0;
	straight_move_cmd.angular.x = 0;
	straight_move_cmd.angular.y = 0;
	straight_move_cmd.angular.z = 0;

	tf_listener = new tf::TransformListener(n);

	ros::Rate rate(max_frequency);
	while(ros::ok())
	{
		ROS_INFO("Start Return To Chargeable Pile");
		ros::spinOnce();
		rate.sleep();
				
		/*if (std::abs((rate.expectedCycleTime() - ros::Duration(1.0 / max_frequency)).toSec()) > 0.001)
    {
      ROS_INFO("Changing frequency from %.2f to %.2f", 1.0 / rate.expectedCycleTime().toSec(), max_frequency);
      rate = ros::Rate(max_frequency);
    }*/
		
		if(marker_is_visible)
		{
			ROS_INFO("Marker_Is_Visible");
			if(is_forward_marker)
			{
				ROS_INFO("Is_Forward_Marker and Start Straight Line Move");
				rc.cmd_vel_pub.publish(straight_move_cmd);
			}
			else
			{
				rc.robot_move_base();
				ros::Duration(10).sleep();						
			}		

		}
		else{
			rc.cmd_vel_pub.publish(rotate_move_cmd);
		}
	}

  return 0;
}
