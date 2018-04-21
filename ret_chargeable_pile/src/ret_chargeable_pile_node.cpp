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

int rotation_direction = 0;

bool marker_is_visible = false;
bool is_forward_marker = false;
bool is_position_unsuitable = false;
bool is_angle_unsuitable = false;

geometry_msgs::Twist rotate_move_cmd;
geometry_msgs::Twist straight_move_cmd;
geometry_msgs::Twist stop_move_cmd;
geometry_msgs::Twist common_move_cmd;

double marker_x;
double marker_y;
double marker_z;
double alter_marker_x;
double alter_marker_y;
double alter_marker_z;

double marker_yaw;
double marker_roll;
double marker_pitch;

double position_threshold = 0.03;
double angle_threshold = 0.04;
double angular_tolerance = 2.5*PI/180; 
double rate_frequency = 100;

double wz = 0.6;
double vx = 0.2;

string odom_frame = "/odom";
string base_frame = "/base_link";

class robot_control{
	public:
	ros::NodeHandle nh;
	ros::Subscriber msg_sub;
	ros::Publisher cmd_vel_pub;

	geometry_msgs::Twist move_cmd;

	robot_control(ros::NodeHandle n):nh(n)
	{
		msg_sub = nh.subscribe("/ar_pose_marker", 1000,&robot_control::poseCallback,this);
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",1000);
	}
	void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
	void robot_move_base();
	void rotate_n_angle(double n,int direction);
	void straight_n_distance(double n);
};


	void robot_control::rotate_n_angle(double n,int direction)
	{
		//rotation direction
		if(direction  < 0)
		{	
			common_move_cmd.linear.x = 0;
			common_move_cmd.linear.y = 0;
			common_move_cmd.linear.z = 0;
			common_move_cmd.angular.x = 0;
			common_move_cmd.angular.y = 0;
			common_move_cmd.angular.z = -wz;
		}
		else
		{
			common_move_cmd.linear.x = 0;
			common_move_cmd.linear.y = 0;
			common_move_cmd.linear.z = 0;
			common_move_cmd.angular.x = 0;
			common_move_cmd.angular.y = 0;
			common_move_cmd.angular.z = wz;
		}
		//get odom data
		tf::StampedTransform OdomToBaselink;
		try
      {
        tf_listener->waitForTransform(odom_frame, base_frame, ros::Time::now(), ros::Duration(2.0) );
				tf_listener->lookupTransform(odom_frame, base_frame, ros::Time::now(), OdomToBaselink);
      }
    catch (tf::TransformException ex)
      {
          ROS_INFO("Cannot find transform between /odom and /base_link");
          cmd_vel_pub.publish(geometry_msgs::Twist());
          ros::shutdown();
      }

		double last_angle = fabs(tf::getYaw(OdomToBaselink.getRotation()));
   	double turn_angle = 0;

		ros::Rate loopRate(rate_frequency);
    while( (fabs(turn_angle + angular_tolerance) < n) && (ros::ok()) )
    {
        //Publish the Twist message and sleep 1 cycle
        cmd_vel_pub.publish(common_move_cmd);
        loopRate.sleep();
        // Get the current rotation
        tf_listener->lookupTransform(odom_frame, base_frame, ros::Time::now(), OdomToBaselink);
     
        double rotation = fabs(tf::getYaw(OdomToBaselink.getRotation()));

        //Compute the amount of rotation since the last loop
        double delta_angle = fabs(rotation - last_angle);

        //Add to the running total
        turn_angle += delta_angle;

        last_angle = rotation;
    }
		cmd_vel_pub.publish(geometry_msgs::Twist());
	}

	void robot_control::straight_n_distance(double n)
	{
		//set veloctiy
		common_move_cmd.linear.x = vx;
		common_move_cmd.linear.y = 0;
		common_move_cmd.linear.z = 0;
		common_move_cmd.angular.x = 0;
		common_move_cmd.angular.y = 0;
		common_move_cmd.angular.z = 0;

		//get odom data
		tf::StampedTransform OdomToBaselink;
		try
      {
        tf_listener->waitForTransform(odom_frame, base_frame, ros::Time::now(), ros::Duration(2.0) );
				tf_listener->lookupTransform(odom_frame, base_frame, ros::Time::now(), OdomToBaselink);
      }
    catch (tf::TransformException ex)
      {
          ROS_INFO("Cannot find transform between /odom and /base_link");
          cmd_vel_pub.publish(geometry_msgs::Twist());
          ros::shutdown();
      }

		ros::Rate loopRate(rate_frequency);

		float x_start = OdomToBaselink.getOrigin().x();
    float y_start = OdomToBaselink.getOrigin().y();

    // Keep track of the distance traveled
    float distance = 0;
    while( (distance < n) && (ros::ok()) )
    {
         //Publish the Twist message and sleep 1 cycle
         cmd_vel_pub.publish(common_move_cmd);
         loopRate.sleep();
         tf_listener->lookupTransform(odom_frame, base_frame, ros::Time::now(), OdomToBaselink);
         //Get the current position
         float x = OdomToBaselink.getOrigin().x();
         float y = OdomToBaselink.getOrigin().y();
         //Compute the Euclidean distance from the start
         distance = sqrt(pow((x - x_start), 2) +  pow((y - y_start), 2));
    }
    //Stop the robot before the rotation
    cmd_vel_pub.publish(geometry_msgs::Twist());
	}

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
					is_forward_marker = false;
					
					ROS_INFO("world_frame:%s\n",(world_frame+"_2").c_str());
					marker_is_visible = true;
#if 1

					tf::StampedTransform CameraToWorld;
    			try{
						tf_listener->waitForTransform(world_frame+"_2", camera_frame, msg->header.stamp, ros::Duration(2.0));
						tf_listener->lookupTransform(world_frame+"_2", camera_frame, msg->header.stamp, CameraToWorld);
   				}
    			catch (tf::TransformException ex){
							ROS_ERROR("%s",ex.what());
							cmd_vel_pub.publish(geometry_msgs::Twist());
							ros::shutdown();
    			}

					marker_x = CameraToWorld.getOrigin().x();
					marker_y = CameraToWorld.getOrigin().y();
					marker_z = CameraToWorld.getOrigin().z();

					CameraToWorld.getBasis().getRPY(marker_roll,marker_pitch,marker_yaw);

					//is_forward_marker
					ROS_INFO("marker_x:%f\n",marker_x);
					ROS_INFO("marker_y:%f\n",marker_y);
					ROS_INFO("marker_z:%f\n",marker_z);
					ROS_INFO("marker_pitch:%f\n",marker_pitch);
				
					alter_marker_x = marker_x;
					alter_marker_y = marker_y - 0.06*cos(marker_pitch);
					alter_marker_z = marker_z - 0.06*sin(marker_pitch);

					ROS_INFO("alter_marker_x:%f\n",alter_marker_x);
					ROS_INFO("alter_marker_y:%f\n",alter_marker_y);
					ROS_INFO("alter_marker_z:%f\n",alter_marker_z);
					ROS_INFO("marker_pitch:%f\n",marker_pitch);				
				
					if(fabs(alter_marker_y) < position_threshold)
					{
						if(fabs(marker_pitch) < angle_threshold)
						{
							is_forward_marker = true;
							is_angle_unsuitable = false;
							is_position_unsuitable = false;
						}
						else
						{
							is_forward_marker = false;
							is_position_unsuitable = false;
							is_angle_unsuitable = true;
						}

					}
					else
					{
						if(fabs(marker_pitch) < angle_threshold)
						{
							is_forward_marker = false;
							is_position_unsuitable = true;
							is_angle_unsuitable = false;
						}
						else
						{
							is_forward_marker = false;
							is_position_unsuitable = true;			
							is_angle_unsuitable = true;
						}
					}					
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
		ROS_INFO("is_position_unsuitable:%d",is_position_unsuitable);
		ROS_INFO("is_angle_unsuitable:%d",is_angle_unsuitable);
#if 1
			if(alter_marker_y > position_threshold)
					rotation_direction = -1;//顺时针
			else if(alter_marker_y < -position_threshold)
					rotation_direction = 1;//逆时针
					
			if(is_position_unsuitable)
			{
				if(alter_marker_y > position_threshold)
				{
					rotate_n_angle(PI/2+marker_pitch,rotation_direction);
					ros::Duration(1.0).sleep();
					straight_n_distance(alter_marker_y);
					ros::Duration(1.0).sleep();
					rotate_n_angle(PI/2,-rotation_direction);
					ros::Duration(1.0).sleep();
				}
				else if(alter_marker_y < -position_threshold)
				{
					rotate_n_angle(PI/2-marker_pitch,rotation_direction);
					ros::Duration(1.0).sleep();
					straight_n_distance(alter_marker_y);
					ros::Duration(1.0).sleep();
					rotate_n_angle(PI/2,-rotation_direction);
					ros::Duration(1.0).sleep();
				}
			}
			else if(!is_position_unsuitable && is_angle_unsuitable)
			{
				if(marker_pitch < 0)
				{
					rotate_n_angle(fabs(marker_pitch),1);
					ros::Duration(1.0).sleep();
				}
				else
				{
					rotate_n_angle(fabs(marker_pitch),-1);
					ros::Duration(1.0).sleep();
				}		
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
	rotate_move_cmd.angular.z = -0.8;

	//straight
	straight_move_cmd.linear.x = 0.2;
	straight_move_cmd.linear.y = 0;
	straight_move_cmd.linear.z = 0;
	straight_move_cmd.angular.x = 0;
	straight_move_cmd.angular.y = 0;
	straight_move_cmd.angular.z = 0;

	//stop
	stop_move_cmd.linear.x = 0;
	stop_move_cmd.linear.y = 0;
	stop_move_cmd.linear.z = 0;
	stop_move_cmd.angular.x = 0;
	stop_move_cmd.angular.y = 0;
	stop_move_cmd.angular.z = 0;

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
			rc.cmd_vel_pub.publish(stop_move_cmd);
			ROS_INFO("Marker_Is_Visible");
			ros::Duration(1.0).sleep();
			if(is_forward_marker)
			{
				ROS_INFO("Is_Forward_Marker and Start Straight Line Move");
				ros::Duration(3.0).sleep();
				rc.cmd_vel_pub.publish(straight_move_cmd);
			}
			else
			{
				ROS_INFO("Not Is_Forward_Marker and Alter Robot Position and Direction");
				ros::Duration(1.0).sleep();		
				rc.robot_move_base();				
			}		

		}
		else{
			rc.cmd_vel_pub.publish(rotate_move_cmd);
		}
	}

  return 0;
}
