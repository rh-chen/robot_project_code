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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <calcalate_global_marker/GetMarkerGlobalPose.h>

#define N_MP 1


using namespace tf;
using namespace std;

tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;

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

std::string world_frame = "map";
std::string marker_msg_frame_id = "ar_marker";

visualization_msgs::Marker createMarker(const std::string markerName,
									uint32_t type, 
									geometry_msgs::Pose pose, 
									geometry_msgs::Vector3 scale, 
									std_msgs::ColorRGBA color,
									int32_t id, 
									std::string frame_id = std::string("s_map"))
	{

			//marker start point
			visualization_msgs::Marker marker;
			marker.header.frame_id = frame_id;
			marker.header.stamp = ros::Time();
			marker.ns = "marker_" + markerName;
			marker.id = id;
			marker.type = type;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = pose.position.x;
			marker.pose.position.y = pose.position.y;
			marker.pose.position.z = pose.position.z;
			marker.pose.orientation.x = pose.orientation.x;
			marker.pose.orientation.y = pose.orientation.y;
			marker.pose.orientation.z = pose.orientation.z;
			marker.pose.orientation.w = pose.orientation.w;
			marker.scale.x = scale.x;
			marker.scale.y = scale.y;
			marker.scale.z = scale.z;
			marker.color.a = color.a;
			marker.color.r = color.r;
			marker.color.g = color.g;
			marker.color.b = color.b;

			return marker;
	}

class calculate_global_pose{
	public:
	ros::NodeHandle nh;
	ros::Subscriber sub_robot_global_pose_;
	ros::Subscriber sub_marker_msg_;
	ros::Publisher pub_marker_global_pose_;
	ros::Publisher pub_vel_;
	ros::Publisher marker_pub;
	bool markerVisible;
	visualization_msgs::Marker markerSphere;

	geometry_msgs::PoseStamped goal;
	geometry_msgs::PoseStamped pre_goal;

	std::vector<marker_pose> temp_marker_pose;
    boost::mutex mtx;

	calculate_global_pose(ros::NodeHandle n):nh(n)
	{
		markerVisible = false;
		sub_marker_msg_ = nh.subscribe("/ar_pose_marker", 30,&calculate_global_pose::getMarkerMsgCallback,this);
		pub_marker_global_pose_ = nh.advertise<geometry_msgs::Pose>("/marker_global_pose",30);
		pub_vel_ = nh.advertise<geometry_msgs::Twist>("/mynt_cmd_vel_mux/input/keyop",30);
		marker_pub = n.advertise<visualization_msgs::Marker>("/forward_marker_pose", 30);
	}

	void getMarkerMsgCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
	bool Loop(calcalate_global_marker::GetMarkerGlobalPose::Request &req,\
			  calcalate_global_marker::GetMarkerGlobalPose::Response &res);
};


	/*void calculate_global_pose::getRobotGlobalPoseCallback(const nav_msgs::Odometry& msg ){
		Eigen::Quaterniond q(msg.pose.pose.orientation.x,\
							 msg.pose.pose.orientation.y,\
							 msg.pose.pose.orientation.z,\
							 msg.pose.pose.orientation.w);

		R = q.toRotationMatrix();

		t(0,0) = msg.pose.pose.position.x;
		t(1,0) = msg.pose.pose.position.y;
		t(2,0) = msg.pose.pose.position.z;
	}*/

	void calculate_global_pose::getMarkerMsgCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
	{	
        mtx.lock();
		//static int id  = 0;
		std::string marker_frame;
		
		if(markerVisible){
			ROS_INFO_STREAM("Marker Visible...");
			//marker_pub.publish(markerSphere);
			mtx.unlock();
			return;
		}

		if((msg->header.frame_id == marker_msg_frame_id) && (msg->markers.size() > 0)){
			ar_track_alvar_msgs::AlvarMarker marker = msg->markers[0];

			marker_frame = marker.pose.header.frame_id;
			ROS_INFO_STREAM("marker_frame_id:" << marker_frame);

			tf::StampedTransform preLocalPoseTransform;;
    		try{
					tf_listener->waitForTransform(world_frame,marker_frame, ros::Time(0), ros::Duration(1.0));
					tf_listener->lookupTransform(world_frame,marker_frame,ros::Time(0), preLocalPoseTransform);	
   			}
    		catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				pub_vel_.publish(geometry_msgs::Twist());
				ros::shutdown();
    		}

			tf::StampedTransform localPoseTransform;;
    		try{
					tf_listener->waitForTransform(world_frame,marker_frame+"_forward", ros::Time(0), ros::Duration(1.0));
					tf_listener->lookupTransform(world_frame,marker_frame+"_forward",ros::Time(0), localPoseTransform);	
   			}
    		catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				pub_vel_.publish(geometry_msgs::Twist());
				ros::shutdown();
    		}

			//tf::Transform CToM = localPoseTransform.inverse();
			
			double marker_x_ = localPoseTransform.getOrigin().x();
			double marker_y_ = localPoseTransform.getOrigin().y();
			double marker_z_ = localPoseTransform.getOrigin().z();

			double roll_,pitch_,yaw_;
			localPoseTransform.getBasis().getRPY(roll_,pitch_,yaw_);

			//ROS_INFO_STREAM("be_marker_x_:" << marker.pose.pose.position.x);
			//ROS_INFO_STREAM("be_marker_y_:" << marker.pose.pose.position.y);
			//ROS_INFO_STREAM("be_marker_z_:" << marker.pose.pose.position.z);
			

			if(temp_marker_pose.size() < N_MP){
				temp_marker_pose.push_back(marker_pose(marker_x_,marker_y_,marker_z_,localPoseTransform));
				ROS_INFO_STREAM("store marker pose");
				mtx.unlock();
				return;
			}
			else{
				tf::Vector3 marker_position(0,0,0);
				tf::Vector3 marker_rpy(0,0,0);

				for(int i = 0;i < temp_marker_pose.size();i++){
					double marker_roll = 0;
					double marker_pitch = 0;
					double marker_yaw = 0;

					marker_position[0] += temp_marker_pose[i].t.getOrigin().x();
					marker_position[1] += temp_marker_pose[i].t.getOrigin().y();
					marker_position[2] += temp_marker_pose[i].t.getOrigin().z();

					temp_marker_pose[i].t.getBasis().getRPY(marker_roll,marker_pitch,marker_yaw);
					marker_rpy[0] += marker_roll;
					marker_rpy[1] += marker_pitch;
					marker_rpy[2] += marker_yaw;
				}
			
				int marker_count = temp_marker_pose.size();
				ROS_INFO_STREAM("marker_count:" << marker_count);

				marker_position[0] /= marker_count;
				marker_position[1] /= marker_count;
				marker_position[2] /= marker_count;
				ROS_INFO_STREAM("marker_position[0]:" << marker_position[0]);
				ROS_INFO_STREAM("marker_position[1]:" << marker_position[1]);
				ROS_INFO_STREAM("marker_position[2]:" << marker_position[2]);

				marker_rpy[0] /= marker_count;
				marker_rpy[1] /= marker_count;
				marker_rpy[2] /= marker_count;
				ROS_INFO_STREAM("marker_rpy[0]:" << marker_rpy[0]);
				ROS_INFO_STREAM("marker_rpy[1]:" << marker_rpy[1]);
				ROS_INFO_STREAM("marker_rpy[2]:" << marker_rpy[2]);

				tf::Quaternion q;
				q.setRPY(marker_rpy[0],marker_rpy[1],marker_rpy[2]);
				tf::Transform t(q,marker_position);

				/*tf::Quaternion qq(marker.pose.pose.orientation.x,\
								  marker.pose.pose.orientation.y,\
								  marker.pose.pose.orientation.z,\
								  marker.pose.pose.orientation.w);
				tf::Vector3 markerOrigin (marker.pose.pose.position.x,marker.pose.pose.position.y,marker.pose.pose.position.z-1.0);	
				tf::Transform m(qq, markerOrigin);
				*/

				tf::Transform p = t;
				/*
				tf::StampedTransform markerForwardToWorld(t, msg->header.stamp, world_frame, marker_frame);
				tf_broadcaster->sendTransform(markerForwardToWorld);
				*/

				/*geometry_msgs::Pose  pose;
				pose.position.x = t.getOrigin().x();
				pose.position.y = t.getOrigin().y();
				pose.position.z = t.getOrigin().z();

				pose.orientation.w = 1.0;

				geometry_msgs::Vector3 scale;
				scale.x = 0.05;
				scale.y = 0.05;
				scale.z = 0.05;

				std_msgs::ColorRGBA color;
				color.a = 1.0;
				color.r = 1.0;

				ROS_INFO_STREAM("marker_id:" << id);
				markerSphere = createMarker("markerSphere",
											visualization_msgs::Marker::SPHERE,
											pose,
											scale,
											color,
											id,
											marker.pose.header.frame_id);
				marker_pub.publish(markerSphere);
				id++;*/
				markerVisible = true;
				
				goal.header.stamp = marker.pose.header.stamp;
				goal.header.frame_id = "map";
				goal.pose.position.x = p.getOrigin().x();
				goal.pose.position.y = p.getOrigin().y();
				goal.pose.position.z = p.getOrigin().z();
				goal.pose.orientation.x = p.getRotation().getX();
				goal.pose.orientation.y = p.getRotation().getY();
				goal.pose.orientation.z = p.getRotation().getZ();
				goal.pose.orientation.w = p.getRotation().getW();

				ROS_INFO_STREAM("marker_x_:" << p.getOrigin().x());
				ROS_INFO_STREAM("marker_y_:" << p.getOrigin().y());
				ROS_INFO_STREAM("marker_z_:" << p.getOrigin().z());
				pre_goal.header.stamp = marker.pose.header.stamp;
				pre_goal.header.frame_id = "map";
				pre_goal.pose.position.x = preLocalPoseTransform.getOrigin().x();
				pre_goal.pose.position.y = preLocalPoseTransform.getOrigin().y();
				pre_goal.pose.position.z = preLocalPoseTransform.getOrigin().z();
				pre_goal.pose.orientation.x = preLocalPoseTransform.getRotation().getX();
				pre_goal.pose.orientation.y = preLocalPoseTransform.getRotation().getY();
				pre_goal.pose.orientation.z = preLocalPoseTransform.getRotation().getZ();
				pre_goal.pose.orientation.w = preLocalPoseTransform.getRotation().getW();
				mtx.unlock();
			}
			std::vector<marker_pose>().swap(temp_marker_pose);
		}
		else{
			geometry_msgs::Twist velocity;
			velocity.angular.z = wz_;

			pub_vel_.publish(velocity);
			mtx.unlock();
		}
	}

	bool calculate_global_pose::Loop(calcalate_global_marker::GetMarkerGlobalPose::Request &req,
			  						 calcalate_global_marker::GetMarkerGlobalPose::Response &res){

		if(req.start){
			/*calculate_global_pose cgp(private_nh);
			tf_listener = new tf::TransformListener(private_nh);
			tf_broadcaster = new tf::TransformBroadcaster();
			*/

			ros::Rate r(30);
			ros::Time begin = ros::Time::now();
			while(ros::ok()){
				r.sleep();
				ros::spinOnce();
				
				ros::Time end = ros::Time::now();
				if((end-begin).toSec() > 15.f){
					res.state = 1;
					res.goal = geometry_msgs::PoseStamped();
					return false;
				}
				if(markerVisible){
					res.state = 0;
					res.goal = goal;
					res.pre_goal = pre_goal;
					return true;
				}
			}
		}
	}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "calculate_global_pose_node");
	ros::NodeHandle private_nh("~");
	calculate_global_pose cgp(private_nh);
	tf_listener = new tf::TransformListener(private_nh);
	tf_broadcaster = new tf::TransformBroadcaster();

	ros::ServiceServer cgp_srv = private_nh.advertiseService("/calculate_marker_global_pose",&calculate_global_pose::Loop,&cgp);
	ros::spin();
  	return 0;
}
