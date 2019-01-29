#include <ros/ros.h>
#include <ros/topic.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
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
#include <cmath>

#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <calculate_global_pose/GetRobotGlobalPose.h>

#define N_MP 30


using namespace tf;
using namespace std;

tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;

typedef struct mp{
	double x;
	double y;
	double z;
	double rol;
	double pit;
	double yaw;
	tf::Transform t;

	mp(double x_,double y_,double z_,double rol_,double pit_,double yaw_,tf::Transform t_)
	{x = x_;y = y_;z = z_;t = t_;rol = rol_;pit = pit_;yaw = yaw_;
		ROS_INFO_STREAM("x:" << x);
		ROS_INFO_STREAM("y:" << y);
		ROS_INFO_STREAM("z:" << z);
		ROS_INFO_STREAM("rol:" << rol);
		ROS_INFO_STREAM("pit:" << pit);
		ROS_INFO_STREAM("yaw:" << yaw);
	}

}marker_pose;

typedef struct range_delta{
    int index;
    float delta_x;
	float delta_y;
	float delta_z;
	float delta_rol;
	float delta_pit;
	float delta_yaw;
	float delta;

    range_delta(int index_,float delta_x_,float delta_y_,float delta_z_,float delta_rol_,float delta_pit_,float delta_yaw_){
        index = index_;
        delta_x = std::exp(delta_x_);
		delta_y = std::exp(delta_y_);
		delta_z = std::exp(delta_z_);
		delta_rol = std::exp(delta_rol_);
		delta_pit = std::exp(delta_pit_);
		delta_yaw = std::exp(delta_yaw_);
		ROS_INFO_STREAM("delta_x:" << delta_x_);
		ROS_INFO_STREAM("delta_y:" << delta_y_);
		ROS_INFO_STREAM("delta_z:" << delta_z_);
		ROS_INFO_STREAM("delta_rol:" << delta_rol_);
		ROS_INFO_STREAM("delta_pit:" << delta_pit_);
		ROS_INFO_STREAM("delta_yaw:" << delta_yaw_);
		delta = delta_x*delta_y*delta_z*delta_rol;
    }
}RangeDelta;

bool range_cmp(const RangeDelta& a,const RangeDelta& b){
    return a.delta < b.delta;
}

double wz_ = 0.2;
std::string world_frame = "base_link";
std::string markerarray_msg_frame_id = "aruco_left_camera_link";


/*visualization_msgs::Marker createMarker(const std::string markerName,
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
*/
class Calculate_Global_Pose{
	public:
	ros::NodeHandle nh;
	ros::Subscriber sub_robot_global_pose_;
	ros::Subscriber sub_marker_msg_;
	//ros::Publisher pub_marker_global_pose_;
	ros::Publisher pub_vel_;
	//ros::Publisher marker_pub;
	bool markerVisible;
	bool enoughMarker;
	//visualization_msgs::Marker markerSphere;

	//geometry_msgs::PoseStamped goal;
	geometry_msgs::PoseStamped pre_goal;

    double marker_pose_x;
    double marker_pose_y;
    double marker_pose_z;
    double marker_pose_q_x;
    double marker_pose_q_y;
    double marker_pose_q_z;
    double marker_pose_q_w;

	//std::vector<marker_pose> temp_marker_pose;
	std::vector<marker_pose> pre_temp_marker_pose;
    boost::mutex mtx;
	bool start_move;

	Calculate_Global_Pose(ros::NodeHandle n):nh(n)
	{
		markerVisible = false;
		start_move = false;
		enoughMarker = false;

		sub_marker_msg_ = nh.subscribe("/aruco_single/aruco_marker", 30,&Calculate_Global_Pose::getMarkerMsgCallback,this);
		//pub_marker_global_pose_ = nh.advertise<geometry_msgs::Pose>("/marker_global_pose",30);
		pub_vel_ = nh.advertise<geometry_msgs::Twist>("/mynt_cmd_vel_mux/input/keyop",30);
		//marker_pub = n.advertise<visualization_msgs::Marker>("/forward_marker_pose", 30);
	}

	void getMarkerMsgCallback(const aruco_msgs::MarkerArray::ConstPtr &msg);
	bool selectMarkerPose(std::vector<marker_pose>& src_,\
			      std::vector<marker_pose>& dst_,\
			      double delta_threshold_,\
			      int n_d_,\
			      double delta_angle_threshold_,\
			      int n_angle_d_);
	bool Loop(calculate_global_pose::GetRobotGlobalPose::Request &req,\
			  calculate_global_pose::GetRobotGlobalPose::Response &res);
};

	bool Calculate_Global_Pose::selectMarkerPose(std::vector<marker_pose>& src_,\
						     std::vector<marker_pose>& dst_,\
						     double delta_threshold_,\
						     int n_d_,\
						     double delta_angle_threshold_,\
						     int n_angle_d_){
		std::vector<marker_pose> pos_marker_pose;
		std::vector<marker_pose> neg_marker_pose;
		int pos_marker_pose_count = 0;
		int neg_marker_pose_count = 0;
		
		for(int i = 0;i < src_.size();i++){
			if(src_[i].rol > 0){
				pos_marker_pose.push_back(src_[i]);
				pos_marker_pose_count++;
			}
			else{
				neg_marker_pose.push_back(src_[i]);
				neg_marker_pose_count++;
			}
		}

		ROS_INFO_STREAM("pos_marker_pose_count:" << pos_marker_pose_count);
		ROS_INFO_STREAM("neg_marker_pose_count:" << neg_marker_pose_count);
		
		double ave_marker_pose_x = 0.f;
		double ave_marker_pose_y = 0.f;
		double ave_marker_pose_z = 0.f;
		double ave_marker_pose_rol = 0.f;
		double ave_marker_pose_pit = 0.f;
		double ave_marker_pose_yaw = 0.f;
	
		if(pos_marker_pose_count > neg_marker_pose_count){
			for(int i = 0;i < pos_marker_pose_count;i++){
				ave_marker_pose_x += pos_marker_pose[i].x;
				ave_marker_pose_y += pos_marker_pose[i].y;
				ave_marker_pose_z += pos_marker_pose[i].z;
				ave_marker_pose_rol += pos_marker_pose[i].rol;
				ave_marker_pose_pit += pos_marker_pose[i].pit;
				ave_marker_pose_yaw += pos_marker_pose[i].yaw;
			}
		
			if(pos_marker_pose_count > 0){
				ave_marker_pose_x /= pos_marker_pose_count;
				ave_marker_pose_y /= pos_marker_pose_count;
				ave_marker_pose_z /= pos_marker_pose_count;
				ave_marker_pose_rol /= pos_marker_pose_count;
				ave_marker_pose_pit /= pos_marker_pose_count;
				ave_marker_pose_yaw /= pos_marker_pose_count;
			}
			else
				return false;

			std::vector<RangeDelta> delta_vec;
			for(int i = 0;i < pos_marker_pose_count;i++){
				delta_vec.push_back(
								RangeDelta(i,\
										   std::fabs(pos_marker_pose[i].x-ave_marker_pose_x),\
										   std::fabs(pos_marker_pose[i].y-ave_marker_pose_y),\
										   std::fabs(pos_marker_pose[i].z-ave_marker_pose_z),\
										   std::fabs(pos_marker_pose[i].rol-ave_marker_pose_rol),\
										   std::fabs(pos_marker_pose[i].pit-ave_marker_pose_pit),\
										   std::fabs(pos_marker_pose[i].yaw-ave_marker_pose_yaw))
							   );
			}

			std::sort(delta_vec.begin(),delta_vec.end(),range_cmp);
		
			double delta_threshold = delta_threshold_;
			double delta_angle_threshold = delta_angle_threshold_;
			int n_d = n_d_;
			int n_angle_d = n_angle_d_;
			double localThreshold = std::pow(std::exp(delta_threshold),n_d)*std::pow(std::exp(delta_angle_threshold),n_angle_d);
			for(int i = 0;i < pos_marker_pose_count;i++){
				
				ROS_INFO_STREAM("pos_delta_vec:" << delta_vec[i].delta);
				ROS_INFO_STREAM("delta_threshold:" << localThreshold);

				if(delta_vec[i].delta < localThreshold)
					dst_.push_back(pos_marker_pose[delta_vec[i].index]);
			}
			
			if(dst_.size() <= 0)
				return false;
			else	
				return true;
		}
		else{
			for(int i = 0;i < neg_marker_pose_count;i++){
				ave_marker_pose_x += neg_marker_pose[i].x;
				ave_marker_pose_y += neg_marker_pose[i].y;
				ave_marker_pose_z += neg_marker_pose[i].z;

				ave_marker_pose_rol += neg_marker_pose[i].rol;
				ave_marker_pose_pit += neg_marker_pose[i].pit;
				ave_marker_pose_yaw += neg_marker_pose[i].yaw;
			}
		
			if(neg_marker_pose_count > 0){
				ave_marker_pose_x /= neg_marker_pose_count;
				ave_marker_pose_y /= neg_marker_pose_count;
				ave_marker_pose_z /= neg_marker_pose_count;

				ave_marker_pose_rol /= neg_marker_pose_count;
				ave_marker_pose_pit /= neg_marker_pose_count;
				ave_marker_pose_yaw /= neg_marker_pose_count;
			}
			else
				return false;
			
			std::vector<RangeDelta> delta_vec;
			for(int i = 0;i < neg_marker_pose_count;i++){
				delta_vec.push_back(
								RangeDelta(i,\
										   std::fabs(neg_marker_pose[i].x-ave_marker_pose_x),\
										   std::fabs(neg_marker_pose[i].y-ave_marker_pose_y),\
										   std::fabs(neg_marker_pose[i].z-ave_marker_pose_z),\
										   std::fabs(neg_marker_pose[i].rol-ave_marker_pose_rol),\
										   std::fabs(neg_marker_pose[i].pit-ave_marker_pose_pit),\
										   std::fabs(neg_marker_pose[i].yaw-ave_marker_pose_yaw))
							   );
			}

			std::sort(delta_vec.begin(),delta_vec.end(),range_cmp);
		
			double delta_threshold = delta_threshold_;
			double delta_angle_threshold = delta_angle_threshold_;
			int n_d = n_d_;
			int n_angle_d = n_angle_d_;
			double localThreshold = std::pow(std::exp(delta_threshold),n_d)*std::pow(std::exp(delta_angle_threshold),n_angle_d);

			for(int i = 0;i < neg_marker_pose_count;i++){
				ROS_INFO_STREAM("neg_delta_vec:" << delta_vec[i].delta);
				ROS_INFO_STREAM("delta_threshold:" << localThreshold);
				
				if(delta_vec[i].delta < localThreshold)
					dst_.push_back(neg_marker_pose[delta_vec[i].index]);
			}
		
			if(dst_.size() <= 0)
				return false;
			else	
				return true;
		}
	}
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

	void Calculate_Global_Pose::getMarkerMsgCallback(const aruco_msgs::MarkerArray::ConstPtr &msg)
	{
		if(!start_move)
			return;

        mtx.lock();
		//static int id  = 0;
		std::string marker_frame;
		
		/*if(markerVisible){
			ROS_INFO_STREAM("Marker Visible...");
			//marker_pub.publish(markerSphere);
			mtx.unlock();
			return;
		}*/

		if((msg->header.frame_id == markerarray_msg_frame_id) && (msg->markers.size() > 0)){
			aruco_msgs::Marker marker = msg->markers[0];
			markerVisible = true;

			marker_frame = marker.header.frame_id;
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

			/*tf::StampedTransform localPoseTransform;;
    		try{
					tf_listener->waitForTransform(world_frame,marker_frame+"_forward", ros::Time(0), ros::Duration(1.0));
					tf_listener->lookupTransform(world_frame,marker_frame+"_forward",ros::Time(0), localPoseTransform);	
   			}
    		catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				pub_vel_.publish(geometry_msgs::Twist());
				ros::shutdown();
    		}*/

			//tf::Transform CToM = localPoseTransform.inverse();
			
			/*double marker_x_ = localPoseTransform.getOrigin().x();
			double marker_y_ = localPoseTransform.getOrigin().y();
			double marker_z_ = localPoseTransform.getOrigin().z();
            */

			double pre_marker_x_ = preLocalPoseTransform.getOrigin().x();
			double pre_marker_y_ = preLocalPoseTransform.getOrigin().y();
			double pre_marker_z_ = preLocalPoseTransform.getOrigin().z();

			//double roll_,pitch_,yaw_;
			//localPoseTransform.getBasis().getRPY(roll_,pitch_,yaw_);

			double pre_roll_,pre_pitch_,pre_yaw_;
			preLocalPoseTransform.getBasis().getRPY(pre_roll_,pre_pitch_,pre_yaw_);
			//ROS_INFO_STREAM("be_marker_x_:" << marker.pose.pose.position.x);
			//ROS_INFO_STREAM("be_marker_y_:" << marker.pose.pose.position.y);
			//ROS_INFO_STREAM("be_marker_z_:" << marker.pose.pose.position.z);
			

			if(pre_temp_marker_pose.size() < N_MP){
				//temp_marker_pose.push_back(marker_pose(marker_x_,\
													   marker_y_,\
													   marker_z_,\
													   roll_,\
													   pitch_,\
													   yaw_,\
													   localPoseTransform));
				pre_temp_marker_pose.push_back(marker_pose(pre_marker_x_,\
														   pre_marker_y_,\
														   pre_marker_z_,\
														   pre_roll_,\
														   pre_pitch_,\
														   pre_yaw_,\
														   preLocalPoseTransform));
				//ROS_INFO_STREAM("Store Marker Temp Pose:" << temp_marker_pose.size());
				ROS_INFO_STREAM("Store Marker Pre Temp Pose:" << pre_temp_marker_pose.size());
				mtx.unlock();
				return;
			}
			else{
				enoughMarker = true;
				//tf::Vector3 marker_position(0,0,0);
				//tf::Vector3 marker_rpy(0,0,0);

				tf::Vector3 pre_marker_position(0,0,0);
				tf::Vector3 pre_marker_rpy(0,0,0);
				
				//std::vector<marker_pose> sel_marker_pose;
				//bool sel_res = selectMarkerPose(temp_marker_pose,sel_marker_pose,0.05,3,0.1,1);
				std::vector<marker_pose> sel_pre_marker_pose;
				bool sel_pre_res = selectMarkerPose(pre_temp_marker_pose,sel_pre_marker_pose,0.05,3,0.1,1);
				//ROS_INFO_STREAM("sel_res:" << sel_res);	
				ROS_INFO_STREAM("sel_pre_res:" << sel_pre_res);
	
				if(!sel_pre_res)
					ROS_ERROR("Select Marker Pose Error...");
				
				//sel_marker_pose
				/*for(int i = 0;i < sel_marker_pose.size();i++){
					double marker_roll = 0;
					double marker_pitch = 0;
					double marker_yaw = 0;

					marker_position[0] += sel_marker_pose[i].t.getOrigin().x();
					marker_position[1] += sel_marker_pose[i].t.getOrigin().y();
					marker_position[2] += sel_marker_pose[i].t.getOrigin().z();

					sel_marker_pose[i].t.getBasis().getRPY(marker_roll,\
														   marker_pitch,\
														   marker_yaw);
				
					marker_rpy[0] += marker_roll;
					marker_rpy[1] += marker_pitch;
					marker_rpy[2] += marker_yaw;
				}*/
		
				//sel_pre_marker_pose
				for(int i = 0;i < sel_pre_marker_pose.size();i++){
					double pre_marker_roll = 0;
					double pre_marker_pitch = 0;
					double pre_marker_yaw = 0;
					
					pre_marker_position[0] += sel_pre_marker_pose[i].t.getOrigin().x();
					pre_marker_position[1] += sel_pre_marker_pose[i].t.getOrigin().y();
					pre_marker_position[2] += sel_pre_marker_pose[i].t.getOrigin().z();

					sel_pre_marker_pose[i].t.getBasis().getRPY(pre_marker_roll,\
															   pre_marker_pitch,\
															   pre_marker_yaw);

					pre_marker_rpy[0] += pre_marker_roll;
					pre_marker_rpy[1] += pre_marker_pitch;
					pre_marker_rpy[2] += pre_marker_yaw;
				}

				//int sel_marker_count = sel_marker_pose.size();
				int sel_pre_marker_count = sel_pre_marker_pose.size();
				//ROS_INFO_STREAM("sel_marker_count:" << sel_marker_count);
				ROS_INFO_STREAM("sel_pre_marker_count:" << sel_pre_marker_count);

				/*marker_position[0] /= sel_marker_count;
				marker_position[1] /= sel_marker_count;
				marker_position[2] /= sel_marker_count;
				ROS_INFO_STREAM("marker_position[0]:" << marker_position[0]);
				ROS_INFO_STREAM("marker_position[1]:" << marker_position[1]);
				ROS_INFO_STREAM("marker_position[2]:" << marker_position[2]);
                */

				pre_marker_position[0] /= sel_pre_marker_count;
				pre_marker_position[1] /= sel_pre_marker_count;
				pre_marker_position[2] /= sel_pre_marker_count;
				ROS_INFO_STREAM("pre_marker_position[0]:" << pre_marker_position[0]);
				ROS_INFO_STREAM("pre_marker_position[1]:" << pre_marker_position[1]);
				ROS_INFO_STREAM("pre_marker_position[2]:" << pre_marker_position[2]);

				/*marker_rpy[0] /= sel_marker_count;
				marker_rpy[1] /= sel_marker_count;
				marker_rpy[2] /= sel_marker_count;
				ROS_INFO_STREAM("marker_rpy[0]:" << marker_rpy[0]);
				ROS_INFO_STREAM("marker_rpy[1]:" << marker_rpy[1]);
				ROS_INFO_STREAM("marker_rpy[2]:" << marker_rpy[2]);
                */

				pre_marker_rpy[0] /= sel_pre_marker_count;
				pre_marker_rpy[1] /= sel_pre_marker_count;
				pre_marker_rpy[2] /= sel_pre_marker_count;
				ROS_INFO_STREAM("pre_marker_rpy[0]:" << pre_marker_rpy[0]);
				ROS_INFO_STREAM("pre_marker_rpy[1]:" << pre_marker_rpy[1]);
				ROS_INFO_STREAM("pre_marker_rpy[2]:" << pre_marker_rpy[2]);

				/*tf::Quaternion q;
				q.setRPY(marker_rpy[0],marker_rpy[1],marker_rpy[2]);
				tf::Transform t(q,marker_position);
                */

				tf::Quaternion pre_q;
				pre_q.setRPY(pre_marker_rpy[0],pre_marker_rpy[1],pre_marker_rpy[2]);
				tf::Transform baselinkTomarker(pre_q,pre_marker_position);


				tf::Quaternion mar_q(marker_pose_q_x,marker_pose_q_y,marker_pose_q_z,marker_pose_q_w);
                tf::Vector3 mar_position(marker_pose_x,marker_pose_y,marker_pose_z);
				tf::Transform mapTomarker(mar_q,mar_position);

				/*tf::Quaternion qq(marker.pose.pose.orientation.x,\
								  marker.pose.pose.orientation.y,\
								  marker.pose.pose.orientation.z,\
								  marker.pose.pose.orientation.w);
				tf::Vector3 markerOrigin (marker.pose.pose.position.x,marker.pose.pose.position.y,marker.pose.pose.position.z-1.0);	
				tf::Transform m(qq, markerOrigin);
				*/

				//tf::Transform p = t;
				tf::Transform pre_p = mapTomarker*baselinkTomarker.inverse();
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
				/*goal.header.stamp = marker.header.stamp;
				goal.header.frame_id = "map";
				goal.pose.position.x = p.getOrigin().x();
				goal.pose.position.y = p.getOrigin().y();
				goal.pose.position.z = p.getOrigin().z();
				goal.pose.orientation.x = p.getRotation().getX();
				goal.pose.orientation.y = p.getRotation().getY();
				goal.pose.orientation.z = p.getRotation().getZ();
				goal.pose.orientation.w = p.getRotation().getW();
                */

				pre_goal.header.stamp = marker.header.stamp;
				pre_goal.header.frame_id = "map";
				pre_goal.pose.position.x = pre_p.getOrigin().x();
				pre_goal.pose.position.y = pre_p.getOrigin().y();
				pre_goal.pose.position.z = pre_p.getOrigin().z();
				pre_goal.pose.orientation.x = pre_p.getRotation().getX();
				pre_goal.pose.orientation.y = pre_p.getRotation().getY();
				pre_goal.pose.orientation.z = pre_p.getRotation().getZ();
				pre_goal.pose.orientation.w = pre_p.getRotation().getW();

				//std::vector<marker_pose>().swap(sel_marker_pose);
				std::vector<marker_pose>().swap(sel_pre_marker_pose);
				mtx.unlock();
			}
			//std::vector<marker_pose>().swap(temp_marker_pose);
			std::vector<marker_pose>().swap(pre_temp_marker_pose);
		}
		else{
			ROS_INFO_STREAM("Not Recognize Marker...");
            if(markerVisible)
			    markerVisible = true;
            else
                markerVisible = false;

			mtx.unlock();
		}
	}

	bool Calculate_Global_Pose::Loop(calculate_global_pose::GetRobotGlobalPose::Request &req,
			  						 calculate_global_pose::GetRobotGlobalPose::Response &res){

		if(req.start){
			/*calculate_global_pose cgp(private_nh);
			tf_listener = new tf::TransformListener(private_nh);
			tf_broadcaster = new tf::TransformBroadcaster();
			*/
		    	
            marker_pose_x = req.marker_pose.pose.position.x;
            marker_pose_y = req.marker_pose.pose.position.y;
            marker_pose_z = req.marker_pose.pose.position.z;
            marker_pose_q_x = req.marker_pose.pose.orientation.x;
            marker_pose_q_y = req.marker_pose.pose.orientation.y;
            marker_pose_q_z = req.marker_pose.pose.orientation.z;
            marker_pose_q_w = req.marker_pose.pose.orientation.w;


			start_move = req.start;

			ros::Rate r(30);
			ros::Time begin = ros::Time::now();
			while(ros::ok()){
				r.sleep();
				ros::spinOnce();
				
				ros::Time end = ros::Time::now();
				if((end-begin).toSec() > 30.f){
					res.status = 1;
					res.text.data = "fail";
					res.robot_pose = geometry_msgs::PoseStamped();

					return false;
				}
				if(markerVisible){
					if(enoughMarker){
						res.status = 0;
						res.text.data = "success"; 
						res.robot_pose = pre_goal;

						markerVisible = false;
						start_move = false;
						enoughMarker = false;
						return true;
					}
					else
					    ROS_INFO_STREAM("Storing Marker Pose...");
				}
				else{
					geometry_msgs::Twist velocity;
					velocity.angular.z = wz_;

					pub_vel_.publish(velocity);
				}
			}
		}
	}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "calculate_robot_global_pose_node");
	ros::NodeHandle private_nh("~");
	Calculate_Global_Pose cgp(private_nh);
	tf_listener = new tf::TransformListener(private_nh);
	tf_broadcaster = new tf::TransformBroadcaster();

	ros::ServiceServer cgp_srv = private_nh.advertiseService("/calculate_robot_global_pose",&Calculate_Global_Pose::Loop,&cgp);
    ROS_INFO("server /calculate_robot_global_pose active...");
	ros::spin();
  	return 0;
}
