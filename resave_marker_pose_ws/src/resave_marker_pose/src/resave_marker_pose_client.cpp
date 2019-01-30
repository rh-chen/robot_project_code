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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <resave_marker_pose/ResaveMarkerPose.h>
#include <actionlib/client/simple_action_client.h> 
#include <actionlib/client/terminal_state.h> 

using namespace tf;
using namespace std;


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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "resave_marker_pose_client");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/rmp/robot_pose", 30);
	ros::ServiceClient rmp_client = n.serviceClient<resave_marker_pose::ResaveMarkerPose>("/resave_marker_pose");

	ros::Rate loop_rate(20);
//while(ros::ok())
{
	resave_marker_pose::ResaveMarkerPose rmp_srv;

	rmp_srv.request.start = true;
	ros::Duration(0.5).sleep();	

	bool res_rmp_srv = rmp_client.call(rmp_srv);

	if(res_rmp_srv){
		ROS_INFO_STREAM("Resave marker pose success...");


		geometry_msgs::Pose  pre_pose;
		pre_pose.position.x = rmp_srv.response.marker_pose.pose.position.x;
		pre_pose.position.y = rmp_srv.response.marker_pose.pose.position.y;
		pre_pose.position.z = rmp_srv.response.marker_pose.pose.position.z;
		ROS_INFO_STREAM("pre_pose.position.x:" << pre_pose.position.x);
		ROS_INFO_STREAM("pre_pose.position.y:" << pre_pose.position.y);
		ROS_INFO_STREAM("pre_pose.position.z:" << pre_pose.position.z);
		pre_pose.orientation.w = 1.0;

        geometry_msgs::Vector3 scale;
        scale.x = 0.1;
        scale.y = 0.1;
        scale.z = 0.1;

        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.r = 1.0;

		visualization_msgs::Marker markerSphere = createMarker("markerSphere",
											visualization_msgs::Marker::SPHERE,
											pre_pose,
											scale,
											color,
											0,
											rmp_srv.response.marker_pose.header.frame_id);
		//ros::Rate loop_rate(20);
		/*while(ros::ok()){
			marker_pub.publish(markerSphere);
			ros::spinOnce();
			loop_rate.sleep();
		}*/

		ROS_INFO_STREAM("Call resave_marker_pose success...");
	}
	else{
		ROS_INFO_STREAM("Call resave_marker_pose fail...");
	}
}
  	return 0;
}
