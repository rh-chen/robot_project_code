#include <nav_msgs/GetMap.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <sys/stat.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>

#include <stdio.h>
#include <math.h>

#include <hash_set>
#include <set>
#include <map>
#include <algorithm> 

#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "scale_map/ScaleMapData.h"
#include "scale_map/GetCoveragePath.h"

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "scale_map_client");

  ros::NodeHandle n;

	ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("/scale_static_map",1);
  ros::ServiceClient client = n.serviceClient<scale_map::ScaleMapData>("/sweeper/scale_map_srv");

  scale_map::ScaleMapData srv;

  srv.request.erosion_radius = 1;
  srv.request.robot_radius = 0.03;

  ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>("/static_map");

  nav_msgs::GetMap getMapSrv;

  ros::Duration(2).sleep();
  if (mapClient.call(getMapSrv)) {
    srv.request.map = getMapSrv.response.map;
    std::cout << "map frame_id: " << srv.request.map.header.frame_id
              << std::endl;
  } else {
    ROS_ERROR("Failed to call /static_map service.");
    return 1;
  }

	ros::Time begin1 = ros::Time::now();
	bool res_srv_scale_map = client.call(srv);
	ros::Time end1 = ros::Time::now();

	std::cout << "call srv_scale_map time cost:" << (end1-begin1).toSec() << std::endl;
	//call scale map service
  if (res_srv_scale_map) {

		ros::ServiceClient client_darp = n.serviceClient<scale_map::GetCoveragePath>("/sweeper/make_coverage_plan");

  	scale_map::GetCoveragePath srv_darp;

		srv_darp.request.erosion_radius = 0.01;
		srv_darp.request.robot_radius = 0.03;
		srv_darp.request.occupancy_threshold = 95;
		srv_darp.request.map = srv.response.map;

		geometry_msgs::PoseStamped start;
		start.header.frame_id = srv.response.map.header.frame_id;
		start.pose.position.x = 11.2;
		start.pose.position.y = 15.8;
		
		srv_darp.request.start = start;

		ros::Time begin = ros::Time::now();
		bool res_srv_darp = client_darp.call(srv_darp);
		ros::Time end = ros::Time::now();

		std::cout << "call srv_darp time cost:" << (end-begin).toSec() << std::endl;
		if(res_srv_darp){
			ROS_INFO("call darp service");

    	ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/cleanner_planner", 1);
			ros::Rate loop_rate(10);
			while(ros::ok()){
				int path_size = srv_darp.response.plan.poses.size();

      	visualization_msgs::MarkerArray markerArray;

      	geometry_msgs::PoseStamped startPose = srv_darp.response.plan.poses[0];

      	//marker start pose
      	geometry_msgs::Pose  plannerStartPose;
      	plannerStartPose.position.x = startPose.pose.position.x;
      	plannerStartPose.position.y = startPose.pose.position.y;
      	plannerStartPose.orientation.w = 1.0;

      	geometry_msgs::Vector3 plannerStartScale;
      	plannerStartScale.x = 0.2;
      	plannerStartScale.y = 0.2;
      	plannerStartScale.z = 0.2;

      	std_msgs::ColorRGBA plannerStartColor;
      	plannerStartColor.a = 1.0;
      	plannerStartColor.g = 1.0;

      	int32_t plannerStartId = 0;

      	visualization_msgs::Marker markerSphereStart = createMarker("PlannerStart",
																																			visualization_msgs::Marker::SPHERE,
																																			plannerStartPose,
																																			plannerStartScale,
																																			plannerStartColor,
																																			plannerStartId,
																																			srv_darp.request.map.header.frame_id);

      	markerArray.markers.push_back(markerSphereStart);

      	geometry_msgs::Point last_point;
				last_point.x = startPose.pose.position.x;
				last_point.y = startPose.pose.position.y;

      	for(int i = 1; i < path_size; ++i) {

        	geometry_msgs::PoseStamped pose = srv_darp.response.plan.poses[i];
        	//ROS_INFO_STREAM("poses:%s" << pose);

        	//marker planner pose
        	geometry_msgs::Pose  markerArrowPose;
        	markerArrowPose.position.x = last_point.x;
        	markerArrowPose.position.y = last_point.y;
        	markerArrowPose.position.z = 0;
        	markerArrowPose.orientation.w = 1.0;

        	geometry_msgs::Vector3 markerArrowScale;
        	markerArrowScale.x = 0.05;
        	markerArrowScale.y = 0.1;
        	markerArrowScale.z = 0.1;

        	std_msgs::ColorRGBA markerArrowColor;
        	markerArrowColor.a = 1.0;
        	markerArrowColor.r = 1.0;

        	int32_t markerArrowId = i;

        	visualization_msgs::Marker markerArrow = createMarker("markerArrow",
																																visualization_msgs::Marker::ARROW,
																																markerArrowPose,
																																markerArrowScale,
																																markerArrowColor,
																																markerArrowId,
																																srv_darp.request.map.header.frame_id);

        	//arrowHead, arrowEnd
        	geometry_msgs::Point p;
        	p.x = pose.pose.position.x;
        	p.y = pose.pose.position.y;
        	p.z = 0;

        	geometry_msgs::Point arrowHeadPoint;
        	arrowHeadPoint.x = p.x - last_point.x;
        	arrowHeadPoint.y = p.y - last_point.y;
        	arrowHeadPoint.z = 0;


        	geometry_msgs::Point arrowEndPoint;
        	arrowEndPoint.x = 0;
        	arrowEndPoint.y = 0;
        	arrowEndPoint.z = 0;

        	markerArrow.points.push_back(arrowEndPoint);
        	markerArrow.points.push_back(arrowHeadPoint);

        	markerArray.markers.push_back(markerArrow);

        	last_point = p;

      	}

      	marker_pub.publish(markerArray);

      	ros::spinOnce();
      	loop_rate.sleep();
			}
		}
  } else {
    ROS_ERROR("Failed to call service /sweeper/scale_map_srv");
  }

  return 0;
}
