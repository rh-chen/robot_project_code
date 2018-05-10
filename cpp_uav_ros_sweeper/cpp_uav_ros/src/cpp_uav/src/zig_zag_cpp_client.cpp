/**
 * @file torres_etal_2016.cpp
 * @brief Coverage path planner based on M. Torres et al, 2016
 * @author Takaki Ueno
 */

/*
 * Copyright (c) 2017 Takaki Ueno
 * Released under the MIT license
 */

// header
#include <torres_etal_2016.hpp>

// cpp standard libraries
#include <array>
#include <vector>

// roscpp
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/GetMap.h>

// geometry_msgs
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

// Service
#include "cpp_uav/Torres16.h"

visualization_msgs::Marker createMarker(const std::string markerName,uint32_t type, geometry_msgs::Pose pose, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color,  int32_t id, std::string frame_id = std::string("s_map"))
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
int main(int argc,char **argv){

	ros::init(argc,argv,"cpp_uav_client");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<cpp_uav::Torres16>("cpp_service");

	cpp_uav::Torres16 srv;
	srv.request.start.x = 1.45;
	srv.request.start.y = 2.5;

	srv.request.erosion_radius = 0.1;     //  unit: meter
  srv.request.robot_radius = 0.4;        //  unit: meter
  srv.request.occupancy_threshold = 95;  //  range:0~100

	srv.request.footprint_length.data = 0.4;
	srv.request.footprint_width.data = 0.4;
	srv.request.horizontal_overwrap.data = 0.1;
	srv.request.vertical_overwrap.data = 0.1;

	ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap getMapSrv;
  if (mapClient.call(getMapSrv)) {
    srv.request.map = getMapSrv.response.map;
    std::cout << "map frame_id: " << srv.request.map.header.frame_id
              << std::endl;
  } else {
    ROS_ERROR("Failed to call /static_map service.");
    return 1;
  }
	
	if(client.call(srv)){
		ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/cleanner_planner",1);
		ros::Rate loop_rate(10);

	while(ros::ok()){
		int path_size = srv.response.path.size();		
		visualization_msgs::MarkerArray markerArray;

		geometry_msgs::Point lastPose = srv.response.path[0];

		geometry_msgs::Pose  plannerStartPose;
		plannerStartPose.position.x = srv.request.start.x;
		plannerStartPose.position.y = srv.request.start.y;
		plannerStartPose.orientation.w = 1.0;

		geometry_msgs::Vector3 plannerStartScale;
		plannerStartScale.x = 0.2;
		plannerStartScale.y = 0.2;
		plannerStartScale.z = 0.2;

		std_msgs::ColorRGBA plannerStartColor;
		plannerStartColor.a = 1.0;
		plannerStartColor.g = 1.0;

		int32_t plannerStartId = 0;

		visualization_msgs::Marker markerSphereStart = createMarker("PlannerStart",visualization_msgs::Marker::SPHERE,plannerStartPose,plannerStartScale,plannerStartColor,plannerStartId, srv.request.map.header.frame_id);

		markerArray.markers.push_back(markerSphereStart);

		geometry_msgs::Point lastPoint;
		lastPoint.x = lastPose.x;
		lastPoint.y = lastPose.y;
		lastPoint.z = 0;

		for(int i = 1; i < path_size; ++i) {
			geometry_msgs::Point pose = srv.response.path[i];
			ROS_INFO_STREAM("poses:%s" << pose);

			geometry_msgs::Pose  markerArrowPose;
			markerArrowPose.position.x = lastPoint.x;
			markerArrowPose.position.y = lastPoint.y;
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

			visualization_msgs::Marker markerArrow = createMarker("markerArrow",visualization_msgs::Marker::ARROW,markerArrowPose,markerArrowScale,markerArrowColor,markerArrowId, srv.request.map.header.frame_id); //make markerArrow in map plane

			geometry_msgs::Point p;
			p.x = pose.x;
			p.y = pose.y;
			p.z = 0;

			geometry_msgs::Point arrowHeadPoint;
			arrowHeadPoint.x = p.x - lastPoint.x;
			arrowHeadPoint.y = p.y - lastPoint.y;
			arrowHeadPoint.z = 0;


			geometry_msgs::Point arrowEndPoint;
			arrowEndPoint.x = 0;
			arrowEndPoint.y = 0;
			arrowEndPoint.z = 0;

			markerArrow.points.push_back(arrowEndPoint);
			markerArrow.points.push_back(arrowHeadPoint);

			markerArray.markers.push_back(markerArrow);

			lastPoint = p;
		}

		geometry_msgs::Pose  plannerGoalPose;
		plannerGoalPose.position.x = srv.request.start.x;
		plannerGoalPose.position.y = srv.request.start.y;
		plannerGoalPose.orientation.w = 1.0;

		geometry_msgs::Vector3 plannerGoalScale;
		plannerGoalScale.x = 0.2;
		plannerGoalScale.y = 0.2;
		plannerGoalScale.z = 0.2;

		std_msgs::ColorRGBA plannerGoalColor;
		plannerGoalColor.a = 1.0;
		plannerGoalColor.b = 1.0;

		int32_t plannerGoalId = path_size + 1;

		visualization_msgs::Marker markerSphereGoal = createMarker("PlannerGoal",visualization_msgs::Marker::SPHERE,plannerGoalPose,plannerGoalScale,plannerGoalColor,plannerGoalId, srv.request.map.header.frame_id);

		markerArray.markers.push_back(markerSphereGoal);

		marker_pub.publish(markerArray);

		ros::spinOnce();
		loop_rate.sleep();
	}
	}else{
		ROS_ERROR("Failed to call service /cpp_service");
	}
	
	return 0;
}
