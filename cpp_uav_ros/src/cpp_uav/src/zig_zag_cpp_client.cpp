#include <cpp_uav.hpp>

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


    ros::Duration(2).sleep();
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
        
        int32_t markerArrowId = -1;
        visualization_msgs::MarkerArray markerArray;

    for(int j = 0;j < srv.response.subpolygons.size();j++){

        geometry_msgs::Point lastPoint;
        lastPoint.x = srv.response.subpolygons[j].points[0].x;
        lastPoint.y = srv.response.subpolygons[j].points[0].y;

		for(int i = 1; i < srv.response.subpolygons[j].points.size(); ++i) {
			geometry_msgs::Point pose;
            pose.x = srv.response.subpolygons[j].points[i].x;
            pose.y = srv.response.subpolygons[j].points[i].y;

			//ROS_INFO_STREAM("poses:%s" << pose);

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

            markerArrowId++;

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

            markerArrowId++;

			visualization_msgs::Marker markerArrow = createMarker("markerArrow",visualization_msgs::Marker::ARROW,markerArrowPose,markerArrowScale,markerArrowColor,markerArrowId, srv.request.map.header.frame_id); //make markerArrow in map plane

			geometry_msgs::Point p;
			p.x = srv.response.subpolygons[j].points[0].x;
			p.y = srv.response.subpolygons[j].points[0].y;
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
    }

	while(ros::ok()){
		marker_pub.publish(markerArray);

		ros::spinOnce();
		loop_rate.sleep();
	}
	}else{
		ROS_ERROR("Failed to call service /cpp_service");
	}
	
	return 0;
}
