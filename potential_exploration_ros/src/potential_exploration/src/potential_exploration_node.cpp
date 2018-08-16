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
#include "potential_exploration/GetNextFrontier.h"

class potential_map_test{
public:
	ros::NodeHandle n;
	ros::Subscriber sub;

	potential_map_test(ros::NodeHandle n_):n(n_){
		sub = n.subscribe("/clicked_point",1000,&potential_map_test::clickCallBack,this);
	}

	visualization_msgs::Marker createMarker(const std::string markerName,
									uint32_t type, 
									geometry_msgs::Pose pose, 
									geometry_msgs::Vector3 scale, 
									std_msgs::ColorRGBA color,
									int32_t id, 
									std::string frame_id = std::string("map"))
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


	void clickCallBack(const geometry_msgs::PointStamped::ConstPtr &msg)
	{
		ros::ServiceClient potential_exploration_client = \
                       n.serviceClient<potential_exploration::GetNextFrontier>("/sweeper/potential_exploration");

		potential_exploration::GetNextFrontier get_next_frontier_srv;

        ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>("/static_map");
        nav_msgs::GetMap getMapSrv;

        ros::Duration(2).sleep();
    
        if(mapClient.call(getMapSrv)){
	        get_next_frontier_srv.request.map = getMapSrv.response.map;
            std::cout << "map frame_id: " << get_next_frontier_srv.request.map.header.frame_id << std::endl;
        }
        else{
            ROS_INFO("Failed to get map...");
        }
		//get_next_frontier_srv.request.map = nav_msgs::OccupancyGrid();
		get_next_frontier_srv.request.start.position.x = msg->point.x;
        get_next_frontier_srv.request.start.position.y = msg->point.y;
        get_next_frontier_srv.request.start.position.z = 0.0;
        get_next_frontier_srv.request.start.orientation.x = -0.707;
        get_next_frontier_srv.request.start.orientation.y = 0;
        get_next_frontier_srv.request.start.orientation.z = 0;
        get_next_frontier_srv.request.start.orientation.w = 0.707;
        get_next_frontier_srv.request.n_frontier = 5;

	    if(potential_exploration_client.call(get_next_frontier_srv)){
		    ROS_INFO("get next frontier success...");
		    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/next_frontiers", 1);
		    ros::Rate loop_rate(10);
		    while(ros::ok()){
			    visualization_msgs::MarkerArray markerArray;
			    for(int i = 0; i < get_next_frontier_srv.response.goal.poses.size(); ++i) {
					geometry_msgs::Pose pose = get_next_frontier_srv.response.goal.poses[i];

					geometry_msgs::Pose  markerPose;
					markerPose.position.x = pose.position.x;
					markerPose.position.y = pose.position.y;

					markerPose.position.z = 0;
					markerPose.orientation.w = 1.0;

					geometry_msgs::Vector3 markerScale;
					markerScale.x = 0.05;
					markerScale.y = 0.1;
					markerScale.z = 0.1;

					std_msgs::ColorRGBA markerColor;
					markerColor.a = 1.0;
					markerColor.r = 1.0;

					int32_t markerId = i;

					visualization_msgs::Marker marker = createMarker("marker",
													visualization_msgs::Marker::SPHERE,
													markerPose,
													markerScale,
													markerColor,
													markerId,
													get_next_frontier_srv.request.map.header.frame_id);

				    markerArray.markers.push_back(marker);
				}
					marker_pub.publish(markerArray);

					ros::spinOnce();
					loop_rate.sleep();
		    }
        }
	    else
	        ROS_INFO("get next frontier fail...");
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "potential_exploration_client");

		ros::NodeHandle n;
	
		potential_map_test pmt(n);
		ros::spin();

		return 0;

#if 0
    ros::NodeHandle n;
    ros::ServiceClient potential_exploration_client = \
                       n.serviceClient<potential_exploration::GetNextFrontier>("/sweeper/potential_exploration");

	potential_exploration::GetNextFrontier get_next_frontier_srv;

    ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>("/static_map");
    nav_msgs::GetMap getMapSrv;

    ros::Duration(2).sleep();
    
    if(mapClient.call(getMapSrv)){
	    get_next_frontier_srv.request.map = getMapSrv.response.map;
        std::cout << "map frame_id: " << get_next_frontier_srv.request.map.header.frame_id << std::endl;
    }
    else{
        ROS_INFO("Failed to get map...");
    }
	//get_next_frontier_srv.request.map = nav_msgs::OccupancyGrid();
	get_next_frontier_srv.request.start.position.x = 0.02;
    get_next_frontier_srv.request.start.position.y = 0.2;
    get_next_frontier_srv.request.start.position.z = 0.0;
    get_next_frontier_srv.request.start.orientation.x = -0.707;
    get_next_frontier_srv.request.start.orientation.y = 0;
    get_next_frontier_srv.request.start.orientation.z = 0;
    get_next_frontier_srv.request.start.orientation.w = 0.707;
    get_next_frontier_srv.request.n_frontier = 1;
	if(potential_exploration_client.call(get_next_frontier_srv))
		ROS_INFO("get next frontier success...");
	else
		ROS_INFO("get next frontier fail...");

	return 0;
#endif
}
