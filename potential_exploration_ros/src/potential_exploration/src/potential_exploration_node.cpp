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

int main(int argc, char **argv) {
    ros::init(argc, argv, "potential_exploration_client");

    ros::NodeHandle n;
    ros::ServiceClient potential_exploration_client = \
                       n.serviceClient<potential_exploration::GetNextFrontier>("/sweeper/potential_exploration");

	potential_exploration::GetNextFrontier get_next_frontier_srv;

    ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>("/static_map");
    nav_msgs::GetMap getMapSrv;
    if(mapClient.call(getMapSrv)){
	    get_next_frontier_srv.request.map = getMapSrv.response.map;
        std::cout << "map frame_id: " << get_next_frontier_srv.request.map.header.frame_id << std::endl;
    }
    else{
        ROS_INFO("Failed to get map...");
    }
	get_next_frontier_srv.request.map = nav_msgs::OccupancyGrid();
	get_next_frontier_srv.request.start.position.x = 0.1;
    get_next_frontier_srv.request.start.position.y = 0.1;
    get_next_frontier_srv.request.start.position.z = 0.0;

	if(potential_exploration_client.call(get_next_frontier_srv))
		ROS_INFO("get next frontier success...");
	else
		ROS_INFO("get next frontier fail...");

	return 0;
}
