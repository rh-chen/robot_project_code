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
#include "scale_map/ScaleMapData.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "scale_map_client");

  ros::NodeHandle n;
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

	//call scale map service
  if (client.call(srv)) {
		ROS_INFO("call darp service");
  } else {
    ROS_ERROR("Failed to call service /sweeper/scale_map_srv");
  }

  return 0;
}
