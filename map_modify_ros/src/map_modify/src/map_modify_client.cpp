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
#include "map_modify/ModifyMap.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "map_modify_client");

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<map_modify::ModifyMap>("/sweeper/map_modify_srv");

  map_modify::ModifyMap srv;
  srv.request.threshold = 95;

  ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>("/static_map");
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_modify", 1);
  ros::Publisher map_contour_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_contour", 1);

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
	bool res_map_modify = client.call(srv);
	ros::Time end1 = ros::Time::now();

	std::cout << "call map_modify_srv time cost:" << (end1-begin1).toSec() << std::endl;
	//call scale map service
  if (res_map_modify) {
    ROS_INFO("Call service /sweeper/map_modify_srv success...");
    
    ros::Rate loop_rate(10);
    while(ros::ok()){
        map_pub.publish(srv.response.map);
        map_contour_pub.publish(srv.response.map_contour);
        ros::spinOnce();

        loop_rate.sleep();
    }

  } else {
    ROS_ERROR("Failed to call service /sweeper/scale_map_srv");
  }

  return 0;
}
