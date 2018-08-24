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
#include "line_detection_and_rotation/MapRotate.h"


int main(int argc, char* argv[])
{
		ros::init(argc, argv, "map_rotate_client");

		ros::NodeHandle n;

		ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("/map_rotate_map",1);
		ros::ServiceClient map_rotate_client = n.serviceClient<line_detection_and_rotation::MapRotate>("/sweeper/map_rotate_srv");

		line_detection_and_rotation::MapRotate map_rotate_srv;

		ros::ServiceClient mapClient = n.serviceClient<nav_msgs::GetMap>("/static_map");
		nav_msgs::GetMap getMapSrv;

		ros::Duration(2).sleep();
		if (mapClient.call(getMapSrv)) {
		  map_rotate_srv.request.map = getMapSrv.response.map;
		  std::cout << "map frame_id: " << map_rotate_srv.request.map.header.frame_id
		            << std::endl;
		} else {
		  ROS_ERROR("Failed to call /static_map service.");
		  return 1;
		}

		bool res = map_rotate_client.call(map_rotate_srv);

		if(res){
				ros::Rate loop_rate(10);
				while(ros::ok()){
						pub_map.publish(map_rotate_srv.response.map);			
						
						ros::spinOnce();
						loop_rate.sleep();
				}		
		}
    return 0;
}
