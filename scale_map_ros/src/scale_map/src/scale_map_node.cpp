#include <deque>
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

namespace scale_map{

using namespace __gnu_cxx; 
using namespace std;
using namespace cv;

bool ScaleMapService(
    scale_map::ScaleMapData::Request &req,     
    scale_map::ScaleMapData::Response &resp) { 
  if (req.erosion_radius < 0) {
    ROS_ERROR("erosion_radius < 0");
    return false;
  }
  if (req.robot_radius < 0) {
    ROS_ERROR("robot_radius < 0");
    return false;
  }
	
  cv::Mat map(req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());
	
  ROS_INFO("start to scale map.");
  return true;
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scale_map_node");

  ros::NodeHandle private_nh("~");

  ros::ServiceServer scale_map_srv = private_nh.advertiseService(
      "/sweeper/scale_map_srv",
      scale_map::ScaleMapService);

  ROS_INFO("scale map service active.");

  ros::spin();

  return (0);
}
