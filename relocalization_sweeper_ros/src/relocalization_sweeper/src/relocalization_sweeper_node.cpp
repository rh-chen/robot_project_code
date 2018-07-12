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
#include "relocalization_sweeper/GetRobotPose.h"


namespace relocalization_robot{
using namespace __gnu_cxx;
using namespace std;
using namespace cv;

class RelocalizationRobot{

};

bool GetRobotCurrentPose(
                         relocalization_sweeper::GetRobotPose::Request &req,
                         relocalization_sweeper::GetRobotPose::Response &res){
    ROS_INFO("start relocalization algorithm...");
    return true;
}

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "relocalization_robot_srv");

  ros::NodeHandle private_nh("~");

  //  advertise a service for getting a coverage plan
  ros::ServiceServer make_coverage_plan_srv = private_nh.advertiseService(
      "/sweeper/relocalization_robot_srv",
      relocalization_robot::GetRobotCurrentPose);

  ROS_INFO("Get Robot Current Pose Service Active...");

  ros::spin();

  return (0);
}


