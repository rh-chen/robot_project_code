#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "DBoW3/DBoW3.h"
#include <opencv2/features2d/features2d.hpp>
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
#include <cv_bridge/cv_bridge.h>

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>
#include <stdio.h>
#include <math.h>
#include <algorithm> 
#include <tf/tf.h>
#include <string>
#include <vector>
#include "relocalization_sweeper/GetRobotPose.h"

std::string voc_dir;
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
    ROS_INFO("Load vocabulary...");
    
    ROS_INFO("Vocabulary Direction:%s",voc_dir.c_str());
    DBoW3::Vocabulary voc(voc_dir+"voc.yml.gz");
    //std::cout << __FILE__ << __LINE__ << std::endl;

    if(voc.empty()){
        ROS_ERROR("Load vocabulary fail...");
        return false;
    }

    //std::cout << __FILE__ << __LINE__ << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr =  cv_bridge::toCvCopy(req.img_data, sensor_msgs::image_encodings::MONO8);

    //std::cout << __FILE__ << __LINE__ << std::endl;
    cv::Mat source = cv_ptr->image;
    ROS_INFO("Image Height:%d,Image Width:%d",source.rows,source.cols);

    //std::cout << __FILE__ << __LINE__ << std::endl;
    return true;
}

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "relocalization_robot_service");

  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("voc_dir",voc_dir,"../voc/");

  //advertise a service for getting a coverage plan
  ros::ServiceServer make_coverage_plan_srv = private_nh.advertiseService(
      "/sweeper/relocalization_robot_srv",
      relocalization_robot::GetRobotCurrentPose);

  ROS_INFO("Get Robot Current Pose Service Active...");

  ros::spin();

  return (0);
}


