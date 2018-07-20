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
#include <cv_bridge/cv_bridge.h>

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>

#include <stdio.h>
#include <math.h>
#include <algorithm> 
#include <tf/tf.h>
#include <string>
#include "relocalization_sweeper/GetRobotPose.h"

using namespace cv;
using namespace std;

std::string data_dir;
int main(int argc,char **argv){
    
    ros::init(argc, argv, "relocalization_robot_client");

    ros::NodeHandle n,pn("~");
    pn.param<std::string>("data_dir",data_dir,"../rgb/");
    std::string image_dir = data_dir + "7_rgb.png";
    cv::Mat test = imread(image_dir,0);

    if(test.empty()){
        ROS_INFO("Load Image Error...");
        return 1;
    }

    ros::ServiceClient client = n.serviceClient<relocalization_sweeper::GetRobotPose>("/sweeper/relocalization_robot_srv");
    relocalization_sweeper::GetRobotPose srv;
   
    ros::Time time=ros::Time::now();  
    cv_bridge::CvImage cvi;
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "mono8";
    cvi.image = test;
     
    sensor_msgs::Image im;
    cvi.toImageMsg(im);

    srv.request.img_data = im;
    
    if(client.call(srv)){
        ROS_INFO("Relocalization Robot Success...");
    }

    return 0; 
}
