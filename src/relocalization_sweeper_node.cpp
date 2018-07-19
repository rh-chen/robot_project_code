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
std::string dataset_dir;

namespace relocalization_robot{
using namespace __gnu_cxx;
using namespace std;
using namespace cv;

bool GetRobotCurrentPose(
                         relocalization_sweeper::GetRobotPose::Request &req,
                         relocalization_sweeper::GetRobotPose::Response &res){
    ROS_INFO("start relocalization algorithm...");
    ROS_INFO("Load vocabulary...");
    
    ROS_INFO("Vocabulary Direction:%s",voc_dir.c_str());
    ROS_INFO("Data Direction:%s",dataset_dir.c_str());

    DBoW3::Vocabulary voc(voc_dir+"voc.yml.gz");
    //std::cout << __FILE__ << __LINE__ << std::endl;

    if(voc.empty()){
        ROS_ERROR("Load vocabulary fail...");
        return false;
    }
    
    std::cout << "Voc Info:" << voc << std::endl;
    //std::cout << __FILE__ << __LINE__ << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr =  cv_bridge::toCvCopy(req.img_data, sensor_msgs::image_encodings::MONO8);

    //std::cout << __FILE__ << __LINE__ << std::endl;
    cv::Mat source = cv_ptr->image;
    cv::Mat source_descriptor;
    vector<KeyPoint> source_keypoints; 

    ROS_INFO("Image Height:%d,Image Width:%d",source.rows,source.cols);
    
    ifstream fin ( dataset_dir+"/data.txt" );
    if ( !fin )
    {
        ROS_INFO("Read Data Fail...");
        return 1;
    }
    
    //read image
    vector<string> rgb_files;
    string rgb_file;
    while (getline(fin,rgb_file) && ros::ok())
    {
        rgb_files.push_back ( dataset_dir+rgb_file );
    }
    fin.close();

    for(int i = 0;i < rgb_files.size();i++)
        ROS_INFO("%s",rgb_files[i].c_str());

    vector<Mat> descriptors;
    Ptr< Feature2D > detector = ORB::create();

    //calculate source keypoints and descriptor
    detector->detectAndCompute(source, Mat(), source_keypoints, source_descriptor );
    
    //int index = 1;
    //extract orb feature
    for ( string rgb_file:rgb_files )
    {
        Mat image = imread(rgb_file,0);

        if(image.empty())
            continue;

        vector<KeyPoint> keypoints; 
        Mat descriptor;
        detector->detectAndCompute( image, Mat(), keypoints, descriptor );
        descriptors.push_back( descriptor );
        //ROS_INFO("Extracting Features From Image:%d ",index++);
    }

    //construct database
    DBoW3::Database db( voc, false, 0);
    for ( int i=0; i<descriptors.size(); i++ )
        db.add(descriptors[i]);

    std::cout << "database info: " << db << std::endl;
    DBoW3::QueryResults ret;
    db.query(source_descriptor, ret, 3);
    std::cout << "searching for source image" << ret << std::endl;
    
    //std::cout << __FILE__ << __LINE__ << std::endl;

    //calculate robot pose according to EPNP
    return true;
}

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "relocalization_robot_service");

  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("voc_dir",voc_dir,"../voc/");
  private_nh.param<std::string>("dataset_dir",dataset_dir,"../data/");

  //advertise a service for getting a coverage plan
  ros::ServiceServer make_coverage_plan_srv = private_nh.advertiseService(
      "/sweeper/relocalization_robot_srv",
      relocalization_robot::GetRobotCurrentPose);

  ROS_INFO("Get Robot Current Pose Service Active...");

  ros::spin();

  return (0);
}


