#include <boost/thread/lock_guard.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "aruco.h"
#include <opencv2/highgui/highgui.hpp>

using namespace aruco;
using namespace cv;
using namespace std;

namespace aruco_marker_detection_ws{

using namespace message_filters::sync_policies;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::CameraInfo> sync_pol;

class ArucoMarkerDetectionNodelet : public nodelet::Nodelet
{
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image>* sub_img_;
  message_filters::Subscriber<sensor_msgs::CameraInfo>* sub_info_;

  message_filters::Synchronizer<sync_pol>* sync;
  //image_geometry::PinholeCameraModel model_;

  virtual void onInit();

  void imageCb(const sensor_msgs::ImageConstPtr& img_msg_,
               const sensor_msgs::CameraInfoConstPtr& info_msg_);
};

void ArucoMarkerDetectionNodelet::onInit()
{
  nh = getMTNodeHandle();
  
  ros::NodeHandle& private_nh = getMTPrivateNodeHandle();
  int queue_size;
  private_nh.param("queue_size", queue_size, 15);
  bool use_exact_sync;
  private_nh.param("exact_sync", use_exact_sync, false);


  std::string img_topic = "/mynteye/left/image_raw";
  std::string camera_info_topic = "/mynteye/left/camera_info";
 
  sub_img_ = new message_filters::Subscriber<sensor_msgs::Image>(nh,img_topic,1);
  sub_info_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,camera_info_topic,1);
  
  NODELET_INFO_STREAM("Subscribe on topic:" << img_topic);
  NODELET_INFO_STREAM("Subscribe on topic:" << camera_info_topic);

  sync = new message_filters::Synchronizer<sync_pol>(sync_pol(queue_size),*sub_img_,*sub_info_);
  sync->registerCallback(boost::bind(&ArucoMarkerDetectionNodelet::imageCb,this,_1,_2)); 
}

void ArucoMarkerDetectionNodelet::imageCb(const sensor_msgs::ImageConstPtr& img_msg_,
                                      	  const sensor_msgs::CameraInfoConstPtr& info_msg_)
{
  	//model_.fromCameraInfo(info_msg_);
	cv::Mat K(3,3,CV_64F);
	cv::Mat D(4,1,CV_64F);
	K.at<double>(0,0) = info_msg_->P[0];
	K.at<double>(0,1) = info_msg_->P[1];
	K.at<double>(0,2) = info_msg_->P[2];

	K.at<double>(1,0) = info_msg_->P[4];
	K.at<double>(1,1) = info_msg_->P[5];
	K.at<double>(1,2) = info_msg_->P[6];


	K.at<double>(2,0) = info_msg_->P[8];
	K.at<double>(2,1) = info_msg_->P[9];
	K.at<double>(2,2) = info_msg_->P[10];

	D.at<double>(0,0) = 0;
	D.at<double>(1,0) = 0;
	D.at<double>(2,0) = 0;
	D.at<double>(3,0) = 0;

	aruco::CameraParameters CamParam(K,D,cv::Size(info_msg_->width,info_msg_->height));

  	sensor_msgs::ImageConstPtr img_msg = img_msg_;
  	cv_bridge::CvImagePtr cv_ptr;
  
  	try{
  		cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat img_mat(cv_ptr->image);
	
	float markerSize = 0.119;
	
	MarkerDetector MDetector;
	MDetector.setDictionary("TAG25h9",0.f);
	std::vector<Marker> Markers = MDetector.detect(img_mat, CamParam, markerSize);

	for(unsigned int i = 0;i < Markers.size();i++){
		ROS_INFO_STREAM("Marker:" << Markers[i]);
		Markers[i].draw(img_mat, cv::Scalar(0, 0, 255), 2);
	}

	cv::namedWindow("InImage", 1);
	cv::imshow("InImage",img_mat);
	while (char(cv::waitKey(0)) != 27)
		;
}
}

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aruco_marker_detection_ws::ArucoMarkerDetectionNodelet,nodelet::Nodelet);
