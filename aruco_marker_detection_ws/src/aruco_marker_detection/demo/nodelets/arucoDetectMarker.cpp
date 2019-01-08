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

namespace aruco_marker_detection_ws{

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::CameraInfo> sync_pol;

class ArucoMarkerDetectionNodelet : public nodelet::Nodelet
{
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image>* sub_img_;
  message_filters::Subscriber<sensor_msgs::CameraInfo>* sub_info_;

  message_filters::Synchronizer<sync_pol>* sync;
  image_geometry::PinholeCameraModel model_;

  virtual void onInit();

  void imageCb(const sensor_msgs::ImageConstPtr& img_msg_,
               const sensor_msgs::CameraInfoConstPtr& info_msg_);
};

void ArucoMarkerDetectionNodelet::onInit()
{
  nh = getMTNodeHandle();

  // Read parameters
  ros::NodeHandle& private_nh = getMTPrivateNodeHandle();
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
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
  	model_.fromCameraInfo(info_msg_);
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
}
}

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aruco_marker_detection_ws::ArucoMarkerDetectionNodelet,nodelet::Nodelet);
