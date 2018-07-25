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

#include<pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>

#include <image_transport/image_transport.h>

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>


#include "InuSensor.h"
#include "DepthStream.h"
#include "AuxStream.h"
#include "InuDefs.h"
#include "InuSensor.h"
#include "InuStreams.h"
#include "OpticalData.h"
#include "VideoStream.h"
#include "WebCamStream.h"
#include "ImuStream.h"

namespace inuitive_ros_wrapper {

using namespace InuDev;

std::shared_ptr<InuDev::CWebCamStream>  webCamStream;
std::shared_ptr<InuDev::CVideoStream>  videoStream;
std::shared_ptr<InuDev::CDepthStream>  depthStream;
std::shared_ptr<InuDev::CDepthStream>  pointCloudStream;
std::shared_ptr<InuDev::CImuStream>  imuStream;

COpticalData optical_data;

sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
sensor_msgs::CameraInfoPtr right_cam_info_msg(new sensor_msgs::CameraInfo());
sensor_msgs::CameraInfoPtr web_cam_info_msg(new sensor_msgs::CameraInfo());

std::shared_ptr<InuDev::CInuSensor>  inuSensor;

class InuitiveRosWrapperNodelet : public nodelet::Nodelet {

    ros::NodeHandle nh;
    ros::NodeHandle nh_ns;
	
    boost::shared_ptr<boost::thread> device_poll_thread;

    image_transport::Publisher pub_depth;
    image_transport::Publisher pub_left;
    image_transport::Publisher pub_right;
    image_transport::Publisher pub_web;
    image_transport::Publisher pub_stereo;

    ros::Publisher pub_left_cam_info;
    ros::Publisher pub_right_cam_info;
    ros::Publisher pub_web_cam_info;
    ros::Publisher pub_imu;
    ros::Publisher pub_point_cloud;

    std::string right_frame_id;
    std::string left_frame_id;
    std::string depth_frame_id;
    std::string imu_frame_id;
    std::string web_frame_id;
    std::string point_cloud_frame_id;

    sensor_msgs::ImagePtr imageToROSmsg(const cv::Mat &img,
    const std::string &encodingType,
    const std::string &frameId,
    const ros::Time &stamp) {
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image& imgMessage = *ptr;
    imgMessage.header.stamp = stamp;
    imgMessage.header.frame_id = frameId;
    imgMessage.height = img.rows;
    imgMessage.width = img.cols;
    imgMessage.encoding = encodingType;
    int num = 1;
    imgMessage.is_bigendian = !(*(char *) &num == 1);
    imgMessage.step = img.cols * img.elemSize();
    size_t size = imgMessage.step * img.rows;
    imgMessage.data.resize(size);
    if (img.isContinuous())
        memcpy((char*) (&imgMessage.data[0]), img.data, size);
    else {
        uchar* opencvData = img.data;
        uchar* rosData = (uchar*) (&imgMessage.data[0]);
        for (unsigned int i = 0; i < img.rows; i++) {
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
        }
    }
    return ptr;
}

void publishVideoImage(const cv::Mat &img,
        const image_transport::Publisher &pub_img,
        const std::string &img_frame_id,
        const ros::Time &stamp) {
		cv::Mat img_rgb(img.rows,img.cols,CV_8UC3);
		for(int i =0;i < img.rows;i++)
			for(int j = 0;j < img.cols;j++)
			{
				img_rgb.at<cv::Vec3b>(i,j)[0] = img.at<cv::Vec4b>(i,j)[0];
				img_rgb.at<cv::Vec3b>(i,j)[1] = img.at<cv::Vec4b>(i,j)[1];
				img_rgb.at<cv::Vec3b>(i,j)[2] = img.at<cv::Vec4b>(i,j)[2];
			}
    pub_img.publish(imageToROSmsg(img_rgb, sensor_msgs::image_encodings::RGB8, img_frame_id, stamp));
}

//void publishVideoStereoImage(const cv::Mat &left, const cv::Mat& right, const image_trans)
void publishWebImage(const cv::Mat &img,
        const image_transport::Publisher &pub_img,
        const std::string &img_frame_id,
        const ros::Time &stamp) {
    pub_img.publish(imageToROSmsg(img, sensor_msgs::image_encodings::MONO8, img_frame_id, stamp));
}

void publishDepth(cv::Mat &depth, const ros::Time &stamp) {
		cv::Mat depth_convert(depth.rows,depth.cols,CV_16UC1);
    depth.convertTo(depth_convert, CV_16UC1);
    pub_depth.publish(imageToROSmsg(depth_convert, sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_id, stamp));
}

bool acc_get = false;
bool gro_get = false;
float acc_x_pre = 0;
float acc_y_pre = 0;
float acc_z_pre = 0;

float acc_x = 0;
float acc_y = 0;
float acc_z = 0;

float gro_x_pre = 0;
float gro_y_pre = 0;
float gro_z_pre = 0;

float gro_x = 0;
float gro_y = 0;
float gro_z = 0;

bool init = false;

void publishIMU(const InuDev::CImuFrame  &imu_frame, ros::Publisher &pub_imu,const ros::Time &stamp) {
	
	std::map< EImuType, CPoint3D >::iterator it; 	
	std::map< EImuType, CPoint3D > _SensorsData = imu_frame.SensorsData;
	sensor_msgs::Imu msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = imu_frame_id;	
	ROS_INFO("imu_sensor_size:%d",(int)(_SensorsData.size()));
	
	it = _SensorsData.begin();
	if(it->first == EImuType::eAccelerometer)
		ROS_INFO("eAccelerometer");
	else if(it->first == EImuType::eGyroscope)
		ROS_INFO("eGyroscope");

	if(!init){
		if(!acc_get){
			if(it->first == EImuType::eAccelerometer){

				acc_x = it->second[0];
				acc_y = it->second[1];
				acc_z = it->second[2];

				acc_x_pre = acc_x;
				acc_y_pre = acc_y;
				acc_z_pre = acc_z;

				acc_get = true;		
			}
		}

		if(!gro_get){
			if(it->first == EImuType::eGyroscope){

				gro_x = it->second[0];
				gro_y = it->second[1];
				gro_z = it->second[2];
				
				gro_x_pre = gro_x;
				gro_y_pre = gro_y;
				gro_z_pre = gro_z;
				
				gro_get = true;
			}
		}

		if(acc_get & gro_get){
			msg.linear_acceleration.x = acc_x;
			msg.linear_acceleration.y = acc_y;
			msg.linear_acceleration.z = acc_z;

			msg.angular_velocity.x = gro_x;
			msg.angular_velocity.y = gro_y;
			msg.angular_velocity.z = gro_z;
			
			init = true;
			acc_get = false;
			gro_get = false;

			pub_imu.publish(msg);
			
		}
	}
	else{
		if(it->first == EImuType::eAccelerometer){
			
			acc_x = it->second[0];
			acc_y = it->second[1];
			acc_z = it->second[2];

			acc_x_pre = acc_x;
			acc_y_pre = acc_y;
			acc_z_pre = acc_z;

			msg.linear_acceleration.x = acc_x;
			msg.linear_acceleration.y = acc_y;
			msg.linear_acceleration.z = acc_z;

			msg.angular_velocity.x = gro_x_pre;
			msg.angular_velocity.y = gro_y_pre;
			msg.angular_velocity.z = gro_z_pre;

			pub_imu.publish(msg);
			
		}
		else if(it->first == EImuType::eGyroscope){
			
			gro_x = it->second[0];
			gro_y = it->second[1];
			gro_z = it->second[2];
				
			gro_x_pre = gro_x;
			gro_y_pre = gro_y;
			gro_z_pre = gro_z;

			
			msg.linear_acceleration.x = acc_x_pre;
			msg.linear_acceleration.y = acc_y_pre;
			msg.linear_acceleration.z = acc_z_pre;

			msg.angular_velocity.x = gro_x;
			msg.angular_velocity.y = gro_y;
			msg.angular_velocity.z = gro_z;

			pub_imu.publish(msg);
		}

	}
	/*for(it = _SensorsData.begin();it != _SensorsData.end();it++)
	{
		if(it->first == EImuType::eAccelerometer)
		{
			msg.linear_acceleration.x = it->second[0];
			msg.linear_acceleration.y = it->second[1];
			msg.linear_acceleration.z = it->second[2];

			msg.linear_acceleration_covariance[0] = 0;
			msg.linear_acceleration_covariance[1] = 0;
			msg.linear_acceleration_covariance[2] = 0;

			msg.linear_acceleration_covariance[3] = 0;
			msg.linear_acceleration_covariance[4] = 0;
			msg.linear_acceleration_covariance[5] = 0;

			msg.linear_acceleration_covariance[6] = 0;
			msg.linear_acceleration_covariance[7] = 0;
			msg.linear_acceleration_covariance[8] = 0;

		}
		else if(it->first == EImuType::eGyroscope)
		{
			msg.angular_velocity.x = it->second[0];
			msg.angular_velocity.y = it->second[1];
			msg.angular_velocity.z = it->second[2];

			msg.angular_velocity_covariance[0] = 0;
			msg.angular_velocity_covariance[1] = 0;
			msg.angular_velocity_covariance[2] = 0;

			msg.angular_velocity_covariance[3] = 0;
			msg.angular_velocity_covariance[4] = 0;
			msg.angular_velocity_covariance[5] = 0;

			msg.angular_velocity_covariance[6] = 0;
			msg.angular_velocity_covariance[7] = 0;
			msg.angular_velocity_covariance[8] = 0;

		}
	}
	pub_imu.publish(msg);*/
}

void publishCamInfo(const sensor_msgs::CameraInfoPtr &cam_info_msg,
		const ros::Publisher &pub_cam_info,
		const ros::Time &stamp) {
	static int seq = 0;
	cam_info_msg->header.stamp = stamp;
	cam_info_msg->header.seq = seq;
	pub_cam_info.publish(cam_info_msg);
	++seq;
}

void fillVideoCamInfo(const sensor_msgs::CameraInfoPtr &left_cam_info_msg,
		const sensor_msgs::CameraInfoPtr &right_cam_info_msg,
		const std::string &left_frame_id,
		const std::string &right_frame_id,
		InuDev::COpticalData &optical_data,
		int height,
		int width)
{
	left_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
	right_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

	left_cam_info_msg->D.resize(5);
	right_cam_info_msg->D.resize(5);

	//D
	for(int i = 0;i < 5;i++)
	{
		left_cam_info_msg->D[i] = optical_data.LensDistortionsRealLeft[i];
		right_cam_info_msg->D[i] = optical_data.LensDistortionsRealRight[i];
	}
	//K
	{
		left_cam_info_msg->K[0] = optical_data.FocalLengthRealLeft[0];
		right_cam_info_msg->K[0] = optical_data.FocalLengthRealRight[0];

		left_cam_info_msg->K[2] = optical_data.OpticalCenterRealLeft[0];
		right_cam_info_msg->K[2] = optical_data.OpticalCenterRealRight[0];

		left_cam_info_msg->K[4] = optical_data.FocalLengthRealLeft[1];
		right_cam_info_msg->K[4] = optical_data.FocalLengthRealRight[1];

		left_cam_info_msg->K[5] = optical_data.OpticalCenterRealLeft[1];
		right_cam_info_msg->K[5] = optical_data.OpticalCenterRealRight[1];	

		left_cam_info_msg->K[8] = 1;
		right_cam_info_msg->K[8] = 1;	
	}

	//R
	CvMat* left_rotation = cvCreateMat(3,1,CV_32FC1); 
	cvmSet(left_rotation,0,0,optical_data.RotationRectifiedLeft[0]);
	cvmSet(left_rotation,1,0,optical_data.RotationRectifiedLeft[1]);
	cvmSet(left_rotation,2,0,optical_data.RotationRectifiedLeft[2]);

	CvMat* right_rotation = cvCreateMat(3,1,CV_32FC1); 
	cvmSet(right_rotation,0,0,optical_data.RotationRectifiedRight[0]);
	cvmSet(right_rotation,1,0,optical_data.RotationRectifiedRight[1]);
	cvmSet(right_rotation,2,0,optical_data.RotationRectifiedRight[2]);

	CvMat* left_rotation_matrix = cvCreateMat(3,3,CV_32FC1);
	CvMat* right_rotation_matrix = cvCreateMat(3,3,CV_32FC1);

	cvRodrigues2(left_rotation,left_rotation_matrix);
	cvRodrigues2(right_rotation,right_rotation_matrix);

	for(int i = 0;i < 3;i++)
		for(int j = 0;j < 3;j++)
		{
			left_cam_info_msg->R[i*3+j] = cvmGet(left_rotation_matrix,i,j);
			right_cam_info_msg->R[i*3+j] = cvmGet(right_rotation_matrix,i,j);
		}
	cvReleaseMat(&left_rotation);
	cvReleaseMat(&right_rotation);
	cvReleaseMat(&left_rotation_matrix);
	cvReleaseMat(&right_rotation_matrix);
	//P
	{
		left_cam_info_msg->P[0] = optical_data.FocalLengthBaseLeft[0];
		right_cam_info_msg->P[0] = optical_data.FocalLengthBaseRight[0];

		left_cam_info_msg->P[5] = optical_data.FocalLengthBaseLeft[1];
		right_cam_info_msg->P[5] = optical_data.FocalLengthBaseRight[1];

		left_cam_info_msg->P[2] = optical_data.OpticalCenterBaseLeft[0];
		right_cam_info_msg->P[2] = optical_data.OpticalCenterBaseRight[0];

		left_cam_info_msg->P[6] = optical_data.OpticalCenterBaseLeft[1];
		right_cam_info_msg->P[6] = optical_data.OpticalCenterBaseRight[1];

		left_cam_info_msg->P[3] = optical_data.TranslationRectifiedLeft[0];
		right_cam_info_msg->P[3] = optical_data.TranslationRectifiedRight[0];

		left_cam_info_msg->P[7] = optical_data.TranslationRectifiedLeft[1];
		right_cam_info_msg->P[7] = optical_data.TranslationRectifiedRight[1];

		left_cam_info_msg->P[10] = 1;
		right_cam_info_msg->P[10] = 1;		
	}


	left_cam_info_msg->width = right_cam_info_msg->width = width;
	left_cam_info_msg->height = right_cam_info_msg->height = height;
	left_cam_info_msg->header.frame_id = left_frame_id;
	right_cam_info_msg->header.frame_id = right_frame_id;
}

void fillWebCamInfo(const sensor_msgs::CameraInfoPtr &web_cam_info_msg,
		const std::string &web_frame_id,
		InuDev::COpticalData &optical_data,
		int height,
		int width)
{
	web_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

	//D
	web_cam_info_msg->D.resize(5);
	for(int i = 0;i < 5;i++)
	{
		web_cam_info_msg->D[i] = optical_data.LensDistortionsRealWebcam[i];
	}
	//K
	{
		web_cam_info_msg->K[0] = optical_data.FocalLengthRealWebcam[0];	
		web_cam_info_msg->K[2] = optical_data.OpticalCenterRealWebcam[0];
		web_cam_info_msg->K[4] = optical_data.FocalLengthRealWebcam[1];
		web_cam_info_msg->K[5] = optical_data.OpticalCenterRealWebcam[1];
		web_cam_info_msg->K[8] = 1;
	}

	//R
	{
		web_cam_info_msg->R[0] = 1;
		web_cam_info_msg->R[4] = 1;
		web_cam_info_msg->R[8] = 1;
	}

	//P
	{
		web_cam_info_msg->P[0] = optical_data.FocalLengthRealWebcam[0];
		web_cam_info_msg->P[5] = optical_data.FocalLengthRealWebcam[1];
		web_cam_info_msg->P[2] = optical_data.OpticalCenterRealWebcam[0];
		web_cam_info_msg->P[6] = optical_data.OpticalCenterRealWebcam[1];
		web_cam_info_msg->P[3] = 0;
		web_cam_info_msg->P[7] = 0;
		web_cam_info_msg->P[10] = 1;
	}

	web_cam_info_msg->width = width;
	web_cam_info_msg->height = height;
	web_cam_info_msg->header.frame_id = web_frame_id;
}

ros::Time getDepthImgStamp(const InuDev::CImageFrame &  depth_frame,bool reset = false) {
    static std::uint64_t img_time_beg = -1;
    static double img_ros_time_beg;
    if (reset) {
        img_time_beg = -1;
        return ros::Time::now();
    }
    if (img_time_beg == -1) {
        img_time_beg = depth_frame.Timestamp;
        img_ros_time_beg = ros::Time::now().toSec();
    }
    return ros::Time(img_ros_time_beg + (depth_frame.Timestamp - img_time_beg) * 0.000000001f);
}

ros::Time getColorImgStamp(const InuDev::CImageFrame*  video_frame,bool reset = false) {
    static std::uint64_t img_time_beg = -1;
    static double img_ros_time_beg;
    if (reset) {
        img_time_beg = -1;
        return ros::Time::now();
    }
    if (img_time_beg == -1) {
        img_time_beg = video_frame->Timestamp;
        img_ros_time_beg = ros::Time::now().toSec();
    }
    return ros::Time(img_ros_time_beg + (video_frame->Timestamp - img_time_beg) * 0.000000001f);
}

ros::Time getWebImgStamp( const InuDev::CImageFrame&  web_frame, bool reset = false) {
    static std::uint64_t img_time_beg = -1;
    static double img_ros_time_beg;
    if (reset) {
		  img_time_beg = -1;
		  return ros::Time::now();
    }
    if (img_time_beg == -1) {
        img_time_beg = web_frame.Timestamp;
        img_ros_time_beg = ros::Time::now().toSec();
    }
    return ros::Time(img_ros_time_beg + (web_frame.Timestamp - img_time_beg) * 0.000000001f);
}

ros::Time getIMUStamp(const InuDev::CImuFrame&  imu_frame,bool reset = false) {
    static std::uint64_t imu_time_beg = -1;
    static double imu_ros_time_beg;
    if (reset) {
        imu_time_beg = -1;
        return ros::Time::now();
    }
    if (imu_time_beg == -1) {
        imu_time_beg = imu_frame.Timestamp;
        imu_ros_time_beg = ros::Time::now().toSec();
    }
    return ros::Time(imu_ros_time_beg + (imu_frame.Timestamp - imu_time_beg) * 0.000000001f);
}

ros::Time HardTime2Softime(double hard_time)
{
  static bool isInited = false;
  static double soft_time_begin(0), hard_time_begin(0); 
  
  if (false == isInited)
  {
    soft_time_begin = ros::Time::now().toSec();
    hard_time_begin = hard_time;
    isInited = true;
  }

  return ros::Time(soft_time_begin + (hard_time - hard_time_begin) * 0.000000001f);
}

void  publishImuMessageReg(std::shared_ptr<InuDev::CImuStream> iStream,   // Parent Stream
		const InuDev::CImuFrame&  iFrame,              // Acquired frame
		InuDev::CInuError retCode)                       // Error code (eOK if frame was successfully acquired)
{
	if (retCode != InuDev::eOK)
	{
		std::cout << "Error in receiving frame: " << std::hex << int(retCode) << " " << std::string(retCode) << std::endl;
		return;
	}

	if (iFrame.Valid == false)
	{
		std::cout << "Frame  " << iFrame.FrameIndex << " is invalid" << std::endl;
		return;
	}

	//publishIMU(iFrame, pub_imu, getIMUStamp(iFrame));
	publishIMU(iFrame, pub_imu, HardTime2Softime(iFrame.Timestamp));
}

void  publishWebMessageReg(std::shared_ptr<InuDev::CWebCamStream> iStream,   // Parent Stream
		const InuDev::CImageFrame&  iFrame,              // Acquired frame
		InuDev::CInuError retCode)                       // Error code (eOK if frame was successfully acquired)
{

	if (retCode != InuDev::eOK)
	{
		std::cout << "Error in receiving frame: " << std::hex << int(retCode) << " " << std::string(retCode) << std::endl;
		return;
	}
	if (iFrame.Valid == false)
	{
		std::cout << "Frame  " << iFrame.FrameIndex << " is invalid" << std::endl;
		return;
	}

	//cv::Mat webImRGB(iFrame.Height(),iFrame.Width(),CV_16UC1);
	const byte* web_data_ptr = iFrame.GetData();

	sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
  sensor_msgs::Image& imgMessage = *ptr;
  imgMessage.header.stamp = HardTime2Softime(iFrame.Timestamp);
  imgMessage.header.frame_id = depth_frame_id;
  imgMessage.height = iFrame.Height();
  imgMessage.width = iFrame.Width();
  imgMessage.encoding = sensor_msgs::image_encodings::MONO8;
  int num = 1;
  imgMessage.is_bigendian = !(*(char *) &num == 1);
  imgMessage.step = iFrame.Width()*1;
  size_t size = imgMessage.step * iFrame.Height();
  imgMessage.data.resize(size);

  memcpy((char*) (&imgMessage.data[0]), web_data_ptr, size);

	fillWebCamInfo(web_cam_info_msg,web_frame_id,optical_data,iFrame.Height(),iFrame.Width());
	//memcpy(webImRGB.data,web_data_ptr,iFrame.Height()*iFrame.Width()*iFrame.BytesPerPixel());
	//publishCamInfo(web_cam_info_msg, pub_web_cam_info, getWebImgStamp(iFrame));
	//publishWebImage(webImRGB, pub_web, web_frame_id, getWebImgStamp(iFrame));
	publishCamInfo(web_cam_info_msg, pub_web_cam_info, HardTime2Softime(iFrame.Timestamp));
	//publishWebImage(webImRGB, pub_web, web_frame_id, HardTime2Softime(iFrame.Timestamp));
	pub_img.publish(ptr);
}

void  publishVideoMessageReg(std::shared_ptr<InuDev::CVideoStream> iStream,   // Parent Stream
		const InuDev::CVideoFrame&  iFrame,              // Acquired frame
		InuDev::CInuError retCode)                       // Error code (eOK if frame was successfully acquired)
{
	if (retCode != InuDev::eOK)
	{
		std::cout << "Error in receiving frame: " << std::hex << int(retCode) << " " << std::string(retCode) << std::endl;
		return;
	}

	if (iFrame.Valid == false)
	{
		std::cout << "Frame  " << iFrame.FrameIndex << " is invalid" << std::endl;
		return;
	}

	const InuDev::CImageFrame*  left_frame = iFrame.GetLeftFrame();
	const InuDev::CImageFrame*  right_frame = iFrame.GetRightFrame();

	cv::Mat rightImRGB(right_frame->Height(),right_frame->Width(),CV_8UC4);
	cv::Mat leftImRGB(left_frame->Height(),left_frame->Width(),CV_8UC4);
	const byte* right_data_ptr = right_frame->GetData();
	const byte* left_data_ptr = left_frame->GetData();


	fillVideoCamInfo(left_cam_info_msg,right_cam_info_msg,left_frame_id,right_frame_id,optical_data,left_frame->Height(),left_frame->Width());
	//right_frame
	//publishCamInfo(right_cam_info_msg, pub_right_cam_info, getColorImgStamp(right_frame));
	//publishVideoImage(rightImRGB, pub_right, right_frame_id, getColorImgStamp(right_frame));

	//left_frame
	//memcpy(leftImRGB.data,left_data_ptr,left_frame->Height()*left_frame->Width()*left_frame->BytesPerPixel());
	//publishCamInfo(left_cam_info_msg, pub_left_cam_info, getColorImgStamp(left_frame));
	publishCamInfo(left_cam_info_msg, pub_left_cam_info, HardTime2Softime(left_frame->Timestamp));
	//publishVideoImage(leftImRGB, pub_left, left_frame_id, getColorImgStamp(left_frame));
	//publishVideoImage(leftImRGB, pub_left, left_frame_id, HardTime2Softime(left_frame->Timestamp));

        // rifht frame
	//memcpy(rightImRGB.data,right_data_ptr,right_frame->Height()*right_frame->Width()*right_frame->BytesPerPixel());
	publishCamInfo(right_cam_info_msg, pub_right_cam_info, HardTime2Softime(right_frame->Timestamp));
	//publishVideoImage(rightImRGB, pub_right, right_frame_id, HardTime2Softime(right_frame->Timestamp));
}

void  publishDepthMessageReg(std::shared_ptr<InuDev::CDepthStream> iStream,   // Parent Stream
		const InuDev::CImageFrame &  iFrame,              // Acquired frame
		InuDev::CInuError retCode)                       // Error code (eOK if frame was successfully acquired)
{
	if (retCode != InuDev::eOK)
	{
		std::cout << "Error in receiving frame: " << std::hex << int(retCode) << " " << std::string(retCode) << std::endl;
		return;
	}

	if (iFrame.Valid == false)
	{
		std::cout << "Frame  " << iFrame.FrameIndex << " is invalid" << std::endl;
		return;
	}

	const byte* depth_data_ptr = iFrame.GetData();
	//cv::Mat depthImage(iFrame.Height(),iFrame.Width(),CV_16UC1);
	//memcpy(depthImage.data,depth_data_ptr,iFrame.Height()*iFrame.Width()*iFrame.BytesPerPixel());

	sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
  sensor_msgs::Image& imgMessage = *ptr;
  imgMessage.header.stamp = HardTime2Softime(iFrame.Timestamp);
  imgMessage.header.frame_id = depth_frame_id;
  imgMessage.height = iFrame.Height();
  imgMessage.width = iFrame.Width();
  imgMessage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  int num = 1;
  imgMessage.is_bigendian = !(*(char *) &num == 1);
  imgMessage.step = iFrame.Width()*2;
  size_t size = imgMessage.step * iFrame.Height();
  imgMessage.data.resize(size);
  
  memcpy((char*) (&imgMessage.data[0]), depth_data_ptr, size);

	//publishDepth(depthImage,getDepthImgStamp(iFrame));
	//publishDepth(depthImage,HardTime2Softime(iFrame.Timestamp));
	pub_depth.publish(ptr);
}

bool device_poll() {
	int frame_rate;
	nh.param(ros::this_node::getName()+"/frame_rate", frame_rate, 30); //default frame_rate = 50
	NODELET_INFO_STREAM("frame_rate:" << frame_rate);
	// Creation of CInuSensor object (Inuitive's sensor representation)
	inuSensor = InuDev::CInuSensor::Create();
	//inuSensor->Reset();
	// Initiate the sensor - it must be call before any access to the sensor. Sensor will start working in low power.
	InuDev::CSensorParams video_sensor_params;
	video_sensor_params.SensorRes =  InuDev::ESensorResolution::eFull;
	video_sensor_params.FPS = frame_rate;

	InuDev::CSensorParams web_sensor_params;
	web_sensor_params.SensorRes =  InuDev::ESensorResolution::eBinning;
	web_sensor_params.FPS = frame_rate;

	InuDev::CInuError retCode = inuSensor->Init();
	if (retCode != InuDev::eOK)
	{
		std::cout << "Failed to connect to Inuitive Sensor. Error: " << std::hex  << int(retCode) << " - " << std::string(retCode) << std::endl;
		return false;
	}
	std::cout << "Connected to Sensor" << std::endl;

	//get optical data about video cam and web cam
	retCode = inuSensor->GetOpticalData(optical_data);
	if (retCode != InuDev::eOK)
	{
		std::cout << "Failed to get video sensor optical params to Inuitive Sensor." << std::endl;
		return false;
	}

	retCode = inuSensor->SetSensorParams(video_sensor_params,eVideo);
	if (retCode != InuDev::eOK)
	{
		std::cout << "Failed to video_sensor_params. Error: " << std::hex  << int(retCode) << " - " << std::string(retCode) << std::endl;
		return false;
	}
	std::cout << "Video_sensor_params is ok" << std::endl;


	retCode = inuSensor->SetSensorParams(web_sensor_params,eWebCam);
	if (retCode != InuDev::eOK)
	{
		std::cout << "Failed to web_sensor_params. Error: " << std::hex  << int(retCode) << " - " << std::string(retCode) << std::endl;
		return false;
	}
	std::cout << "Web_sensor_params is ok" << std::endl;

	// Start acquiring frames - it must be call before starting acquiring any type of frames (depth, video, head, etc.)
	retCode = inuSensor->Start();
	if (retCode != InuDev::eOK)
	{
		std::cout << "Failed to start to Inuitive Sensor." << std::endl;
		return false;
	}
	std::cout << "Sensor is started" << std::endl;

	//construct webCamStream
	webCamStream = inuSensor->CreateWebCamStream();
	if (webCamStream == nullptr)
	{
		std::cout << "Unexpected error, failed to get web Stream" << std::endl;
		return false;
	}

	retCode = webCamStream->Init( InuDev::CWebCamStream::EOutputType::eRaw);
	if (retCode != InuDev::eOK)
	{
		std::cout << "web initiation error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		return false;
	}
	std::cout << "Web Stream is initialized" << std::endl;

	//construct videoStream
	videoStream = inuSensor->CreateVideoStream();
	if (videoStream == nullptr)
	{
		std::cout << "Unexpected error, failed to get Video Stream" << std::endl;
		return false;
	}

	retCode = videoStream->Init(true,InuDev::CVideoStream::EOutputType::eRGBA);
	if (retCode != InuDev::eOK)
	{
		std::cout << "video initiation error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		return false;
	}
	std::cout << "Vedio Stream is initialized" << std::endl;		

	//construct depthStream
	depthStream = inuSensor->CreateDepthStream();
	if (depthStream == nullptr)
	{
		std::cout << "Unexpected error, failed to get Depth Stream" << std::endl;
		return false;
	}
	retCode = depthStream->Init(InuDev::CDepthStream::EOutputType::eDepth);

	if (retCode != InuDev::eOK)
	{
		std::cout << "Depth initiation error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		return false;
	}
	std::cout << "Depth Stream is initialized" << std::endl;

	//construct imuStream
	imuStream = inuSensor->CreateImuStream();
	if (imuStream == nullptr)
	{
		std::cout << "Unexpected error, failed to get imu Stream" << std::endl;
		return false;
	}

	retCode = imuStream->Init();
	if (retCode != InuDev::eOK)
	{
		std::cout << "imu initiation error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		return false;
	}
	std::cout << "imu Stream is initialized" << std::endl;

	//start imu stream
	retCode = imuStream->Start();
	if (retCode != InuDev::eOK)
	{
		std::cout << "Start error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		return false;
	}
	std::cout << "imu Stream is started" << std::endl;
	//start web cam stream
	retCode = webCamStream->Start();
	if (retCode != InuDev::eOK)
	{
		std::cout << "Start error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		return false;
	}
	std::cout << "Web Stream is started" << std::endl;

	//start video stream
	retCode = videoStream->Start();
	if (retCode != InuDev::eOK)
	{
		std::cout << "Start error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		return false;
	}
	std::cout << "Vedio Stream is started" << std::endl;

	//start depth stream
	retCode = depthStream->Start();
	if (retCode != InuDev::eOK)
	{
		std::cout << "Start error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		return false;
	}
	std::cout << "Depth Stream is started" << std::endl;

	retCode = imuStream->Register(std::bind(&InuitiveRosWrapperNodelet::publishImuMessageReg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	if (retCode != InuDev::eOK)
	{
		std::cout << "IMU register error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		return false;
	}

	retCode = webCamStream->Register(std::bind(&InuitiveRosWrapperNodelet::publishWebMessageReg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	if (retCode != InuDev::eOK)
	{
		std::cout << "WEB register error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		return false;
	}

	retCode = videoStream->Register(std::bind(&InuitiveRosWrapperNodelet::publishVideoMessageReg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	if (retCode != InuDev::eOK)
	{
		std::cout << "VIDEO register error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		return false;
	}

	retCode = depthStream->Register(std::bind(&InuitiveRosWrapperNodelet::publishDepthMessageReg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	if (retCode != InuDev::eOK)
	{
		std::cout << "DEPTH register error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		return false;
	}
}

void onInit() {

	std::string img_topic = "image_rect_color";
	std::string left_topic = "left/" + img_topic;
	std::string left_cam_info_topic = "left/camera_info";
	left_frame_id = "/inuitive_left_frame";

	std::string right_topic = "right/" + img_topic;
	std::string right_cam_info_topic = "right/camera_info";
	right_frame_id = "/inuitive_right_frame";

	std::string depth_topic = "depth/depth_registered";
	depth_frame_id = "/inuitive_depth_frame";

	std::string imu_topic = "imu";
	imu_frame_id = "/inuitive_imu_frame";

	std::string web_topic = "web/" + img_topic;
        std::string stereo_topic = "stereo/stereo_image_rect_color"; 
	std::string web_cam_info_topic = "web/camera_info";
	web_frame_id = "inuitive_web_frame";

	std::string point_cloud_topic = "point_cloud";
	std::string point_cloud_cam_info_topic = "point_cloud/camera_info";
	point_cloud_frame_id = "inuitive_point_cloud_frame";

	nh = getMTNodeHandle();
	nh_ns = getMTPrivateNodeHandle();
	nh_ns.getParam("left_topic", left_topic);
	nh_ns.getParam("left_cam_info_topic", left_cam_info_topic);
	nh_ns.getParam("right_topic", right_topic);
	nh_ns.getParam("right_cam_info_topic", right_cam_info_topic);
	nh_ns.getParam("depth_topic", depth_topic);
	nh_ns.getParam("imu_topic", imu_topic);

	nh_ns.getParam("left_frame_id", left_frame_id);
	nh_ns.getParam("right_frame_id", right_frame_id);

	image_transport::ImageTransport it_inuitive(nh);

	pub_left = it_inuitive.advertise(left_topic, 100);
	NODELET_INFO_STREAM("Advertized on topic " << left_topic);
	pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1);
	NODELET_INFO_STREAM("Advertized on topic " << left_cam_info_topic);

	pub_right = it_inuitive.advertise(right_topic, 100);
	NODELET_INFO_STREAM("Advertized on topic " << right_topic);
	pub_right_cam_info = nh.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1);
	NODELET_INFO_STREAM("Advertized on topic " << right_cam_info_topic);

	pub_web = it_inuitive.advertise(web_topic, 100);
        pub_stereo = it_inuitive.advertise(stereo_topic, 100);
	NODELET_INFO_STREAM("Advertized on topic " << web_topic);
	pub_web_cam_info = nh.advertise<sensor_msgs::CameraInfo>(web_cam_info_topic, 1);
	NODELET_INFO_STREAM("Advertized on topic " << web_cam_info_topic);

	pub_depth = it_inuitive.advertise(depth_topic, 100);
	NODELET_INFO_STREAM("Advertized on topic " << depth_topic);

	pub_imu = nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
	NODELET_INFO_STREAM("Advertized on topic " << imu_topic);

	pub_point_cloud = nh.advertise<sensor_msgs::PointCloud>(point_cloud_topic, 1);
	NODELET_INFO_STREAM("Advertized on topic " << point_cloud_topic);

	device_poll_thread = boost::shared_ptr<boost::thread>
		(new boost::thread(boost::bind(&InuitiveRosWrapperNodelet::device_poll, this)));
}

~InuitiveRosWrapperNodelet() {		
	//depthstream
	if(depthStream != NULL){
		InuDev::CInuError retCode = depthStream->Stop();
		if (retCode != InuDev::eOK){
			std::cout << "Depth Stop error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		}
		std::cout << "Depth frames acquisition stopped" << std::endl;
		retCode = depthStream->Terminate();
		if (retCode != InuDev::eOK) {
			std::cout << "Depth Terminate error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		}
		std::cout << "Depth Stream was finalized" << std::endl;
	}
	//videostream
	if(videoStream != NULL){
		InuDev::CInuError retCode = videoStream->Stop();
		if (retCode != InuDev::eOK){
			std::cout << "DepvideoStreamth Stop error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		}
		std::cout << "videoStream frames acquisition stopped" << std::endl;
		retCode = videoStream->Terminate();
		if (retCode != InuDev::eOK) {
			std::cout << "videoStream Terminate error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		}
		std::cout << "videoStream Stream was finalized" << std::endl;
	}
	//imustream
	if(imuStream != NULL){
		InuDev::CInuError retCode = imuStream->Stop();
		if (retCode != InuDev::eOK){
			std::cout << "imuStream Stop error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		}
		std::cout << "imuStream frames acquisition stopped" << std::endl;
		retCode = imuStream->Terminate();
		if (retCode != InuDev::eOK) {
			std::cout << "imuStream Terminate error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		}
		std::cout << "imuStream Stream was finalized" << std::endl;
	}
	//wedcamstream
	if(webCamStream != NULL){
		InuDev::CInuError retCode = webCamStream->Stop();
		if (retCode != InuDev::eOK){
			std::cout << "webCamStream Stop error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		}
		std::cout << "webCamStream frames acquisition stopped" << std::endl;
		retCode = webCamStream->Terminate();
		if (retCode != InuDev::eOK) {
			std::cout << "webCamStream Terminate error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		}
		std::cout << "webCamStream Stream was finalized" << std::endl;
	}
	//pointcloudstream
	if(pointCloudStream != NULL){
		InuDev::CInuError retCode = pointCloudStream->Stop();
		if (retCode != InuDev::eOK){
			std::cout << "pointCloudStream Stop error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		}
		std::cout << "pointCloudStream frames acquisition stopped" << std::endl;
		retCode = pointCloudStream->Terminate();
		if (retCode != InuDev::eOK) {
			std::cout << "pointCloudStream Terminate error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		}
		std::cout << "pointCloudStream Stream was finalized" << std::endl;
	}

	if(inuSensor != NULL){
		InuDev::CInuError retCode = inuSensor->Stop();
		if (retCode != InuDev::eOK)
		{
			std::cout << "Failed to stop CInuSensor, error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		}
		std::cout << "Sensor was stopped" << std::endl;

		// Turn off the sensor
		retCode = inuSensor->Terminate();
		if (retCode != InuDev::eOK)
		{
			std::cout << "Failed to terminate CInuSensor, error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		}
		std::cout << "Disconnected from Sensor" << std::endl;
	}
}

};

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(inuitive_ros_wrapper::InuitiveRosWrapperNodelet, nodelet::Nodelet);
