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


namespace inuitive_ros_wrapper {

using namespace InuDev;

InuDev::CImageFrame  depth_frame;
InuDev::CImageFrame  point_cloud_frame;
InuDev::CVideoFrame  video_frame;
const InuDev::CImageFrame*  left_frame;
const InuDev::CImageFrame*  right_frame;
InuDev::CImuFrame  imu_frame;
InuDev::CImageFrame  web_frame;

std::shared_ptr<InuDev::CWebCamStream>  webCamStream;
std::shared_ptr<InuDev::CVideoStream>  videoStream;
std::shared_ptr<InuDev::CDepthStream>  depthStream;
std::shared_ptr<InuDev::CDepthStream>  pointCloudStream;
std::shared_ptr<InuDev::CAuxStream>  auxStream;


std::shared_ptr<InuDev::CInuSensor>  inuSensor;

class InuitiveRosWrapperNodelet : public nodelet::Nodelet {

    ros::NodeHandle nh;
    ros::NodeHandle nh_ns;

		boost::shared_ptr<boost::thread> device_poll_thread;

    image_transport::Publisher pub_depth;
    image_transport::Publisher pub_left;
    image_transport::Publisher pub_right;
		image_transport::Publisher pub_web;

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
//std::cout << __FILE__ << __LINE__ << std::endl;
    if (img.isContinuous())
        memcpy((char*) (&imgMessage.data[0]), img.data, size);
    else {
        uchar* opencvData = img.data;
        uchar* rosData = (uchar*) (&imgMessage.data[0]);
        for (unsigned int i = 0; i < img.rows; i++) {
//std::cout << __FILE__ << __LINE__ << std::endl;
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
//std::cout << __FILE__ << __LINE__ << std::endl;
        }
    }
//std::cout << __FILE__ << __LINE__ << std::endl;
    return ptr;
}

void publishImage(const cv::Mat &img,
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

void publishDepth(cv::Mat &depth, const ros::Time &stamp) {
		cv::Mat depth_convert(depth.rows,depth.cols,CV_16UC1);
    depth.convertTo(depth_convert, CV_16UC1);
    pub_depth.publish(imageToROSmsg(depth_convert, sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_id, stamp));
}

void publishIMU(const InuDev::CImuFrame  &imu_frame, ros::Publisher &pub_imu,const ros::Time &stamp) {

		std::map< EAuxType, CPoint3D >::iterator it; 	
		std::map< EAuxType, CPoint3D > _SensorsData = imu_frame.SensorsData;
    sensor_msgs::Imu msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = imu_frame_id;

		for(it = _SensorsData.begin();it != _SensorsData.end();it++)
		{

			if(it->first == EAuxType::eAccelerometer)
			{
				msg.linear_acceleration.x = it->second[0];
				msg.linear_acceleration.y = it->second[1];
				msg.linear_acceleration.z = it->second[2];

				msg.linear_acceleration_covariance[0] = 0.04;
				msg.linear_acceleration_covariance[1] = 0;
				msg.linear_acceleration_covariance[2] = 0;

				msg.linear_acceleration_covariance[3] = 0;
				msg.linear_acceleration_covariance[4] = 0.04;
				msg.linear_acceleration_covariance[5] = 0;

				msg.linear_acceleration_covariance[6] = 0;
				msg.linear_acceleration_covariance[7] = 0;
				msg.linear_acceleration_covariance[8] = 0.04;

			}
			else if(it->first == EAuxType::eGyroscope)
			{
				msg.angular_velocity.x = it->second[0];
				msg.angular_velocity.y = it->second[1];
				msg.angular_velocity.z = it->second[2];

				msg.angular_velocity_covariance[0] = 0.02;
				msg.angular_velocity_covariance[1] = 0;
				msg.angular_velocity_covariance[2] = 0;

				msg.angular_velocity_covariance[3] = 0;
				msg.angular_velocity_covariance[4] = 0.02;
				msg.angular_velocity_covariance[5] = 0;

				msg.angular_velocity_covariance[6] = 0;
				msg.angular_velocity_covariance[7] = 0;
				msg.angular_velocity_covariance[8] = 0.02;
			
			}
		}
		pub_imu.publish(msg);
}

void publishCamInfo(const sensor_msgs::CameraInfoPtr &cam_info_msg,
        const ros::Publisher &pub_cam_info,
        const ros::Time &stamp) {
//std::cout << __FILE__ << __LINE__ << std::endl;
    static int seq = 0;
    cam_info_msg->header.stamp = stamp;
    cam_info_msg->header.seq = seq;
//std::cout << __FILE__ << __LINE__ << std::endl;
    pub_cam_info.publish(cam_info_msg);
//std::cout << __FILE__ << __LINE__ << std::endl;
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
		}

		left_cam_info_msg->K[8] = 1;
		right_cam_info_msg->K[8] = 1;	

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
		web_cam_info_msg->D.resize(5);
		
		//D
		for(int i = 0;i < 5;i++)
		{
		  web_cam_info_msg->D[i] = optical_data.LensDistortionsRealLeft[i];
		}
		//K
		{
			web_cam_info_msg->K[0] = optical_data.FocalLengthRealWebcam[0];	
			web_cam_info_msg->K[2] = optical_data.OpticalCenterRealWebcam[0];
			web_cam_info_msg->K[4] = optical_data.FocalLengthRealWebcam[1];
			web_cam_info_msg->K[5] = optical_data.OpticalCenterRealWebcam[1];
		}
		web_cam_info_msg->K[8] = 1;

		web_cam_info_msg->R[0] = 1;
		web_cam_info_msg->R[4] = 1;
		web_cam_info_msg->R[8] = 1;

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

ros::Time getDepthImgStamp(bool reset = false) {
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
    return ros::Time(img_ros_time_beg + (depth_frame.Timestamp - img_time_beg) * 0.0001f);
}

ros::Time getColorImgStamp(bool reset = false) {
    static std::uint64_t img_time_beg = -1;
    static double img_ros_time_beg;
    if (reset) {
        img_time_beg = -1;
        return ros::Time::now();
    }
    if (img_time_beg == -1) {
std::cout << __FILE__ << __LINE__ << std::endl;
        img_time_beg = video_frame.Timestamp;
std::cout << __FILE__ << __LINE__ << std::endl;
        img_ros_time_beg = ros::Time::now().toSec();
std::cout << __FILE__ << __LINE__ << std::endl;
    }
    return ros::Time(img_ros_time_beg + (video_frame.Timestamp - img_time_beg) * 0.0001f);
}
ros::Time getWebImgStamp(bool reset = false) {
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
    return ros::Time(img_ros_time_beg + (web_frame.Timestamp - img_time_beg) * 0.0001f);
}

ros::Time getIMUStamp(bool reset = false) {
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
    return ros::Time(imu_ros_time_beg + (imu_frame.Timestamp - imu_time_beg) * 0.0001f);
}

ros::Time getIMUStamp(const InuDev::CImuFrame *imu_frame, bool reset = false) {
    static std::uint64_t imu_time_beg = -1;
    static double imu_ros_time_beg;
    if (reset) {
        imu_time_beg = -1;
        return ros::Time::now();
    }
    if (imu_time_beg == -1) {
        imu_time_beg = imu_frame->Timestamp;
        imu_ros_time_beg = ros::Time::now().toSec();
    }
    return ros::Time(imu_ros_time_beg + (imu_frame->Timestamp - imu_time_beg) * 0.0001f);
}

bool device_poll() {
		// Creation of CInuSensor object (Inuitive's sensor representation)
    inuSensor = InuDev::CInuSensor::Create();
		inuSensor->Reset();
		
    // Initiate the sensor - it must be call before any access to the sensor. Sensor will start working in low power.
		InuDev::CSensorParams sensor_params;
		sensor_params.SensorRes =  InuDev::ESensorResolution::eBinning;
		sensor_params.FPS = 30;

    InuDev::CInuError retCode = inuSensor->Init();
    if (retCode != InuDev::eOK)
    {
        std::cout << "Failed to connect to Inuitive Sensor. Error: " << std::hex  << int(retCode) << " - " << std::string(retCode) << std::endl;
        return false;
    }
    std::cout << "Connected to Sensor" << std::endl;

		retCode = inuSensor->SetSensorParams(sensor_params, InuDev::ECameraName::eVideo);
		if (retCode != InuDev::eOK)
    {
        std::cout << "Failed to set video sensor params to Inuitive Sensor." << std::endl;
        return false;
    }
		
		//get optical data about video cam and web cam
		std::cout << __FILE__ << __LINE__ << std::endl;
		COpticalData optical_data;
		inuSensor->GetOpticalData(optical_data);
		std::cout << __FILE__ << __LINE__ << std::endl;

    // Start acquiring frames - it must be call before starting acquiring any type of frames (depth, video, head, etc.)
    retCode = inuSensor->Start();
    if (retCode != InuDev::eOK)
    {
        std::cout << "Failed to start to Inuitive Sensor." << std::endl;
        return false;
    }
    std::cout << "Sensor is started" << std::endl;
   
		//camera information
		sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr right_cam_info_msg(new sensor_msgs::CameraInfo());
		sensor_msgs::CameraInfoPtr web_cam_info_msg(new sensor_msgs::CameraInfo());
    //sensor_msgs::CameraInfoPtr depth_cam_info_msg(new sensor_msgs::CameraInfo());
	
		//construct webCamStream
		webCamStream = inuSensor->CreateWebCamStream();
		if (webCamStream == nullptr)
		{
				std::cout << "Unexpected error, failed to get web Stream" << std::endl;
				return false;
		}
		 
		retCode = webCamStream->Init( InuDev::CWebCamStream::EOutputType::eDefault);
		if (retCode != InuDev::eOK)
		{
				std::cout << "web initiation error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
				return false;
		}
		std::cout << "web Stream is initialized" << std::endl;

		retCode = webCamStream->Start();
		if (retCode != InuDev::eOK)
		{
				std::cout << "Start error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
				return false;
		}
		std::cout << "Web Stream is started" << std::endl;
		
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

		retCode = videoStream->Start();

		if (retCode != InuDev::eOK)
		{
				std::cout << "Start error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
				return false;
		}
		std::cout << "Vedio Stream is started" << std::endl;	

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
		retCode = depthStream->Start();

		if (retCode != InuDev::eOK)
		{
				std::cout << "Start error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
				return false;
		}
		std::cout << "Depth Stream is started" << std::endl;

		//construct imuStream
		auxStream = inuSensor->CreateAuxStream();
		if (auxStream == nullptr)
		{
				std::cout << "Unexpected error, failed to get web Stream" << std::endl;
				return false;
		}
		 
		retCode = auxStream->Init();
		if (retCode != InuDev::eOK)
		{
				std::cout << "aux initiation error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
				return false;
		}
		std::cout << "aux Stream is initialized" << std::endl;
		retCode = auxStream->Start();
			if (retCode != InuDev::eOK)
			{
					std::cout << "Start error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
					return false;
			}
			std::cout << "aux Stream is started" << std::endl;

		//loop
    while (nh_ns.ok()) {

        int left_SubNumber = pub_left.getNumSubscribers();
        int right_SubNumber = pub_right.getNumSubscribers();
        int depth_SubNumber = pub_depth.getNumSubscribers();
        int imu_SubNumber = pub_imu.getNumSubscribers();
				int web_SubNumber = pub_web.getNumSubscribers();
        int pointcloud_SubNumber = pub_point_cloud.getNumSubscribers();

        bool runLoop = (left_SubNumber + right_SubNumber + depth_SubNumber + imu_SubNumber + web_SubNumber + pointcloud_SubNumber) > 0;
        if (runLoop) {
            bool img_get = false;
            if ((left_SubNumber > 0 ||  right_SubNumber > 0) && videoStream->GetFrame(video_frame) == InuDev::eOK) {
      							//std::cout << __FILE__ << __LINE__ << std::endl;
										std::cout << "video_frame_stamp:" << video_frame.Timestamp << std::endl;
										std::cout << "video_frame_index:" << video_frame.FrameIndex << std::endl;

										left_frame = video_frame.GetLeftFrame();
										right_frame = video_frame.GetRightFrame();
										
										static int image_count = 0;
										image_count ++;

										std::cout << "image_count:" << image_count << std::endl;
										std::cout << "Color left Image Height:" << left_frame->Height() << "Image Width:" << left_frame->Width() << std::endl;
										std::cout << "Color left Bytes Per Pixel:" << left_frame->BytesPerPixel() << std::endl;
										std::cout << "Color left Image Buffer Size:" << left_frame->BufferSize() << std::endl;

										std::cout << "Color right Image Height:" << right_frame->Height() << "Image Width:" << right_frame->Width() << std::endl;
										std::cout << "Color right Bytes Per Pixel:" << right_frame->BytesPerPixel() << std::endl;
										std::cout << "Color right Image Buffer Size:" << right_frame->BufferSize() << std::endl;

										cv::Mat rightImRGB(right_frame->Height(),right_frame->Width(),CV_8UC4);
										cv::Mat leftImRGB(left_frame->Height(),left_frame->Width(),CV_8UC4);
										const byte* right_data_ptr = right_frame->GetData();
										const byte* left_data_ptr = left_frame->GetData();

									
									fillVideoCamInfo(left_cam_info_msg,right_cam_info_msg,left_frame_id,right_frame_id,optical_data,right_frame->Height(),right_frame->Width());
									//right_frame
									memcpy(rightImRGB.data,right_data_ptr,right_frame->Height()*right_frame->Width()*right_frame->BytesPerPixel());
									publishCamInfo(right_cam_info_msg, pub_right_cam_info, getColorImgStamp());
									publishImage(rightImRGB, pub_right, right_frame_id, getColorImgStamp());
		
									//left_frame
									memcpy(leftImRGB.data,left_data_ptr,left_frame->Height()*left_frame->Width()*left_frame->BytesPerPixel());
									publishCamInfo(left_cam_info_msg, pub_left_cam_info, getColorImgStamp());
									publishImage(leftImRGB, pub_left, left_frame_id, getColorImgStamp());

								
		              img_get = true;

            }
            if (depth_SubNumber > 0 && depthStream->GetFrame(depth_frame) == InuDev::eOK) {
								static int depth_count = 0;
								depth_count ++;

								std::cout << "depth_count:" << depth_count << std::endl;
								std::cout << "depth Height:" << depth_frame.Height() << "Image Width:" << depth_frame.Width() << std::endl;
								std::cout << "depth Bytes Per Pixel:" << depth_frame.BytesPerPixel() << std::endl;
								std::cout << "depth Buffer Size:" << depth_frame.BufferSize() << std::endl;

								const byte* depth_data_ptr = depth_frame.GetData();
								cv::Mat depthImage(depth_frame.Height(),depth_frame.Width(),CV_16UC1);
								memcpy(depthImage.data,depth_data_ptr,depth_frame.Height()*depth_frame.Width()*depth_frame.BytesPerPixel());
								
								publishDepth(depthImage,getDepthImgStamp());
								img_get = true;
            }
#if 0
						if(pointcloud_SubNumber > 0 && pointCloudStream->GetFrame(point_cloud_frame) == InuDev::eOK)
						{
								static int point_cloud_count = 0;
								point_cloud_count ++;

								std::cout << "point_cloud_count:" << point_cloud_count << std::endl;
								std::cout << "point cloud Height:" << point_cloud_frame.Height() << "point cloud Width:" << point_cloud_frame.Width() << std::endl;
								std::cout << "point cloud Bytes Per Pixel:" << point_cloud_frame.BytesPerPixel() << std::endl;
								std::cout << "point cloud Buffer Size:" << point_cloud_frame.BufferSize() << std::endl;

								const byte* point_cloud_data_ptr = point_cloud_frame.GetData();

								cv::Mat point_cloud(point_cloud_frame.Height(),point_cloud_frame.Width(),CV_32FC1);
								memcpy(point_cloud.data,point_cloud_data_ptr,point_cloud_frame.Height()*point_cloud_frame.Width()*point_cloud_frame.BytesPerPixel());

								int point_cloud_num = point_cloud_frame.Width();
								sensor_msgs::PointCloud ros_point_cloud;
								ros_point_cloud.header.frame_id = point_cloud_frame_id;
								ros_point_cloud.header.stamp = getDepthImgStamp();

								ros_point_cloud.channels.resize(3);
								ros_point_cloud.channels[0].name = "r";
								ros_point_cloud.channels[0].values.resize(point_cloud_num);
								ros_point_cloud.channels[1].name = "g";
								ros_point_cloud.channels[1].values.resize(point_cloud_num);
								ros_point_cloud.channels[2].name = "b";
								ros_point_cloud.channels[2].values.resize(point_cloud_num);
		
								ros_point_cloud.points.resize(point_cloud_num);

								for(int i = 0;i < point_cloud_num;i++)
								{
									ros_point_cloud.points[i].x = point_cloud.at<float>(0,i)/1000;
									ros_point_cloud.points[i].y = point_cloud.at<float>(1,i)/1000;
									ros_point_cloud.points[i].z = point_cloud.at<float>(2,i)/1000;

									ros_point_cloud.channels[0].values[i] = 255;
									ros_point_cloud.channels[1].values[i] = 255;
									ros_point_cloud.channels[2].values[i] = 255;
								}

								pub_point_cloud.publish(ros_point_cloud);

						}
#endif
						if(web_SubNumber > 0 && webCamStream->GetFrame(web_frame) == InuDev::eOK)
						{
							static int web_count = 0;
							web_count ++;

							std::cout << "web_count:" << web_count << std::endl;
							std::cout << "web Image Height:" << web_frame.Height() << "Image Width:" << web_frame.Width() << std::endl;
							std::cout << "web Bytes Per Pixel:" << web_frame.BytesPerPixel() << std::endl;
							std::cout << "web Image Buffer Size:" << web_frame.BufferSize() << std::endl;

							cv::Mat webImRGB(web_frame.Height(),web_frame.Width(),CV_8UC4);
							const byte* web_data_ptr = web_frame.GetData();

							fillWebCamInfo(web_cam_info_msg,web_frame_id,optical_data,web_frame.Height(),web_frame.Width());
							memcpy(webImRGB.data,web_data_ptr,web_frame.Height()*web_frame.Width()*web_frame.BytesPerPixel());
              publishCamInfo(web_cam_info_msg, pub_web_cam_info, getWebImgStamp());
              publishImage(webImRGB, pub_web, web_frame_id, getWebImgStamp());
              img_get = true;

						}
					
						//reset
            if (!img_get) {
                getColorImgStamp(true);
            }

            if (imu_SubNumber > 0) {

								static int imu_count = 0;
								imu_count ++;
								std::cout << "imu_count:" << imu_count << std::endl;

								retCode = auxStream->GetFrame(imu_frame);

								if (retCode != InuDev::eOK)
								{
								    std::cout << "Failed to get imu_frame with error: " << std::hex << int(retCode) << " - " << std::string(retCode) << std::endl;
								}
								else
								{
		              publishIMU(imu_frame, pub_imu, getIMUStamp());
								}
            } else {
                getIMUStamp(nullptr, true);  
            }
        }
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

    image_transport::ImageTransport it_inuitive(nh);

    pub_left = it_inuitive.advertise(left_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_topic);
    pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_cam_info_topic);

    pub_right = it_inuitive.advertise(right_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_topic);
    pub_right_cam_info = nh.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_cam_info_topic);

    pub_web = it_inuitive.advertise(web_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << web_topic);
    pub_web_cam_info = nh.advertise<sensor_msgs::CameraInfo>(web_cam_info_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << web_cam_info_topic);

    pub_depth = it_inuitive.advertise(depth_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << depth_topic);

    pub_imu = nh.advertise<sensor_msgs::Imu>(imu_topic, 1);
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
		//auxstream
		if(auxStream != NULL){
		  InuDev::CInuError retCode = auxStream->Stop();
		  if (retCode != InuDev::eOK){
		      std::cout << "auxStream Stop error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		  }
		  std::cout << "auxStream frames acquisition stopped" << std::endl;
		  retCode = auxStream->Terminate();
		  if (retCode != InuDev::eOK) {
		      std::cout << "auxStream Terminate error: " << std::hex << int(retCode) << " - "  << std::string(retCode) << std::endl;
		  }
		  std::cout << "auxStream Stream was finalized" << std::endl;
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
