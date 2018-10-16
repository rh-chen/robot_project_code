/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.
*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "reconstruction3d_nodelet/util2d.h"
#include "reconstruction3d_nodelet/util3d.h"
#include "reconstruction3d_nodelet/MsgConversion.h" 

namespace reconstruction3d_nodelet
{

class PointCloudXYZRGB : public nodelet::Nodelet
{
public:
	PointCloudXYZRGB() :
		maxDepth_(0.0),
		minDepth_(0.0),
		voxelSize_(0.0),
		decimation_(1),
		noiseFilterRadius_(0.0),
		noiseFilterMinNeighbors_(5),
		approxSyncStereo_(0),
		exactSyncStereo_(0),
        isMilliMeter_(true),
        frame_id_depth_("stereo_depth_optical_frame"),
        frame_id_cloud_("stereo_cloud_optical_frame"),
        offset_t_(0),
        offset_b_(0)
	{}

	virtual ~PointCloudXYZRGB()
	{
		if(approxSyncStereo_)
			delete approxSyncStereo_;
		if(exactSyncStereo_)
			delete exactSyncStereo_;
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh  = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 10;
		bool approxSync = true;

		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("max_depth", maxDepth_, maxDepth_);
		pnh.param("min_depth", minDepth_, minDepth_);
		pnh.param("voxel_size", voxelSize_, voxelSize_);
		pnh.param("decimation", decimation_, decimation_);
		pnh.param("noise_filter_radius", noiseFilterRadius_, noiseFilterRadius_);
		pnh.param("noise_filter_min_neighbors", noiseFilterMinNeighbors_, noiseFilterMinNeighbors_);
		pnh.param("isMilliMeter", isMilliMeter_, isMilliMeter_);
		pnh.param("frame_id_cloud", frame_id_cloud_, frame_id_cloud_);
		pnh.param("frame_id_depth", frame_id_depth_, frame_id_depth_);
        pnh.param("offset_t", offset_t_, offset_t_);
        pnh.param("offset_b", offset_b_, offset_b_);

        NODELET_INFO("Approximate time sync = %s", approxSync?"true":"false");
		if(approxSync)
		{
			approxSyncStereo_ = new message_filters::Synchronizer<MyApproxSyncStereoPolicy>(MyApproxSyncStereoPolicy(queueSize), imageLeft_, imageRight_, cameraInfoLeft_, cameraInfoRight_);
			approxSyncStereo_->registerCallback(boost::bind(&PointCloudXYZRGB::stereoCallback, this, _1, _2, _3, _4));
		}
		else
		{
			exactSyncStereo_ = new message_filters::Synchronizer<MyExactSyncStereoPolicy>(MyExactSyncStereoPolicy(queueSize), imageLeft_, imageRight_, cameraInfoLeft_, cameraInfoRight_);
			exactSyncStereo_->registerCallback(boost::bind(&PointCloudXYZRGB::stereoCallback, this, _1, _2, _3, _4));
		}

		ros::NodeHandle left_nh(nh, "left");
		ros::NodeHandle right_nh(nh, "right");
		ros::NodeHandle left_pnh(pnh, "left");
		ros::NodeHandle right_pnh(pnh, "right");
		image_transport::ImageTransport left_it(left_nh);
		image_transport::ImageTransport right_it(right_nh);
		image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), left_pnh);
		image_transport::TransportHints hintsRight("raw", ros::TransportHints(), right_pnh);

		imageLeft_.subscribe(left_it, left_nh.resolveName("image"), 1, hintsLeft);
		imageRight_.subscribe(right_it, right_nh.resolveName("image"), 1, hintsRight);
		cameraInfoLeft_.subscribe(left_nh, "camera_info", 1);
		cameraInfoRight_.subscribe(right_nh, "camera_info", 1);

        cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

        image_transport::ImageTransport depth_it(nh);
        depthPub_ = depth_it.advertiseCamera("depth_ghc", 1, false);
	}

	void stereoCallback(const sensor_msgs::ImageConstPtr& imageLeft,
			const sensor_msgs::ImageConstPtr& imageRight,
			const sensor_msgs::CameraInfoConstPtr& camInfoLeft,
			const sensor_msgs::CameraInfoConstPtr& camInfoRight)
	{
		if(!(imageLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
				imageLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
				imageLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				imageLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
			!(imageRight->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
				imageRight->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
				imageRight->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				imageRight->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
		{
			NODELET_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (enc=%s)", imageLeft->encoding.c_str());
			return;
		}

		if(cloudPub_.getNumSubscribers() || depthPub_.getNumSubscribers())
		{
			ros::WallTime time = ros::WallTime::now();

			cv_bridge::CvImageConstPtr ptrLeftImage, ptrRightImage;
			if(imageLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
				imageLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
			{
				ptrLeftImage = cv_bridge::toCvShare(imageLeft, "mono8");
			}
			else
			{
				ptrLeftImage = cv_bridge::toCvShare(imageLeft, "bgr8");
			}
			ptrRightImage = cv_bridge::toCvShare(imageRight, "mono8");

            cv::Mat mat_left  = ptrLeftImage->image;
            cv::Mat mat_right = ptrRightImage->image;

            int offset_t = offset_t_;
            int offset_b = offset_b_;

            int roi_w = mat_left.cols;
            int roi_h = mat_right.rows - offset_t - offset_b;

            cv::Rect roi = cv::Rect(0, offset_t, roi_w, roi_h);

            CameraModel m_l = cameraModelFromROS(*camInfoLeft,  Transform::getIdentity());
            CameraModel m_r = cameraModelFromROS(*camInfoRight, Transform::getIdentity());
            CameraModel camera_model_l(m_l.fx(), m_l.fy(), m_l.cx(), roi_h/2.0, Transform::getIdentity(), m_l.Tx(), roi.size());
            CameraModel camera_model_r(m_r.fx(), m_r.fy(), m_r.cx(), roi_h/2.0, Transform::getIdentity(), m_r.Tx(), roi.size());
            StereoCameraModel stereo_camera_model = StereoCameraModel("ros", camera_model_l, camera_model_r, Transform());

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud;
			pclCloud = util3d::cloudFromStereoImages(cv::Mat(mat_left, roi), cv::Mat(mat_right, roi), stereo_camera_model, decimation_);

            cv::Mat frameDepth = cv::Mat(pclCloud->height, pclCloud->width, isMilliMeter_ ? CV_16UC1 : CV_32FC1);

            for(unsigned int h = 0; h < pclCloud->height; h++)
            {
                for(unsigned int w = 0; w < pclCloud->width; w++)
                {
			        pcl::PointXYZRGB &pt = pclCloud->at(h*pclCloud->width + w);

                    float depth = pt.z;

                    if(isMilliMeter_)
                    {
                        unsigned short depthMM = 0;
                        if(depth <= (float)USHRT_MAX)
                        {
                            depthMM = (unsigned short)depth;
                        }
                        frameDepth.at<unsigned short>(h,w) = depthMM;

                        pt.x /= 1000.0f;
                        pt.y /= 1000.0f;
                        pt.z /= 1000.0f;
                    }
                    else
                    {
                        frameDepth.at<float>(h,w) = depth;
                    }
                }
            }

            /*
            cv::Mat medianFilterResult;
            cv::medianBlur(frameDepth, medianFilterResult, 7);
            publishDepth(medianFilterResult, camInfoLeft);
            */


            //cv::Mat medianFilterResult;
           // cv::bilateralFilter(frameDepth, medianFilterResult, 25, 25 * 2, 25 / 2);
            publishDepth(frameDepth, camInfoLeft);





			publishCloud(pclCloud, imageLeft->header);

			NODELET_DEBUG("point_cloud_xyzrgb from stereo time = %f s", (ros::WallTime::now() - time).toSec());
		}
	}

    void publishDepth(cv::Mat &depth, const sensor_msgs::CameraInfoConstPtr &camInfo) {

        std::string encoding = "";

        if(depth.type() == CV_16UC1)
            encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        if(depth.type() == CV_32FC1)
            encoding = sensor_msgs::image_encodings::TYPE_32FC1;

        sensor_msgs::Image left_img_msg;
        std_msgs::Header header_left;
        header_left.frame_id = frame_id_depth_;
        header_left.stamp = ros::Time::now();
        cv_bridge::CvImage left_img_bridge;
        left_img_bridge = cv_bridge::CvImage(header_left, encoding, depth);
        left_img_bridge.toImageMsg(left_img_msg);

        sensor_msgs::Image left_data;
        left_data.header.frame_id = frame_id_depth_;
        left_data.height = depth.rows;
        left_data.width  = depth.cols ;
        left_data.encoding = encoding;
        left_data.is_bigendian = false;
        left_data.step = depth.step;
        left_data.data = left_img_msg.data;

        sensor_msgs::CameraInfo left_info;
        left_info = *camInfo;
        left_info.header = left_data.header;

        ros::Time stamp = ros::Time::now();

        depthPub_.publish(left_data, left_info, stamp);
    }

	void publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pclCloud, const std_msgs::Header & header)
	{
		if(pclCloud->size() && (minDepth_ != 0.0 || maxDepth_ > minDepth_))
			pclCloud = util3d::passThrough(pclCloud, "z", minDepth_, maxDepth_>minDepth_?maxDepth_:std::numeric_limits<float>::max(), false);

		if(pclCloud->size() && voxelSize_ > 0.0)
			pclCloud = util3d::voxelize(pclCloud, voxelSize_);

		// Do radius filtering after voxel filtering ( a lot faster)
		if(pclCloud->size() && noiseFilterRadius_ > 0.0 && noiseFilterMinNeighbors_ > 0)
		{
			if(voxelSize_ <= 0.0 && !(minDepth_ != 0.0 || maxDepth_ > minDepth_))
				pclCloud = util3d::removeNaNFromPointCloud(pclCloud);

			pcl::IndicesPtr indices = util3d::radiusFiltering(pclCloud, noiseFilterRadius_, noiseFilterMinNeighbors_);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud(*pclCloud, *indices, *tmp);
			pclCloud = tmp;
		}

		sensor_msgs::PointCloud2 rosCloud;
		pcl::toROSMsg(*pclCloud, rosCloud);
		rosCloud.header.stamp = header.stamp;
		//rosCloud.header.frame_id = header.frame_id;
		rosCloud.header.frame_id = frame_id_cloud_;

		cloudPub_.publish(rosCloud);
	}

private:

	double maxDepth_;
	double minDepth_;
	double voxelSize_;
	int decimation_;
	double noiseFilterRadius_;
	int noiseFilterMinNeighbors_;
    bool isMilliMeter_;
    int offset_t_;
    int offset_b_;

	ros::Publisher cloudPub_;
    image_transport::CameraPublisher depthPub_;

	image_transport::SubscriberFilter imageLeft_;
	image_transport::SubscriberFilter imageRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyApproxSyncStereoPolicy;
	message_filters::Synchronizer<MyApproxSyncStereoPolicy> * approxSyncStereo_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyExactSyncStereoPolicy;
	message_filters::Synchronizer<MyExactSyncStereoPolicy> * exactSyncStereo_;

    std::string frame_id_cloud_;
    std::string frame_id_depth_;
};

PLUGINLIB_EXPORT_CLASS(reconstruction3d_nodelet::PointCloudXYZRGB, nodelet::Nodelet);
}

