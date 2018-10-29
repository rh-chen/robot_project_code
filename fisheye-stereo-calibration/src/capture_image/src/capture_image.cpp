#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

namespace capture_image
{

    class CaptureImage : public nodelet::Nodelet
    {
        public:
            CaptureImage() : approxSyncStereo_(0), exactSyncStereo_(0) {}

            virtual ~CaptureImage()
            {
                if(approxSyncStereo_)
                    delete approxSyncStereo_;
                if(exactSyncStereo_)
                    delete exactSyncStereo_;
            }

        private:
            virtual void onInit()
            {
                ros::NodeHandle & nh = getNodeHandle();
                ros::NodeHandle & pnh = getPrivateNodeHandle();

                int queueSize = 10;
                i = 0;
                bool approxSync = true;
                pnh.param("approx_sync", approxSync, approxSync);

                if(approxSync)
                {

                    approxSyncStereo_ = new message_filters::Synchronizer<MyApproxSyncStereoPolicy>(MyApproxSyncStereoPolicy(queueSize), imageLeft_, imageRight_, cameraInfoLeft_, cameraInfoRight_);
                    approxSyncStereo_->registerCallback(boost::bind(&CaptureImage::stereoCallback, this, _1, _2, _3, _4));
                }
                else
                {
                    exactSyncStereo_ = new message_filters::Synchronizer<MyExactSyncStereoPolicy>(MyExactSyncStereoPolicy(queueSize), imageLeft_, imageRight_, cameraInfoLeft_, cameraInfoRight_);
                    exactSyncStereo_->registerCallback(boost::bind(&CaptureImage::stereoCallback, this, _1, _2, _3, _4));
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


                cv_bridge::CvImageConstPtr ptrLeftImage, ptrRightImage;

                ptrLeftImage = cv_bridge::toCvShare(imageLeft, "mono8");

                ptrRightImage = cv_bridge::toCvShare(imageRight, "mono8");

                cv::imshow("img_left", ptrLeftImage->image);
                cv::imshow("img_right", ptrRightImage->image);

                char key =    cv::waitKey(100);
                std::string left = "/home/wzm/fisheye-stereo-calibration/imgs/left";
                std::string right = "/home/wzm/fisheye-stereo-calibration/imgs/right";
                std::ostringstream oss_left;
                oss_left << left << i << ".jpg";
                std::ostringstream oss_right;
                oss_right << right << i << ".jpg";
                if(key == 'w')
                {
                    cv::imwrite(oss_left.str(),ptrLeftImage->image);
                    cv::imwrite(oss_right.str(),ptrRightImage->image);
                    std::cout << " "<<std::endl;
                    i++;
                }

            }

        private:

            int i ;

            image_transport::SubscriberFilter imageLeft_;
            image_transport::SubscriberFilter imageRight_;
            message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
            message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;

            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyApproxSyncStereoPolicy;
            message_filters::Synchronizer<MyApproxSyncStereoPolicy> * approxSyncStereo_;

            typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MyExactSyncStereoPolicy;
            message_filters::Synchronizer<MyExactSyncStereoPolicy> * exactSyncStereo_;

    };

    PLUGINLIB_EXPORT_CLASS(capture_image::CaptureImage, nodelet::Nodelet);
}

