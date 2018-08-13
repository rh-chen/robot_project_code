/*ros*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/stereo_camera_model.h> 
#include <image_transport/image_transport.h> 
#include <image_transport/subscriber_filter.h> 
#include <message_filters/sync_policies/approximate_time.h> 
#include <message_filters/sync_policies/exact_time.h> 
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdio.h"
#include "stdlib.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class mynt_publish_camera_info{
    public:
    		image_transport::CameraPublisher Pub;
				image_transport::ImageTransport it;
        ros::NodeHandle nh;
        //ros::Publisher image_pub;
        //ros::Publisher caminfo_pub;
        ros::Subscriber image_sub;

        mynt_publish_camera_info(ros::NodeHandle n):nh(n),it(n){
            image_sub = nh.subscribe("/mynteye/depth",100,&mynt_publish_camera_info::imageCallBack,this);
            //image_pub = nh.advertise<sensor_msgs::Image>("/mynteye/depth/image_raw",100);
            //caminfo_pub = nh.advertise<sensor_msgs::CameraInfo>("/mynteye/depth/camera_info",100);
						Pub = it.advertiseCamera("/mynteye/depth/image_raw", 1, false);
        }

        void imageCallBack(const sensor_msgs::Image &msg);

};


void mynt_publish_camera_info::imageCallBack(const sensor_msgs::Image &msg){
    
    sensor_msgs::Image image_msg = msg;

    sensor_msgs::CameraInfo camerainfo_msg;
	camerainfo_msg.height = msg.height;
	camerainfo_msg.width = msg.width;
    camerainfo_msg.header.stamp = msg.header.stamp;
	
	std::vector<double> D_;
	D_.push_back(-0.420841);
	D_.push_back(0.221797);
	D_.push_back(-0.000350528);
	D_.push_back(0.000331603);
	D_.push_back(-0.105724);
	
	camerainfo_msg.distortion_model = "plumb_bob";
	camerainfo_msg.D = D_;

	camerainfo_msg.K[0] = 1614.26;
	camerainfo_msg.K[1] = 0;
	camerainfo_msg.K[2] = 608.569;
	camerainfo_msg.K[3] = 0;
	camerainfo_msg.K[4] = 1614.23;
	camerainfo_msg.K[5] = 371.982;
	camerainfo_msg.K[6] = 0;
	camerainfo_msg.K[7] = 0;
	camerainfo_msg.K[8] = 1;

	camerainfo_msg.R[0] = 0.999999;
	camerainfo_msg.R[1] = -0.00105316;
	camerainfo_msg.R[2] = -0.00126153;
	camerainfo_msg.R[3] = 0.00105288;
	camerainfo_msg.R[4] = 0.999999;
	camerainfo_msg.R[5] = -0.000228497;
	camerainfo_msg.R[6] = 0.00126176;
	camerainfo_msg.R[7] = 0.000227169;
	camerainfo_msg.R[8] = 0.999999;

	camerainfo_msg.P[0] = 1620;
	camerainfo_msg.P[1] = 0;
	camerainfo_msg.P[2] = 629.864;
	camerainfo_msg.P[3] = 0;

	camerainfo_msg.P[4] = 0;
	camerainfo_msg.P[5] = 1620;
	camerainfo_msg.P[6] = 372.997;
	camerainfo_msg.P[7] = 0;

	camerainfo_msg.P[8] = 0;
	camerainfo_msg.P[9] = 0;
	camerainfo_msg.P[10] = 1;
	camerainfo_msg.P[11] = 0;

	Pub.publish(image_msg,camerainfo_msg,msg.header.stamp);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mynt_d_camera_info_node");

    ros::NodeHandle nh_;
    mynt_publish_camera_info mynt(nh_);

    ros::spin();

    return 0;
}
