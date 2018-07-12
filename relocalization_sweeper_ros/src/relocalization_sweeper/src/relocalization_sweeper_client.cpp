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

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>

#include <stdio.h>
#include <math.h>

#include <hash_set>
#include <set>
#include <map>
#include <algorithm> 

#include <tf/tf.h>
#include "relocalization_sweeper/GetRobotPose.h"


int main(int argc,char **argv){
    
    ros::init(argc, argv, "relocalization_robot_client");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<relocalization_sweeper::GetRobotPose>("/sweeper/relocalization_robot_srv");
    relocalization_sweeper::GetRobotPose srv;
    
    srv.request.img_data = sensor_msgs::Image();
    
    if(client.call(srv)){
        ROS_INFO("Relocalization Robot Success...");
    }

    return 0; 
}
