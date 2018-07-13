#include <deque>
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
#include "map_modify/ModifyMap.h"


namespace ns_map_modify{

using namespace __gnu_cxx; 
using namespace std;
using namespace cv;

bool MapModifyService(
    map_modify::ModifyMap::Request &req,     
    map_modify::ModifyMap::Response &res) {
    ROS_INFO("Start Modify Map...");
    
    for(int i = 0;i < req.map.data.size();i++)
        if(req.map.data[i] == -1)
            req.map.data[i] = 100;
   
    cv::Mat map(req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());
    vector<int8_t> map_data; 

    cv::Mat bin;
    cv::threshold(map,bin,req.threshold,255,cv::THRESH_BINARY_INV);
//template eliminate noise
#if 1
for(int l = 0;l < 5;l++){
    for(int i = 1;i < req.map.info.height-1;i++){
        for(int j = 1;j < req.map.info.width-1;j++){
            if(bin.at<unsigned char>(i,j) == 0){
                
                int count_value_255_v = 0;
                int count_value_255_h = 0;
                {
                     if(bin.at<unsigned char>(i-1,j-1) == 255)
                        count_value_255_h++;
                     if(bin.at<unsigned char>(i-1,j+1) == 255)
                        count_value_255_h++;
                     if(bin.at<unsigned char>(i+1,j-1) == 255)
                        count_value_255_h++;
                     if(bin.at<unsigned char>(i+1,j+1) == 255)
                        count_value_255_h++;
                }

                if(count_value_255_h >= 3){
                    bin.at<unsigned char>(i,j) = 255;
                    continue;
                }
                else{
                    if(bin.at<unsigned char>(i-1,j) == 255)
                        count_value_255_v++;
                    if(bin.at<unsigned char>(i+1,j) == 255)
                        count_value_255_v++;
                    if(bin.at<unsigned char>(i,j-1) == 255)
                        count_value_255_v++;
                    if(bin.at<unsigned char>(i,j+1) == 255)
                        count_value_255_v++;
                }
                if(count_value_255_v >= 3)
                    bin.at<unsigned char>(i,j) = 255;
            }
            else{
                
                int count_value_0_v = 0;
                int count_value_0_h = 0;
                {
                     if(bin.at<unsigned char>(i-1,j-1) == 0)
                        count_value_0_h++;
                     if(bin.at<unsigned char>(i-1,j+1) == 0)
                        count_value_0_h++;
                     if(bin.at<unsigned char>(i+1,j-1) == 0)
                        count_value_0_h++;
                     if(bin.at<unsigned char>(i+1,j+1) == 0)
                        count_value_0_h++;
                }

                if(count_value_0_h >= 3){
                    bin.at<unsigned char>(i,j) = 0;
                    continue;
                }
                else{
                    if(bin.at<unsigned char>(i-1,j) == 0)
                        count_value_0_v++;
                    if(bin.at<unsigned char>(i+1,j) == 0)
                        count_value_0_v++;
                    if(bin.at<unsigned char>(i,j-1) == 0)
                        count_value_0_v++;
                    if(bin.at<unsigned char>(i,j+1) == 0)
                        count_value_0_v++;
                }
                if(count_value_0_v >= 3)
                    bin.at<unsigned char>(i,j) = 0;

            }
        }
    }
}
#endif 
    for(int i = 0;i < req.map.info.height;i++){
        for(int j = 0;j < req.map.info.width;j++){
           char value = bin.at<char>(i,j);
           map_data.push_back(value); 
        }
     }

    res.map.info.height = req.map.info.height;
    res.map.info.width = req.map.info.width;
    res.map.info.resolution = req.map.info.resolution;
    res.map.data = map_data;

    res.map.header.frame_id = req.map.header.frame_id;
    res.map.info.origin.position.x = req.map.info.origin.position.x;
    res.map.info.origin.position.y = req.map.info.origin.position.y;

    return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_modify_node");

  ros::NodeHandle private_nh("~");

  ros::ServiceServer map_modify_srv = private_nh.advertiseService("/sweeper/map_modify_srv",ns_map_modify::MapModifyService);

  ROS_INFO(" Map modify service active.");

  ros::spin();

  return 0;
}
