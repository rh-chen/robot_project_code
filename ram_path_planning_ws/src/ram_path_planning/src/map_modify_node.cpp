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
#include "ram_path_planning/ModifyMap.h"
#include "ram_path_planning/lsd.h"


namespace ns_map_modify{

using namespace __gnu_cxx; 
using namespace std;
using namespace cv;

void BresenhamLine(cv::Point& start,cv::Point& end,vector<cv::Point>& v){
  int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;
  int cnt = 0;

  dx = abs(end.x-start.x); dy = abs(end.y-start.y);
  
  if (dy <= dx) {
    d = 2*dy - dx; incr1 = 2 * dy; incr2 = 2 * (dy - dx);
    if (start.x > end.x) {
      x = end.x; y = end.y;
      ydirflag = (-1);
      xend = start.x;
    } else {
      x = start.x; y = start.y;
      ydirflag = 1;
      xend = end.x;
    }
    v.push_back(cv::Point(x,y));
    if (((end.y - start.y) * ydirflag) > 0) {
      while (x < xend) {
	    x++;
	    if (d <0) {
	        d+=incr1;
	    } else {
	        y++; d+=incr2;
	    }
        v.push_back(cv::Point(x,y));
      }
    } else {
      while (x < xend) {
	    x++;
	    if (d <0) {
	        d+=incr1;
	    } else {
	        y--; d+=incr2;
	    }
        v.push_back(cv::Point(x,y));
      }
    }		
  } else {
    d = 2*dx - dy;
    incr1 = 2*dx; incr2 = 2 * (dx - dy);
    if (start.y > end.y) {
      y = end.y; x = end.x;
      yend = start.y;
      xdirflag = (-1);
    } else {
      y = start.y; x = start.x;
      yend = end.y;
      xdirflag = 1;
    }
    v.push_back(cv::Point(x,y));
    if (((end.x - start.x) * xdirflag) > 0) {
      while (y < yend) {
	    y++;
	    if (d <0) {
	        d+=incr1;
	    } else {
	        x++; d+=incr2;
	    }
        v.push_back(cv::Point(x,y));
      }
    } else {
      while (y < yend) {
	    y++;
	    if (d <0) {
	        d+=incr1;
	    } else {
	        x--; d+=incr2;
	    }
        v.push_back(cv::Point(x,y));
      }
    }
  }
}

bool MapModifyService(
    ram_path_planning::ModifyMap::Request &req,     
    ram_path_planning::ModifyMap::Response &res) {
    ROS_INFO("Start Modify Map...");
    
    for(int i = 0;i < req.map.data.size();i++)
        if(req.map.data[i] == -1)
            req.map.data[i] = 100;
   
    cv::Mat map(req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());
    vector<int8_t> map_data; 

    cv::Mat bin_step1,bin_step1_out;
    cv::threshold(map,bin_step1,req.threshold,255,cv::THRESH_BINARY_INV);

    int iterate_num;
//template eliminate noise
#if 1
    cv::Mat bin_temp;
    bin_step1.convertTo(bin_temp,CV_64FC1);

    //std::cout << __FILE__ << __LINE__ << std::endl;
    image_double image = new_image_double(bin_temp.cols,bin_temp.rows);
    image->data = bin_temp.ptr<double>(0);

    //std::cout << __FILE__ << __LINE__ << std::endl;
    ntuple_list ntl = lsd(image);

    //std::cout << __FILE__ << __LINE__ << std::endl;
    //cv::Mat lsd = cv::Mat::zeros(bin_temp.rows,bin_temp.cols,CV_8UC1);
    cv::Point pt1,pt2;

    //std::cout << __FILE__ << __LINE__ << std::endl;
    ROS_INFO("Lines Number:%d",ntl->size);
    vector<cv::Point> v_point;

    for(int j = 0;j < ntl->size;j++){
        cv::Point start,end;

        start.x = int(ntl->values[0 + j * ntl->dim]);
        start.y = int(ntl->values[1 + j * ntl->dim]);
        end.x = int(ntl->values[2 + j * ntl->dim]);
        end.y = int(ntl->values[3 + j * ntl->dim]);
        
        int delta_x = end.x-start.x;
        int delta_y = end.y-start.y;

        if(sqrt(delta_x*delta_x+delta_y*delta_y) < 6.0)
            continue;

        BresenhamLine(start,end,v_point);
    }

    ROS_INFO("v_point_size:%d",(int)v_point.size());
    for(int i = 0;i < v_point.size();i++){
        map.at<unsigned char>(v_point[i].y,v_point[i].x) = 100;
    }
    
    vector<cv::Point>().swap(v_point);
   //std::cout << __FILE__ << __LINE__ << std::endl;
    free_ntuple_list(ntl);
#endif
    //std::cout << __FILE__ << __LINE__ << std::endl;
    cv::Mat bin_step2;
    cv::threshold(map,bin_step2,req.threshold,255,cv::THRESH_BINARY_INV);
    
    cv::Mat bin_step2_out;
    bin_step2.copyTo(bin_step2_out);
#if 1
iterate_num = 9;
int count_step_2;
for(int l = 0;l < iterate_num;l++){
    count_step_2 = 0;
    for(int i = 1;i < req.map.info.height-1;i++){
        for(int j = 1;j < req.map.info.width-1;j++){
            if(bin_step2_out.at<unsigned char>(i,j) == 0){
                
                int count_value_255_v = 0;
                int count_value_255_h = 0;
                {
                     if(bin_step2_out.at<unsigned char>(i-1,j-1) == 255)
                        count_value_255_h++;
                     if(bin_step2_out.at<unsigned char>(i-1,j+1) == 255)
                        count_value_255_h++;
                     if(bin_step2_out.at<unsigned char>(i+1,j-1) == 255)
                        count_value_255_h++;
                     if(bin_step2_out.at<unsigned char>(i+1,j+1) == 255)
                        count_value_255_h++;
                }

                if(count_value_255_h > 3){
                    map.at<unsigned char>(i,j) = 0;
                    bin_step2_out.at<unsigned char>(i,j) = 255;
                    count_step_2 ++;
                }
                else{
                    if(bin_step2_out.at<unsigned char>(i-1,j) == 255)
                        count_value_255_v++;
                    if(bin_step2_out.at<unsigned char>(i+1,j) == 255)
                        count_value_255_v++;
                    if(bin_step2_out.at<unsigned char>(i,j-1) == 255)
                        count_value_255_v++;
                    if(bin_step2_out.at<unsigned char>(i,j+1) == 255)
                        count_value_255_v++;


                    if(count_value_255_v > 3){
                        map.at<unsigned char>(i,j) = 0;
                        bin_step2_out.at<unsigned char>(i,j) = 255;
                        count_step_2 ++;
                    }
                }
            }
            else{
                
                int count_value_0_v = 0;
                int count_value_0_h = 0;
                {
                     if(bin_step2_out.at<unsigned char>(i-1,j-1) == 0)
                        count_value_0_h++;
                     if(bin_step2_out.at<unsigned char>(i-1,j+1) == 0)
                        count_value_0_h++;
                     if(bin_step2_out.at<unsigned char>(i+1,j-1) == 0)
                        count_value_0_h++;
                     if(bin_step2_out.at<unsigned char>(i+1,j+1) == 0)
                        count_value_0_h++;
                }

                if(count_value_0_h > 3){
                    map.at<unsigned char>(i,j) = 100;
                    bin_step2_out.at<unsigned char>(i,j) = 0;
                    count_step_2 ++;
                }
                else{
                    if(bin_step2_out.at<unsigned char>(i-1,j) == 0)
                        count_value_0_v++;
                    if(bin_step2_out.at<unsigned char>(i+1,j) == 0)
                        count_value_0_v++;
                    if(bin_step2_out.at<unsigned char>(i,j-1) == 0)
                        count_value_0_v++;
                    if(bin_step2_out.at<unsigned char>(i,j+1) == 0)
                        count_value_0_v++;

                    if(count_value_0_v > 3){
                        map.at<unsigned char>(i,j) = 100;
                        bin_step2_out.at<unsigned char>(i,j) = 0;
                        count_step_2 ++;
                    }

                }
            }
        }
    }
    ROS_INFO("count_step2:%d",count_step_2);
}

cv::Mat bin_step3;
cv::threshold(map,bin_step3,req.threshold,255,cv::THRESH_BINARY_INV);
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::findContours(bin_step3,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,cv::Point());

int max_contour_point = 0;
for(int i = 0;i < contours.size();i++){
	if(contours[i].size() > max_contour_point){
		max_contour_point = contours[i].size();
	}
}
ROS_INFO("max_contour_poit:%d",max_contour_point);
ROS_INFO("contours_size:%d",contours.size());

if(contours.size() > 1){
	for(int i = 0;i < contours.size();i++){
		if(contours[i].size() < max_contour_point){
			double contour_center_x = 0;
			double contour_center_y = 0;
			for(int j = 0;j < contours[i].size();j++){
				contour_center_x += contours[i][j].x;
				contour_center_y += contours[i][j].y;
			}
			contour_center_x /= (contours[i].size()*1.0);
			contour_center_y /= (contours[i].size()*1.0);
			
			ROS_INFO("contour_center_x:%f",contour_center_x);
			ROS_INFO("contour_center_y:%f",contour_center_y);
			cv::floodFill(map,
						  cv::Point(std::floor(contour_center_x),std::floor(contour_center_y)),
						  cv::Scalar(100), 0, 0, 0, 8 | cv::FLOODFILL_FIXED_RANGE);
		}
	}
 }



cv::Mat bin_step4;
cv::threshold(map,bin_step4,req.threshold,255,cv::THRESH_BINARY_INV);
std::vector<std::vector<cv::Point> > contours_step4;
std::vector<cv::Vec4i> hierarchy_step4;
cv::findContours(bin_step4,contours_step4,hierarchy_step4,CV_RETR_TREE,CV_CHAIN_APPROX_NONE,cv::Point());

int limit_contour_point = 9;
for(int i = 0;i < contours_step4.size();i++){
	if(contours_step4[i].size() < limit_contour_point){
		cv::Rect boundRect = cv::boundingRect(cv::Mat(contours_step4[i]));
		for(int i = boundRect.tl().x;i < boundRect.br().x;i++){
			for(int j = boundRect.tl().y;j < boundRect.br().y;j++){
				map.at<unsigned char>(j,i) = 0;
			}
		}
	}
}
#endif
    for(int i = 0;i < req.map.info.height;i++){
        for(int j = 0;j < req.map.info.width;j++){
           char value = map.at<char>(i,j);
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

  ROS_INFO("Ready to map modify...");

  ros::spin();

  return 0;
}
