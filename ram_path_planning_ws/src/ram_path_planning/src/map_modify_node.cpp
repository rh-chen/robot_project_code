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

#define step_length 1

//#define SHOW_DEBUG

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

#if 1
cv::Point2i getTopLeftPoint(cv::Mat& image) {
  int nRows = image.rows;
  int nCols = image.cols;

  if (image.isContinuous()) {
    nCols *= nRows;
    nRows = 1;
  }

  unsigned char* p;

  for (int i = 0; i < nRows; ++i) {
    p = image.ptr(i);
    for (int j = 0; j < nCols; ++j) {
      if (p[j] == 255) {
        if (image.isContinuous()) {
          nCols = image.cols;
          cv::Point2i P(j % nCols, j / nCols);
          return P;
        } else {
          cv::Point2i P(j, i);
          return P;
        }
      }
    }
  }

  cv::Point2i P(-1, -1);
  return P;
}

cv::Point2i getStartPoint(cv::Mat& img, cv::Point2i p, int gsize) {
  int qx, qy;
  qx = (ceil(float(p.x) / gsize) - 1) * gsize;
  qy = (ceil(float(p.y) / gsize) - 1) * gsize;
  
  if(qx < 0)
    qx = 0;

  if(qy < 0)
    qy = 0;

  cv::Point2i q(qx, qy);
  return q;
}

bool objectInUGB(cv::Mat& img, cv::Point2i q, int ugb, int gsize) {
  cv::Point2i pt;
  switch (ugb) {
    case 1:
      pt.x = q.x;
      pt.y = q.y - gsize;
      break;
    case 2:
      pt.x = q.x - gsize;
      pt.y = q.y - gsize;
      break;
    case 3:
      pt.x = q.x - gsize;
      pt.y = q.y;
      break;
    case 4:
      pt.x = q.x;
      pt.y = q.y;
      break;
    default:
      break;
  }

  if (pt.x < 0 || pt.y < 0 || pt.x >= img.cols || pt.y >= img.rows) {
    return false;
  }

  unsigned char* p;
  for (int i = pt.y; i <= pt.y + gsize; i++) {
    p = img.ptr(i);
    for (int j = pt.x; j <= pt.x + gsize; ++j) {
      if (p[j] == 255) {
        return true;
      }
    }
  }
  return false;
}

int getPointType(cv::Mat& img, cv::Point2i q, int gsize) {
  int m = 0, r = 0, t = 10;
  for (int k = 1; k < 5; k++) {
    if (objectInUGB(img, q, k, gsize)) {
      m++;
      r += k;
    }
  }
  if (m == 2 && (r == 4 || r == 6)) {
    t = -2;
  } else if (m == 0 || m == 4) {
    t = 0;
  } else {
    t = 2 - m;
  }
  return t;
}

cv::Point2i getNextPoint(cv::Point2i currentpoint, int d, int gsize) {
  cv::Point2i nextpoint;
  switch (d) {
    case 0:
      nextpoint.x = currentpoint.x + gsize;
      nextpoint.y = currentpoint.y;
      break;
    case 1:
      nextpoint.x = currentpoint.x;
      nextpoint.y = currentpoint.y - gsize;
      break;
    case 2:
      nextpoint.x = currentpoint.x - gsize;
      nextpoint.y = currentpoint.y;
      break;
    case 3:
      nextpoint.x = currentpoint.x;
      nextpoint.y = currentpoint.y + gsize;
      break;
  }
  return nextpoint;
}

std::vector<cv::Point2i> makeOIP(cv::Mat& img, int gsize) {
  std::vector<cv::Point2i> vertices;

  cv::Point2i topleftpoint = getTopLeftPoint(img);
  //ROS_INFO("topleftpoint:%d,%d",topleftpoint.x,topleftpoint.y);
  cv::Point2i startpoint = getStartPoint(img, topleftpoint, gsize);
  //ROS_INFO("startpoint:%d,%d",startpoint.x,startpoint.y);
  cv::Point2i q = startpoint;
  int type = getPointType(img, q, gsize);

  int d = (2 + type) % 4;
  do {
    if (type == 1 || type == -1) {
      vertices.push_back(q);
    }
    q = getNextPoint(q, d, gsize);
    type = getPointType(img, q, gsize);
    if (type == -2) {
      type = -1;
    }
    d = (d + type) % 4;
    if (d < 0) {
      d += 4;
    }
  } while (q != startpoint);

  return vertices;
}

#endif

bool MapModifyService(
    ram_path_planning::ModifyMap::Request &req,     
    ram_path_planning::ModifyMap::Response &res) {
    ROS_INFO("Start Modify Map...");
    
    for(int i = 0;i < req.map.data.size();i++)
        if(req.map.data[i] == -1)
            req.map.data[i] = 100;
   
    cv::Mat map(req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());
    vector<int8_t> map_data; 
    cv::Mat bin_step,bin_step_out,bin_blurred_temp;
    cv::threshold(map,bin_step,req.threshold,255,cv::THRESH_BINARY_INV);
#ifdef SHOW_DEBUG
    cv::imshow("bin_step",bin_step);
#endif

    cv::Mat blurred_bin(map.size(), CV_8UC1);
    //cv::blur(bin_step,blurred_bin,cv::Size(5,5));
    cv::GaussianBlur(bin_step,blurred_bin,cv::Size(3,3),0,0);
#ifdef SHOW_DEBUG    
    cv::imshow("blurred_bin",blurred_bin);
#endif
    
    cv::Mat canny_bin(map.size(), CV_8UC1);
    cv::Canny(blurred_bin, canny_bin, 60, 255, 3);

#ifdef SHOW_DEBUG
    imshow("canny_bin", canny_bin);
#endif
    cv::Mat contours_bin(map.rows,map.cols,CV_8UC1,cv::Scalar(255));
    std::vector<std::vector<cv::Point> > contours_canny;
    std::vector<std::vector<cv::Point> > contours_ext;
    std::vector<cv::Vec4i> hierarchy_canny;
    std::vector<cv::Vec4i> hierarchy_ext;

    cv::findContours(canny_bin, contours_canny, hierarchy_canny, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,0));
    cv::drawContours(contours_bin, contours_canny, -1, cv::Scalar(0), 2, 8, hierarchy_canny, 1, cv::Point());
#ifdef SHOW_DEBUG
    imshow("contours_bin",contours_bin);
#endif

    cv::Point start_point((req.start_position_x-req.map.info.origin.position.x)/req.map.info.resolution,\
                          (req.start_position_y-req.map.info.origin.position.y)/req.map.info.resolution);
    cv::Rect roi;
    uchar seedColor = 128;
    cv::Mat mask_img = contours_bin.clone();
    cv::Mat masked_img;

    cv::floodFill(mask_img,start_point,cv::Scalar(128),&roi,cv::Scalar(3),cv::Scalar(3));
#ifdef SHOW_DEBUG
    imshow("mask_img",mask_img);
#endif
    masked_img = (mask_img == seedColor);
   
#ifdef SHOW_DEBUG
    imshow("masked_img",masked_img);
#endif
#if 0
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
#if 0
    int iterate_num = 7;
    int count_step;
    for(int l = 0;l < iterate_num;l++){
        count_step = 0;
        for(int i = 1;i < req.map.info.height-1;i++){
            for(int j = 1;j < req.map.info.width-1;j++){
                if(bin_step.at<unsigned char>(i,j) == 0){
                    
                    int count_value_255_v = 0;
                    int count_value_255_h = 0;
                    {
                         if(masked_img.at<unsigned char>(i-1,j-1) == 255)
                            count_value_255_h++;
                         if(masked_img.at<unsigned char>(i-1,j+1) == 255)
                            count_value_255_h++;
                         if(masked_img.at<unsigned char>(i+1,j-1) == 255)
                            count_value_255_h++;
                         if(masked_img.at<unsigned char>(i+1,j+1) == 255)
                            count_value_255_h++;
                    }

                    if(count_value_255_h > 3){
                        masked_img.at<unsigned char>(i,j) = 255;
                        count_step ++;
                    }
                    else{
                        if(masked_img.at<unsigned char>(i-1,j) == 255)
                            count_value_255_v++;
                        if(masked_img.at<unsigned char>(i+1,j) == 255)
                            count_value_255_v++;
                        if(masked_img.at<unsigned char>(i,j-1) == 255)
                            count_value_255_v++;
                        if(masked_img.at<unsigned char>(i,j+1) == 255)
                            count_value_255_v++;


                        if(count_value_255_v > 3){
                            masked_img.at<unsigned char>(i,j) = 255;
                            count_step ++;
                        }
                    }
                }
                else{
                    
                    int count_value_0_v = 0;
                    int count_value_0_h = 0;
                    {
                         if(masked_img.at<unsigned char>(i-1,j-1) == 0)
                            count_value_0_h++;
                         if(masked_img.at<unsigned char>(i-1,j+1) == 0)
                            count_value_0_h++;
                         if(masked_img.at<unsigned char>(i+1,j-1) == 0)
                            count_value_0_h++;
                         if(masked_img.at<unsigned char>(i+1,j+1) == 0)
                            count_value_0_h++;
                    }

                    if(count_value_0_h > 3){
                        masked_img.at<unsigned char>(i,j) = 0;
                        count_step ++;
                    }
                    else{
                        if(masked_img.at<unsigned char>(i-1,j) == 0)
                            count_value_0_v++;
                        if(masked_img.at<unsigned char>(i+1,j) == 0)
                            count_value_0_v++;
                        if(masked_img.at<unsigned char>(i,j-1) == 0)
                            count_value_0_v++;
                        if(masked_img.at<unsigned char>(i,j+1) == 0)
                            count_value_0_v++;

                        if(count_value_0_v > 3){
                            masked_img.at<unsigned char>(i,j) = 0;
                            count_step ++;
                        }

                    }
                }
            }
        }
    }
    #endif


    cv::Mat temp_out;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::dilate(masked_img,temp_out,element, cv::Point(-1,-1),1);
    //temp_out.copyTo(masked_img);
    cv::erode(temp_out,masked_img,element, cv::Point(-1,-1),1);

#ifdef SHOW_DEBUG
    imshow("masked_img_step_1",masked_img);
#endif

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(masked_img,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,cv::Point());

    int max_contour_point = 0;
    for(int i = 0;i < contours.size();i++){
        if(contours[i].size() > max_contour_point){
            max_contour_point = contours[i].size();
        }
    }
    ROS_INFO_STREAM("max_ext_contour_point:" << max_contour_point);
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
                
                cv::floodFill(masked_img,
                              cv::Point(std::floor(contour_center_x),std::floor(contour_center_y)),
                              cv::Scalar(0), 0, 0, 0, 8 | cv::FLOODFILL_FIXED_RANGE);
            }
        }
     }

#ifdef SHOW_DEBUG
    imshow("masked_img_step_2",masked_img);
#endif


    std::vector<std::vector<cv::Point> > contours_step;
    std::vector<cv::Vec4i> hierarchy_step;
    cv::findContours(masked_img,contours_step,hierarchy_step,CV_RETR_TREE,CV_CHAIN_APPROX_NONE,cv::Point());

    int limit_contour_point = 16;
    for(int i = 0;i < contours_step.size();i++){
        if(contours_step[i].size() < limit_contour_point){
            cv::Rect boundRect = cv::boundingRect(cv::Mat(contours_step[i]));
            for(int i = boundRect.tl().x;i < boundRect.br().x;i++){
                for(int j = boundRect.tl().y;j < boundRect.br().y;j++){
                    masked_img.at<unsigned char>(j,i) = 255;
                }
            }
        }
    }

#ifdef SHOW_DEBUG
    imshow("masked_img_step_3",masked_img);
#endif
    std::vector<cv::Point> contour_rect;
    std::vector<cv::Point2i> vertices_point;
    vertices_point = makeOIP(masked_img,step_length);
    ROS_INFO_STREAM("vertices_point:" << vertices_point.size());    
    cv::Mat map_re(req.map.info.height, req.map.info.width, CV_8UC1,cv::Scalar::all(0));

    cv::Point **polygonPointsEx = new cv::Point *[1];
    polygonPointsEx[0] = new cv::Point[vertices_point.size()];
    
    for(int i = 0;i < vertices_point.size();i++){
        polygonPointsEx[0][i].x = vertices_point[i].x;
        polygonPointsEx[0][i].y = vertices_point[i].y;

        contour_rect.push_back(cv::Point(vertices_point[i].x,vertices_point[i].y));
    }

    const cv::Point* ppt[1] = { polygonPointsEx[0] };

    int npt[] = { vertices_point.size() };
    cv::polylines(map_re, ppt, npt, 1, 1, cv::Scalar::all(255), 1, 8, 0);

    cv::fillPoly(map_re, ppt,npt,1,cv::Scalar::all(255));
    delete[] polygonPointsEx[0];
    delete[] polygonPointsEx;

#ifdef SHOW_DEBUG
    imshow("map_re",map_re);
    cv::waitKey(0);
#endif
    /*std::vector<cv::Point2f> goodCorners;
    int max_corners = 20;
    double quality_level = 0.01;
    double min_distance = 3.0;
    int block_size = 3;
    bool use_harris = false;
    double k = 0.04;

    cv::goodFeaturesToTrack(map_re,goodCorners,max_corners,quality_level,min_distance,cv::Mat(),block_size,use_harris,k);
    ROS_INFO_STREAM("goodCorners:" << goodCorners.size());
    
    for(int i = 0;i < goodCorners.size();i++){
        cv::circle(map_re,goodCorners[i],1,cv::Scalar::all(255),2,8,0);
    }*/

    /*cv::Rect rect = cv::boundingRect(vertices_point);
    ros::Time begin = ros::Time::now();
    for(int i = rect.tl().y;i < rect.br().y;i++){
        for(int j = rect.tl().x;j < rect.br().x;j++){
            double toNearestEdge = cv::pointPolygonTest(vertices_point, Point2f(j,i),true);
            //ROS_INFO_STREAM("toNearestEdge:" << toNearestEdge);
            if(std::fabs(toNearestEdge) > 9.0 && (int)toNearestEdge > 0){
               //ROS_INFO_STREAM("toNearestEdge:" << toNearestEdge);
               map_re.at<unsigned char>(i,j) = bin_step_out.at<unsigned char>(i,j);
            }
        }
    }*/

    for(int i = 0;i < req.map.info.height;i++){
        for(int j = 0;j < req.map.info.width;j++){
           char value = map_re.at<char>(i,j);

           if(value == 0)
                map_data.push_back(100);
           else
                map_data.push_back(0);
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
