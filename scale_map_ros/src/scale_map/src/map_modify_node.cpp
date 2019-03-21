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
#include "scale_map/ModifyMap.h"
#include "lsd.h"

#define step_length 3
//#define DEBUG_SHOW

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

/*void deleteNoisePoint(cv::Mat& src_,cv::Mat& dst_){
    int iterate_num = 1;
    int count_step;
    for(int l = 0;l < iterate_num;l++){
        count_step = 0;
        for(int i = 1;i < src_.rows-1;i++){
            for(int j = 1;j < src_.cols-1;j++){
                if(src_.at<unsigned char>(i,j) == 0){
                
                    int count_value_255_v = 0;
                    int count_value_255_h = 0;
                    {
                        if(src_.at<unsigned char>(i-1,j-1) == 255)
                            count_value_255_h++;
                        if(src_.at<unsigned char>(i-1,j+1) == 255)
                            count_value_255_h++;
                        if(src_.at<unsigned char>(i+1,j-1) == 255)
                            count_value_255_h++;
                        if(src_.at<unsigned char>(i+1,j+1) == 255)
                            count_value_255_h++;
                    }

                    if(count_value_255_h >= 3){
                        dst_.at<unsigned char>(i,j) = 255;
                        count_step ++;
                    }
                    else{
                        if(src_.at<unsigned char>(i-1,j) == 255)
                            count_value_255_v++;
                        if(src_.at<unsigned char>(i+1,j) == 255)
                            count_value_255_v++;
                        if(src_.at<unsigned char>(i,j-1) == 255)
                            count_value_255_v++;
                        if(src_.at<unsigned char>(i,j+1) == 255)
                            count_value_255_v++;


                        if(count_value_255_v >= 3){
                            dst_.at<unsigned char>(i,j) = 255;
                            count_step ++;
                        }
                    }
                }
                else{
                
                    int count_value_0_v = 0;
                    int count_value_0_h = 0;
                    {
                        if(src_.at<unsigned char>(i-1,j-1) == 0)
                            count_value_0_h++;
                        if(src_.at<unsigned char>(i-1,j+1) == 0)
                            count_value_0_h++;
                        if(src_.at<unsigned char>(i+1,j-1) == 0)
                            count_value_0_h++;
                        if(src_.at<unsigned char>(i+1,j+1) == 0)
                            count_value_0_h++;
                    }

                    if(count_value_0_h >= 3){
                        dst_.at<unsigned char>(i,j) = 0;
                        count_step ++;
                    }
                    else{
                        if(src_.at<unsigned char>(i-1,j) == 0)
                            count_value_0_v++;
                        if(src_.at<unsigned char>(i+1,j) == 0)
                            count_value_0_v++;
                        if(src_.at<unsigned char>(i,j-1) == 0)
                            count_value_0_v++;
                        if(src_.at<unsigned char>(i,j+1) == 0)
                            count_value_0_v++;

                        if(count_value_0_v >= 3){
                            dst_.at<unsigned char>(i,j) = 0;
                            count_step ++;
                        }

                    }
                }
            }
        }
        ROS_INFO("count_step:%d",count_step);
    }
}*/
#endif

/*void correctThinWalls(cv::Mat& src_){
    for(int v = 1;v < src_.rows;v++){
        for(int u = 1;u < src_.cols;u++){
            if(src_.at<uchar>(v-1,u-1)==255 && \
               src_.at<uchar>(v-1,u)==0 && \
               src_.at<uchar>(v,u-1)==0 && \
               src_.at<uchar>(v,u)==255)

               src_.at<uchar>(v,u)=0;
            else if(src_.at<uchar>(v-1,u-1)==0 && \
                    src_.at<uchar>(v-1,u)==255 && \
                    src_.at<uchar>(v,u-1)==255 && \
                    src_.at<uchar>(v,u)==0)
                    
                    src_.at<uchar>(v,u-1)=0;
        }
    }
}*/

bool MapModifyService(
    scale_map::ModifyMap::Request &req,     
    scale_map::ModifyMap::Response &res) {
    ROS_INFO("Start Modify Map...");
    
    for(int i = 0;i < req.map.data.size();i++)
        if(req.map.data[i] == -1)
            req.map.data[i] = 100;
   
    cv::Mat map(req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());
    vector<int8_t> map_data; 

    cv::Mat bin_,bin_re_,temp_re_,bin,temp_bin_;
    cv::threshold(map,bin_,req.threshold,255,cv::THRESH_BINARY_INV);

#ifdef DEBUG_SHOW    
    cv::imshow("bin",bin_);
#endif

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    //cv::dilate(bin_, bin, element, cv::Point(-1, -1), 3);
    
    //bin to construct in contour
    cv::erode(bin_,temp_bin_,element,cv::Point(-1,-1),1);
    cv::dilate(temp_bin_,bin,element,cv::Point(-1,-1),1);

    //bin_re_ to construct ex contour
    cv::erode(bin_,temp_re_,element,cv::Point(-1,-1),2);
    cv::dilate(temp_re_,bin_re_,element,cv::Point(-1,-1),2);

#ifdef DEBUG_SHOW    
    cv::imshow("bin_re_",bin_re_);
#endif

    //Ex rect
    std::vector<cv::Point> contour_rect;
    std::vector<cv::Point2i> vertices_point;
    vertices_point = makeOIP(bin_re_,step_length);
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

#ifdef DEBUG_SHOW
    cv::imshow("map_re",map_re);
#endif

    //In 
    /*std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point> > valid_internal_contours;

	cv::findContours(bin,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_NONE,cv::Point());

	ROS_INFO_STREAM("contours_size:" << contours.size());
	int external_contour_id;
	int max_external_contour = 0;

	for(int i = 0;i < contours.size();i++){
		if(contours[i].size() > max_external_contour){
			max_external_contour = contours[i].size();
			external_contour_id = i;
		}
	}
	ROS_INFO_STREAM("valid_external_contour_size:" << contours[external_contour_id].size());
	
    for(int i = 0;i < contours.size();i++){
        if(i != external_contour_id)
			if(contours[i].size() > req.internal_contour_threshold)
            {
				valid_internal_contours.push_back(contours[i]);
				ROS_INFO_STREAM("valid_internal_contour_size:" << contours[i].size());
			}
	}

	ROS_INFO_STREAM("valid_internal_contour_number:" << valid_internal_contours.size());

    cv::Point **polygonPointsIn = new cv::Point *[valid_internal_contours.size()];
    for(int i = 0;i < valid_internal_contours.size();i++){
        polygonPointsIn[i] = new cv::Point[valid_internal_contours[i].size()];
    }
    
    for(int i = 0;i < valid_internal_contours.size();i++){
        for(int j = 0;j < valid_internal_contours[i].size();j++){
            polygonPointsIn[i][j].x = valid_internal_contours[i][j].x;
            polygonPointsIn[i][j].y = valid_internal_contours[i][j].y;
        }
    }

    for(int i = 0;i < valid_internal_contours.size();i++){
        const cv::Point* ppt_obj[1] = { polygonPointsIn[i] };

        int npt_obj[] = { valid_internal_contours[i].size() };
        cv::polylines(map_re, ppt_obj, npt_obj, 1, 1, cv::Scalar::all(0), 1, 8, 0);

        cv::fillPoly(map_re, ppt_obj,npt_obj,1,cv::Scalar::all(0));
    }
    
    
    for(int i = 0;i < valid_internal_contours.size();i++){
        delete[] polygonPointsIn[i];
    }
    delete[] polygonPointsIn;*/

    //Ex con
    /*cv::Mat map_co(req.map.info.height, req.map.info.width, CV_8UC1,cv::Scalar::all(0));

    cv::Point **polygonPointsExCon = new cv::Point *[1];
    polygonPointsExCon[0] = new cv::Point[contours[external_contour_id].size()];
    
    for(int j = 0;j < contours[external_contour_id].size();j++){
        polygonPointsExCon[0][j].x = contours[external_contour_id][j].x;
        polygonPointsExCon[0][j].y = contours[external_contour_id][j].y;
    }

    const cv::Point* ppt_ex_con[1] = { polygonPointsExCon[0] };

    int npt_ex_con[] = { contours[external_contour_id].size() };
    cv::polylines(map_co, ppt_ex_con, npt_ex_con, 1, 1, cv::Scalar::all(255), 1, 8, 0);

    //cv::fillPoly(map_co, ppt_ex_con,npt_ex_con,1,cv::Scalar::all(0));
    
    delete[] polygonPointsExCon[0];
    delete[] polygonPointsExCon;*/

    //Ex con fit
    cv::Mat map_co_fit(req.map.info.height, req.map.info.width, CV_8UC1,cv::Scalar::all(0));
    
    std::vector<cv::Point> contour_poly;
    double epsilon_approx = 3;
    cv::approxPolyDP(cv::Mat(contour_rect), contour_poly,epsilon_approx,true);

    cv::Point **polygonPointsExConFit = new cv::Point *[1];
    polygonPointsExConFit[0] = new cv::Point[contour_poly.size()];
    
    for(int j = 0;j < contour_poly.size();j++){
        polygonPointsExConFit[0][j].x = contour_poly[j].x;
        polygonPointsExConFit[0][j].y = contour_poly[j].y;
    }

    const cv::Point* ppt_ex_con_fit[1] = { polygonPointsExConFit[0] };

    int npt_ex_con_fit[] = { contour_poly.size() };
    cv::polylines(map_co_fit, ppt_ex_con_fit, npt_ex_con_fit, 1, 1, cv::Scalar::all(255), 1, 8, 0);
    
    delete[] polygonPointsExConFit[0];
    delete[] polygonPointsExConFit;

#ifdef DEBUG_SHOW
    cv::imshow("map_co_fit",map_co_fit);
#endif

    cv::Rect rect = cv::boundingRect(contour_poly);
    ros::Time begin = ros::Time::now();
    for(int i = rect.tl().y;i < rect.br().y;i++){
        for(int j = rect.tl().x;j < rect.br().x;j++){
            double toNearestEdge = cv::pointPolygonTest(contour_poly, Point2f(j,i),true);
            //ROS_INFO_STREAM("toNearestEdge:" << toNearestEdge);
            if(std::fabs(toNearestEdge) > 9.0){
               map_re.at<unsigned char>(i,j) = bin.at<unsigned char>(i,j);
            }
        }
    }
    ros::Time end = ros::Time::now();

    ROS_INFO_STREAM("#######################time cost:" << (end-begin).toSec());

#ifdef DEBUG_SHOW
    cv::imshow("map_re_",map_re);
    cv::waitKey(0);
#endif

#if 0
    cv::Mat bin_temp;
    bin_step1_out.convertTo(bin_temp,CV_64FC1);

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
        ROS_INFO("v_point_size:%d",(int)v_point.size());
        v_point.clear();
    }

    ROS_INFO("v_point_size:%d",(int)v_point.size());
    for(int i = 0;i < v_point.size();i++){
        map.at<unsigned char>(v_point[i].y,v_point[i].x) = 100;
    }
    
    vector<cv::Point>().swap(v_point);
    free_ntuple_list(ntl);
#endif
    cv::Mat temp_map;
    cv::dilate(map_re,temp_map,element,cv::Point(-1,-1),1);
    cv::erode(temp_map,map_re,element,cv::Point(-1,-1),1);

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

  ROS_INFO(" Map modify service active.");

  ros::spin();

  return 0;
}
