#include <cpp_uav.hpp>

// cpp standard libraries
#include <array>
#include <vector>

// roscpp
#include <ros/ros.h>

// geometry_msgs
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

// Service
#include "cpp_uav/Torres16.h"

#include "opencv2/opencv.hpp"

#define PI 3.1415926

using namespace cv;
using namespace std;

/*float fittingLine(vector<cv::Point>& point)
{
	float x_sum =0;
	float y_sum = 0;
	float xy_sum = 0;
	float xx_sum = 0;
	std::cout << "fitting_line_corner_size:" << point.size() << std::endl;
	for(int i = 0;i < point.size();i++)
	{
		x_sum += point.at(i).x;
		y_sum += point.at(i).y;
		xy_sum += (point.at(i).x*point.at(i).y);
		xx_sum += (point.at(i).x*point.at(i).x);
	}
	
	if(fabs(xx_sum-x_sum*x_sum/(point.size()*1.0)) < 0.01)
	{
		if((xy_sum-x_sum*y_sum/(point.size()*1.0))/(xx_sum-x_sum*x_sum/(point.size()*1.0)) < 0)
			return tan(-89.9/180.0*PI);
		else
			return tan(89.9/180.0*PI);
	}
	else
		return (xy_sum-x_sum*y_sum/(point.size()*1.0))/(xx_sum-x_sum*x_sum/(point.size()*1.0));
}
void cornerDetectionByAngle(cv::Mat& binary,vector<cv::Point>& corner)
{
	vector <vector<Point>> map_contours;  
  findContours(binary,map_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

	int c_i = 0;
	int contour_point_size = map_contours[c_i].size();
	std::cout << "contour_point_size:" << contour_point_size << std::endl;
	
	int num_point_fitting_back;
	int num_point_fitting_front;
	int front_i,back_i;
	int start_index = 5;
	float k_back,k_front;
  //std::cout << __FILE__ << __LINE__ << std::endl;
  for (int i = start_index;i < contour_point_size-start_index;i++){
		vector<cv::Point> point_front;
		vector<cv::Point> point_back;
		num_point_fitting_back = 5;
		num_point_fitting_front = 5;

		front_i = i;
		back_i = i;
		while(num_point_fitting_back > 0)
		{
			point_back.push_back(map_contours[c_i].at(back_i));
			back_i--;
			num_point_fitting_back--;
		}
		
		k_back = fittingLine(point_back);
		std::cout << "k_back:" << k_back << std::endl;
		while(num_point_fitting_front > 0)
		{
			point_front.push_back(map_contours[c_i].at(front_i));
			front_i++;
			num_point_fitting_front--;
		}

		k_front = fittingLine(point_front);
		std::cout << "k_front:" << k_front << std::endl;
		float angle = atan2((k_front-k_back),(1.0+k_back*k_front));
		
		std::cout << "by_angle:" << angle << std::endl;
		if(fabs(angle) > PI/3 && fabs(angle) < PI*2/3)
			corner.push_back(map_contours[c_i].at(i));
	}
	std::cout << "by_angle_corner_size:" << corner.size() << std::endl;
}*/
float getDistance(cv::Point& pa,cv::Point& pb)
{
	return sqrt((pa.x-pb.x)*(pa.x-pb.x)+(pa.y-pb.y)*(pa.y-pb.y));
}
void cornerDetection(cv::Mat& binary,vector<cv::Point>& corner)
{
	vector <vector<Point>> map_contours;  
  findContours(binary,map_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

	int c_i = 0;
	int point_internal = 10;
	int contour_point_size = map_contours[c_i].size();
	std::cout << "contour_point_size:" << contour_point_size << std::endl;

  //std::cout << __FILE__ << __LINE__ << std::endl;
  float fMax = -1;
  int   iMax = -1;
  bool  bStart = false;

  for (int i = 0;i < contour_point_size;i++){
      cv::Point pa = map_contours[c_i].at((i+contour_point_size-point_internal)%contour_point_size);
      cv::Point pb = map_contours[c_i].at((i+contour_point_size+point_internal)%contour_point_size);
      cv::Point pc = map_contours[c_i].at(i);
      
  		//std::cout << __FILE__ << __LINE__ << std::endl;
      float fA = getDistance(pa,pb);
      float fB = getDistance(pa,pc)+getDistance(pb,pc);
      float fAng = fA/fB;
      float fSharp = 1-fAng;
      if (fSharp > 0.15){
          bStart = true;
          if (fSharp > fMax){
              fMax = fSharp;
              iMax = i;
          }
      }else{
          if (bStart){
  						//std::cout << __FILE__ << __LINE__ << std::endl;
							corner.push_back(map_contours[c_i].at(iMax));
              iMax  = -1;
              fMax  = -1;
              bStart = false;
          }
      }
  }  

}

/*
*@brief map coordinate to world coordinate
*@param resolution origin_x origin_y mx my 
*return wx wy
*/
void MapToWorld(
    double resolution, double origin_x, double origin_y, unsigned int mx,
    unsigned int my, double *wx, double *wy) {
  *wx = origin_x + (mx + 0.5) * resolution;
  *wy = origin_y + (my + 0.5) * resolution;
}

/*
*@brief world coordinate to map coordinate
*@param resolution origin_x origin_y mx my 
*return wx wy
*/
bool WorldToMap(
    double resolution, double origin_x, double origin_y, unsigned int size_x,
    unsigned int size_y, double wx, double wy, int *mx, int *my) {
  if (wx < origin_x || wy < origin_y)
    return false;

  *mx = static_cast<int>((wx - origin_x) / resolution);
  *my = static_cast<int>((wy - origin_y) / resolution);

  if (*mx < size_x && *my < size_y)
    return true;

  return false;
}
/**
 * @brief Generate vector of polygon from PointVector
 * @param polygon
 * @return std::vector<geometry_msgs::Polygon> Vector of subpolygons assumed to be passed to ROS msg
 */
std::vector<geometry_msgs::Polygon> generatePolygonVector(const PointVector& polygon)
{
  // convert PointVector (a.k.a. std::vector<geometry_msgs::Point>) to geometry_msgs::Polygon
  // so that polygon is visualized on the window
  geometry_msgs::Polygon poly;
  for (const auto& vertex : polygon)
  {
    geometry_msgs::Point32 p;
    p.x = vertex.x;
    p.y = vertex.y;
    poly.points.push_back(p);
  }

  std::vector<geometry_msgs::Polygon> polygons = { poly };

  return polygons;
}

/**
 * @brief Generate vector of polygon from std::vector<PointVector>
 * @param polygon
 * @return std::vector<geometry_msgs::Polygon> Vector of subpolygons assumed to be passed to ROS msg
 */
std::vector<geometry_msgs::Polygon> generatePolygonVector(const std::vector<PointVector>& subPolygons)
{
  std::vector<geometry_msgs::Polygon> subPolygonsRet;

  // convert PointVector (a.k.a. std::vector<geometry_msgs::Point>) to geometry_msgs::Polygon
  // so that polygon is visualized on the window
  for (const auto& subPolygon : subPolygons)
  {
    geometry_msgs::Polygon poly;
    for (const auto& vertex : subPolygon)
    {
      geometry_msgs::Point32 pt;
      pt.x = vertex.x;
      pt.y = vertex.y;
      poly.points.push_back(pt);
    }
    subPolygonsRet.push_back(poly);
  }

  return subPolygonsRet;
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

#endif
/**
 * @brief Plans coverage path
 * @param req[in] Contains values neccesary to plan a path
 * @param res[out] Contains resulting path
 * @return bool
 * @details For details of this service, cf. srv/Torres16.srv
 */
bool plan(cpp_uav::Torres16::Request& req, cpp_uav::Torres16::Response& res)
{
	if (req.erosion_radius < 0) {
    ROS_ERROR("erosion_radius < 0");
    return false;
  }
  if (req.robot_radius < 0) {
    ROS_ERROR("robot_radius < 0");
    return false;
  }
  if (req.occupancy_threshold < 0 || req.occupancy_threshold > 100) {
    ROS_ERROR("occupancy_threshold out of range 0~100");
    return false;
  }
  if (req.map.info.resolution < 0) {
    ROS_ERROR("invalid map: resolution < 0");
    return false;
  }
  if (req.map.info.width * req.map.info.height < 1) {
    ROS_ERROR("invalid map: width * height < 1");
    return false;
  }
  if (req.map.info.width * req.map.info.height != req.map.data.size()) {
    ROS_ERROR("invalid map: width * height != data size");
    return false;
  }

  std::cout << __FILE__ << __LINE__ << std::endl;
  //  build start and goal
  cv::Point cv_start_point;
  if (false == WorldToMap(
                   req.map.info.resolution, req.map.info.origin.position.x,
                   req.map.info.origin.position.y, req.map.info.width,
                   req.map.info.height, req.start.x,
                   req.start.y, &cv_start_point.x, &cv_start_point.y)) {
    ROS_ERROR("Invalid start. Out of map.");
    return false;
  }

  std::cout << __FILE__ << __LINE__ << std::endl;
  //  build map
  for (int i = 0; i < req.map.data.size(); ++i) {
    if (-1 == req.map.data[i]) {  //  replace unknown with 100
      req.map.data[i] = 100;
    }
  }
  std::cout << __FILE__ << __LINE__ << std::endl;
  cv::Mat map(
      req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());

	/*save image*/
	//imwrite("map.jpg",map);

  std::cout << __FILE__ << __LINE__ << std::endl;
  //  binarization
  cv::Mat bin;
  cv::threshold(
      map, bin, req.occupancy_threshold, 255, cv::THRESH_BINARY_INV);

     PointVector polygon;

    //rectilinear polygon
 	double delta_point = 8;
	std::vector<cv::Point2i> vertices_point;
	vertices_point = makeOIP(bin,delta_point);

	for(int j = 0;j < vertices_point.size();j++){
		double point_x = vertices_point[j].x*req.map.info.resolution + req.map.info.origin.position.x;
		double point_y = vertices_point[j].y*req.map.info.resolution + req.map.info.origin.position.y;
        //double point_x = vertices_point[j].x;
        //double point_y = vertices_point[j].y;

		geometry_msgs::Point point_;
        
        point_.x = point_x;
        point_.y = point_y;
        point_.z = 0;
        ROS_INFO_STREAM("point_:" << point_);
        polygon.push_back(point_);
	}

    std::cout << "polygon_point_size:" << polygon.size() << std::endl;

    std::vector<PointVector> subPolygons = decomposePolygon(polygon);

    res.subpolygons = generatePolygonVector(subPolygons);
    
    //res.subpolygons = generatePolygonVector(polygon);
    return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zig_zag_cpp_service_node");
  ros::NodeHandle nh;

  ros::ServiceServer planner = nh.advertiseService("cpp_service", plan);
  ROS_INFO("Ready to plan.");

  ros::spin();

  return 0;
}
