/**
 * @file torres_etal_2016.cpp
 * @brief Coverage path planner based on M. Torres et al, 2016
 * @author Takaki Ueno
 */

/*
 * Copyright (c) 2017 Takaki Ueno
 * Released under the MIT license
 */

// header
#include <torres_etal_2016.hpp>

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

using namespace cv;
using namespace std;

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

/**
 * @brief Plans coverage path
 * @param req[in] Contains values neccesary to plan a path
 * @param res[out] Contains resulting path
 * @return bool
 * @details For details of this service, cf. srv/Torres16.srv
 */
bool plan(cpp_uav::Torres16::Request& req, cpp_uav::Torres16::Response& res)
{
  // see torres et al. 2016 for the flow of this algorithm
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

  //std::cout << __FILE__ << __LINE__ << std::endl;
  //  build map
  for (int i = 0; i < req.map.data.size(); ++i) {
    if (-1 == req.map.data[i]) {  //  replace unknown with 100
      req.map.data[i] = 100;
    }
  }
  //std::cout << __FILE__ << __LINE__ << std::endl;
  cv::Mat map(
      req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());

	/*save image*/
	imwrite("map.jpg",map);

  //std::cout << __FILE__ << __LINE__ << std::endl;
  //  binarization
  cv::Mat binarization;
  cv::threshold(
      map, binarization, req.occupancy_threshold, 255, cv::THRESH_BINARY_INV);

	/*save image*/
	imwrite("map_binary.jpg",binarization);

  //  erosion
  cv::Mat erosion, element;
  element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::erode(
      binarization, erosion, element, cv::Point(-1, -1),
      (req.erosion_radius + req.map.info.resolution - 0.01) /
          req.map.info.resolution);

  //std::cout << __FILE__ << __LINE__ << std::endl;
	/*save image*/
	imwrite("erosion.jpg",erosion);

	//find contours
	vector <vector<Point>> map_contours;  
  findContours( erosion,map_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	PointVector map_contour_point;

	if(map_contours.size() > 1)
		ROS_ERROR("Map unsuitable.Remap environment.");
	else{
			geometry_msgs::Point local_point;
			for(int i = 0;i < map_contours[0].size();i++){
				MapToWorld(req.map.info.resolution,
									 req.map.info.origin.position.x,
									 req.map.info.origin.position.y,
									 map_contours[0].at(i).x,
									 map_contours[0].at(i).y,
									 &local_point.x,
									 &local_point.y);
				map_contour_point.push_back(local_point);
			}
	}
  // polygon from request and path for response
  PointVector polygon, candidatePath;
  // start point of coverage path
  geometry_msgs::Point start;
  // parameters of coverage path
  std_msgs::Float64 footprintLength, footprintWidth, horizontalOverwrap, verticalOverwrap;

  polygon = map_contour_point;//req.polygon;
  start = req.start;

  footprintLength = req.footprint_length;
  footprintWidth = req.footprint_width;
  horizontalOverwrap = req.horizontal_overwrap;
  verticalOverwrap = req.vertical_overwrap;

  // isOptimal is true if computed path is optimal
  bool isOptimal = computeConvexCoverage(polygon, footprintWidth.data, horizontalOverwrap.data, candidatePath);

  if (isOptimal == true)
  {
    // fill "subpolygon" field of response so that polygon is visualized
    res.subpolygons = generatePolygonVector(polygon);

    // set optimal alternative as optimal path
    // see torres et al. 2016 for Optimal Alternative
    res.path = identifyOptimalAlternative(polygon, candidatePath, start);
  }
  else
  {
    std::vector<PointVector> subPolygons = decomposePolygon(polygon);

    // sum of length of all coverage path
    double pathLengthSum = 0;

    // compute length of coverage path of each subpolygon
    for (const auto& polygon : subPolygons)
    {
      PointVector partialPath;
      computeConvexCoverage(polygon, footprintWidth.data, horizontalOverwrap.data, partialPath);
      pathLengthSum += calculatePathLength(partialPath);
    }

    // existsSecondOptimalPath is true if there is at least one coverage that has no intersection with polygon
    // second optimal path is the path that has second shortest sweep direction without any intersection with polygon
    PointVector secondOptimalPath;
    bool existsSecondOptimalPath =
        findSecondOptimalPath(polygon, footprintWidth.data, horizontalOverwrap.data, candidatePath);

    if (existsSecondOptimalPath == true)
    {
      // compute optimal alternative for second optimal path
      secondOptimalPath = identifyOptimalAlternative(polygon, candidatePath, start);

      // if the length of second optimal path is shorter than the sum of coverage path of subpolygons,
      // set second optimal path as the path
      if (pathLengthSum > calculatePathLength(secondOptimalPath))
      {
        // fill "subpolygon" field of response so that polygon is visualized
        res.subpolygons = generatePolygonVector(polygon);

        res.path = secondOptimalPath;
        return true;
      }
    }
    else if (subPolygons.size() < 2)
    {
      // if number of subpolygon is smaller than 2,
      // it means no valid path can be computed
      ROS_ERROR("Unable to generate path.");
      return true;
    }

    // fill "subpolygon" field of response so that polygon is visualized
    res.subpolygons = generatePolygonVector(subPolygons);

    // compute coverage path of subpolygons
    PointVector multipleCoveragePath =
        computeMultiplePolygonCoverage(subPolygons, footprintWidth.data, horizontalOverwrap.data);

    res.path = multipleCoveragePath;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zig_zag_cpp_service");
  ros::NodeHandle nh;

  ros::ServiceServer planner = nh.advertiseService("cpp_service", plan);
  ROS_INFO("Ready to plan.");

  ros::spin();

  return 0;
}
