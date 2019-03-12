#include <string>
#include <strings.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <polypartition.h>
#include <polypartition/PolyPartitionMsg.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <deque>
#define CGAL_PARTITION_BRUTE_FORCE_FIX

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Partition_traits_2<K> Traits;
typedef Traits::Point_2 Point_2;
typedef Traits::Polygon_2 Polygon_2;
typedef Polygon_2::Vertex_iterator Vertex_iterator;
typedef std::list<Polygon_2> Polygon_list;
typedef CGAL::Creator_uniform_2<int, Point_2> Creator;
typedef CGAL::Random_points_in_square_2< Point_2, Creator > Point_generator;


namespace Cpp{

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
/*double polygonArea(PointVector& pv_,int n){
	if(n < 3)
		return 0;

	double sum = pv_[0].y * (pv_[n-1].x - pv_[1].x);
	for(int i = 1;i < n;i++)
		sum += pv_[i].y * (pv_[i-1].x - pv_[(i+1) % n].x);
	
	sum *= 0.5;

	return sum;
}*/
bool PolyPartitionCallback(polypartition::PolyPartitionMsg::Request& req,
			               polypartition::PolyPartitionMsg::Response& res){
  	if (req.height_between_layers <= 0)
  	{
    	ROS_ERROR_STREAM("Height between layers cannot be <= 0");
    	return false;
 	}

  	if (req.deposited_material_width <= 0)
  	{
    	ROS_ERROR_STREAM("Deposited material width cannot be <= 0");
    	return false;
  	}

  	if (req.contours_filtering_tolerance < 0)
  	{
    	ROS_ERROR_STREAM("Contours filtering tolerance cannot be < 0");
    	return false;
  	}

  	if(req.map.data.size() != req.map.info.width*req.map.info.height){
		ROS_ERROR_STREAM("Bad map data...");
		return false;
	}

	/*if(req.transform.size() != 3*3){
		ROS_ERROR_STREAM("Bad transform matrix...");
		return false;
	}*/

	if(req.internal_contour_threshold <= 0){
		ROS_ERROR_STREAM("internal contour threshold cannot be <=0");
	}
	
	for(int i = 0;i < req.map.data.size();i++){
		if(req.map.data[i] == -1)
			req.map.data[i] = 100;
	}

	cv::Mat map(req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());

	cv::Mat bin;
	cv::threshold(map,bin,req.occupancy_threshold,255,cv::THRESH_BINARY_INV);
	
#if 1
	double delta_point = 6;
	cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat bin_out_erode;
	cv::erode(bin,bin_out_erode,element_erode);
	bin_out_erode.copyTo(bin);
	cv::Mat bin_out_dilate; 
	cv::dilate(bin,bin_out_dilate, element_dilate);
 	bin_out_dilate.copyTo(bin);

	//output the bin map data
	std::vector<int8_t> map_data; 
	for(int i = 0;i < bin.rows;i++){
        for(int j = 0;j < bin.cols;j++){
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

	//rectlinear polygon
	std::vector<cv::Point2i> vertices_point;
	vertices_point = makeOIP(bin,delta_point);
    
    
    Polygon_2 polygon;
    for(int j = vertices_point.size()-1;j >= 0;j--)
    {
		double point_x = vertices_point[j].x*req.map_resolution+req.map_origin_x;
		double point_y = vertices_point[j].y*req.map_resolution+req.map_origin_y;
	    polygon.push_back(Point_2(point_x,point_y));			
	}

    if(polygon.is_clockwise_oriented())
        polygon.reverse_orientation();

    TPPLPartition obj;
    std::list<TPPLPoly> polys;
    std::list<TPPLPoly> result;

    TPPLPoly poly;

    ROS_INFO("polygon.size:%d",polygon.size());
    poly.Init(polygon.size());

    for( int i = 0;i < polygon.size();i++){
        Point_2 p = polygon.vertex(i);
        std::cout << "x():" << polygon[i].x() << std::endl;
        std::cout << "y():" << polygon[i].y() << std::endl;
        poly[i].x = p.x();
        poly[i].y = p.y();
        
        geometry_msgs::Pose pose;
        pose.position.x = p.x();
        pose.position.y = p.y();
        res.pose.push_back(pose);
	}

    bool res_polypartition = obj.ConvexPartition_OPT(&poly,&result);
    if(res_polypartition){
        ROS_INFO_STREAM("polypartition_size:" << result.size());
        ROS_INFO_STREAM("ConvexPartition success...");
        std::list<TPPLPoly>::iterator iter;

        for(iter = result.begin();iter != result.end();iter++){
            geometry_msgs::Polygon partial_polygon;
            
            TPPLPoly tpp_poly = *iter;
            for(int j = 0;j < tpp_poly.GetNumPoints();j++){
                geometry_msgs::Point32 point_32;
                point_32.x = tpp_poly.GetPoint(j).x;
                point_32.y = tpp_poly.GetPoint(j).y;

                partial_polygon.points.push_back(point_32);
            }
            res.polygon.push_back(partial_polygon);
        }
        
    }
    else
        ROS_INFO_STREAM("ConvexPartition failure...");

#endif
	return true;
}

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "polypartition_node");
  ros::NodeHandle private_nh("~");

  ros::ServiceServer polypartition_srv = private_nh.advertiseService(
      "/sweeper/polypartition_srv",
      Cpp::PolyPartitionCallback);

  ROS_INFO("Ready to poly partition...");
  ros::spin();

  return 0;
}
