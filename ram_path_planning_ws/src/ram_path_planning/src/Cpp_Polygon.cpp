#include <string>
#include <strings.h>
#include <eigen_conversions/eigen_msg.h>
#include <ram_path_planning/donghong_ding.hpp>
#include <ram_path_planning/AdditiveManufacturingTrajectory.h>
#include <ram_path_planning/cgutil.hpp>
#include <ram_path_planning/cpp_uav.hpp>
#include <ros/ros.h>
#include <unique_id/unique_id.h>
#include <visualization_msgs/Marker.h>
#include <ram_path_planning/Cpp.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <deque>

#include <polypartition.h>

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
typedef std::list<Polygon_2> Polygon_list;

bool use_gui = false;

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
/*bool hasConvexDefects(std::vector<cv::Vec4i>& defects_,int start_,int end_,int& mid_){
	for(int i = 0;i < defects_.size();i++){
		if((defects_[i][0] == start_) && (defects_[i][1] == end_)){
			if(defects_[i][3]/256 > DEFECT_LIMIT){
				mid_ = defects_[i][2];
				return true;
			}
		}
	}
	return false;
}*/

double polygonArea(PointVector& pv_,int n){
	if(n < 3)
		return 0;

	double sum = pv_[0].y * (pv_[n-1].x - pv_[1].x);
	for(int i = 1;i < n;i++)
		sum += pv_[i].y * (pv_[i-1].x - pv_[(i+1) % n].x);
	
	sum *= 0.5;

	return sum;
}
bool ZigZagCpp(ram_path_planning::Cpp::Request& req,
			   ram_path_planning::Cpp::Response& res){
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
	
	double delta_point = 10;

	//rectlinear polygon
	std::vector<cv::Point2i> vertices_point;
	vertices_point = makeOIP(bin,delta_point);

    Polygon_2 polygon;
    
	for(int j = 0;j < vertices_point.size();j++){
		double point_x = vertices_point[j].x*req.map_resolution+req.map_origin_x;
		double point_y = vertices_point[j].y*req.map_resolution+req.map_origin_y;
				
		geometry_msgs::Pose pose_;
		pose_.position.x = point_x;
		pose_.position.y = point_y;

		res.pose.push_back(pose_);	
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
        //ROS_INFO_STREAM("x():" << polygon[i].x());
        //ROS_INFO_STREAM("y():" << polygon[i].y());
        poly[i].x = p.x();
        poly[i].y = p.y();
    }

    polys.push_back(poly);

	//Find  contour
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point> > valid_internal_contours;

	cv::findContours(bin,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_NONE,cv::Point());

	ROS_INFO("find contours:%d",(int)contours.size());
	int external_contour_id;
	int max_external_contour = 0;

	//find external contour and has subcontour 
	for(int i = 0;i < contours.size();i++){
		ROS_INFO("contours[%d]:%d",i,contours[i].size());
		if(contours[i].size() > max_external_contour){
			max_external_contour = contours[i].size();
			external_contour_id = i;
		}
	}

	ROS_INFO("valid_external_contour_size:%d",contours[external_contour_id].size());
	//find subcontour
	for(int i = 0;i < contours.size();i++){
        if(i != external_contour_id)
			if(contours[i].size() > req.internal_contour_threshold)
            {
				valid_internal_contours.push_back(contours[i]);
				ROS_INFO("valid_internal_contour_size:%d",contours[i].size());
			}
	}

	ROS_INFO("valid_internal_contour_number:%d",valid_internal_contours.size());
	
	//add internal contour data
	for(int i = 0;i < valid_internal_contours.size();i++){
	    TPPLPoly poly_cell;
        Polygon_2 polygon_cell;
        /*cv::RotatedRect rRect = cv::minAreaRect(valid_internal_contours[i]);
		cv::Point2f vertices[4];
		rRect.points(vertices);

        for(int i = 0;i < 4;i++){
			double point_x = vertices[i].x*req.map_resolution+req.map_origin_x;
			double point_y = vertices[i].y*req.map_resolution+req.map_origin_y;

			polygon_->GetPointIds()->InsertNextId(pts->GetNumberOfPoints());
			pts->InsertNextPoint(point_x,point_y,0.0);
		}*/

		std::vector<cv::Point> convex_contour_poly_p;
		cv::convexHull(valid_internal_contours[i],convex_contour_poly_p,false,true);

		for(int i = 0;i < convex_contour_poly_p.size();i++){
			double point_x = convex_contour_poly_p[i].x*req.map_resolution+req.map_origin_x;
			double point_y = convex_contour_poly_p[i].y*req.map_resolution+req.map_origin_y;
            
            polygon_cell.push_back(Point_2(point_x,point_y));
		}

        if(polygon_cell.is_counterclockwise_oriented())
            polygon_cell.reverse_orientation();

        ROS_INFO("polygon_cell.size:%d",polygon_cell.size());
        poly_cell.Init(polygon_cell.size());

        for( int i = 0;i < polygon_cell.size();i++){
            Point_2 p_cell = polygon_cell.vertex(i);
            //ROS_INFO_STREAM("x():" << polygon_cell[i].x());
            //ROS_INFO_STREAM("y():" << polygon_cell[i].y());
            poly_cell[i].x = p_cell.x();
            poly_cell[i].y = p_cell.y();
        }

        polys.push_back(poly_cell);
	}

	std::vector<PointVector> final_path;
	// Generate trajectory
  	if (polys.size() > 0)
  	{

        bool res_polypartition = obj.ConvexPartition_CGAL(&polys,&result); 
        ROS_INFO_STREAM("res_polypartition:" << res_polypartition);

        if(!res_polypartition)
            return false;
        
        ROS_INFO_STREAM("polypartition_size:" << result.size());
		ROS_INFO_STREAM("ConvexPartition success...");
        
        std_msgs::Float64 footprintLength, footprintWidth, horizontalOverwrap, verticalOverwrap;
        footprintLength.data = 0.3;
        footprintWidth.data = 0.3; 
        horizontalOverwrap.data = 0.1;
        verticalOverwrap.data = 0.1;

        std::list<TPPLPoly>::iterator iter;
        for(iter = result.begin();iter != result.end();iter++){
            geometry_msgs::Polygon partial_polygon;
            PointVector polygon_bcd,candidatePath;

            TPPLPoly tpp_poly = *iter;
            for(int j = 0;j < tpp_poly.GetNumPoints();j++){
                geometry_msgs::Point32 point_32;
                geometry_msgs::Point point_divide_bcd;

                point_32.x = tpp_poly.GetPoint(j).x;
                point_32.y = tpp_poly.GetPoint(j).y;
                
                point_divide_bcd.x = tpp_poly.GetPoint(j).x;
                point_divide_bcd.y = tpp_poly.GetPoint(j).y;

                partial_polygon.points.push_back(point_32);
                polygon_bcd.push_back(point_divide_bcd);
             }
             res.polygon.push_back(partial_polygon);

             bool isOptimal = computeConvexCoverage(polygon_bcd, footprintWidth.data, horizontalOverwrap.data, candidatePath);
                
             ROS_INFO_STREAM("isOptimal:" << isOptimal);
			 if(isOptimal){
		        geometry_msgs::Point start = polygon_bcd[0];

				PointVector optimal_path = identifyOptimalAlternative(polygon_bcd, candidatePath, start);
				
                optimal_path.insert(optimal_path.begin(),start);
				final_path.push_back(optimal_path);
			 }
			 else{
			    ROS_INFO("zigzag path plannning fail...");
				return false;
			 }
        }
  	}

	for(int index_i = 0;index_i < final_path.size();index_i++){
		nav_msgs::Path path_bcd;

		for(int index_j = 0;index_j < final_path[index_i].size();index_j++){
			geometry_msgs::PoseStamped current_pose;
			current_pose.pose.position.x = final_path[index_i][index_j].x;
			current_pose.pose.position.y = final_path[index_i][index_j].y;
			current_pose.pose.position.z = final_path[index_i][index_j].z;

			path_bcd.poses.push_back(current_pose);
		}
		res.path.push_back(path_bcd);
	}
	return true;
}

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "cpp_node");
  ros::NodeHandle private_nh("~");

  ros::ServiceServer zigzag_cpp_srv = private_nh.advertiseService(
      "/sweeper/zigzag_cpp_srv",
      Cpp::ZigZagCpp);

  ROS_INFO("Ready to make zigzag cpp...");
  ros::spin();

  return 0;
}
