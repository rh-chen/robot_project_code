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
#include<boost/shared_ptr.hpp>


typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT Ft;
typedef Kernel::Point_2 PointCgal;
typedef Kernel::Segment_2 SegmentCgal;
typedef Kernel::Direction_2 DirectionCgal;
typedef Kernel::Line_2 LineCgal;
typedef Kernel::Vector_2 VectorCgal;
typedef CGAL::Polygon_2<Kernel> PolygonCgal;
typedef CGAL::Polygon_with_holes_2<Kernel> PolygonWithHolesCgal;
typedef std::list<PolygonCgal> PolygonListCgal;
typedef std::vector<PointCgal> ContainerCgal;
typedef CGAL::Straight_skeleton_2<K>  Ss;
typedef boost::shared_ptr<Ss> SsPtr;

typedef CGAL::Straight_skeleton_2<K>::Vertex_const_handle     Vertex_const_handle ;
typedef CGAL::Straight_skeleton_2<K>::Halfedge_const_handle   Halfedge_const_handle ;

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
bool compare_polygon(const PolygonCgal& poly_a,const PolygonCgal& poly_b){
    
    double a_x = 0;
    double a_y = 0;
    double b_x = 0;
    double b_y = 0;

    for(int j = 0;j < poly_a.size();j++){
        PointCgal p = poly_a.vertex(j);

        a_x += p.x();
        a_y += p.y();
    }

    a_x /= (poly_a.size()*1.0);
    a_y /= (poly_a.size()*1.0);

    for(int j = 0;j < poly_b.size();j++){
        PointCgal p = poly_b.vertex(j);

        b_x += p.x();
        b_y += p.y();
    }

    b_x /= (poly_b.size()*1.0);
    b_y /= (poly_b.size()*1.0);


    if(a_x < b_x)
        return true;
    else{
        if(fabs(a_x-b_x) < 1e-6)
        {
            if(a_y > b_y)
                return true;
            else
                return false;
        }
        else
            return false;
    }
}

bool isBadPolygon(PointVector& polygon_,double step_){
    
    if(polygon_.size() < 3)
    {
        return true;
    }

    PointVector polygon_convex_hull;
    Direction sweepDirection = identifyOptimalSweepDir(polygon_,polygon_convex_hull);

    double rotationAngle = calculateHorizontalAngle(sweepDirection.baseEdge.front(), sweepDirection.baseEdge.back());
    PointVector rotatedPolygon = rotatePoints(polygon_convex_hull, -rotationAngle);
	
    for(int i = 0;i < rotatedPolygon.size();i++){
		ROS_INFO("zig_rotatedPolygon_X:%f,zig_rotatedPolygon_Y:%f",rotatedPolygon[i].x,rotatedPolygon[i].y);
	}
    
    PointVector dir{ sweepDirection.opposedVertex, sweepDirection.baseEdge.front(), sweepDirection.baseEdge.back() };
    dir = rotatePoints(dir, -rotationAngle);
    Direction rotatedDir;
    rotatedDir.opposedVertex = dir.at(0);
    rotatedDir.baseEdge.front() = dir.at(1);
    rotatedDir.baseEdge.back() = dir.at(2);

    ROS_INFO("zig_rotatedDir.baseEdge.at(0).y:%f",rotatedDir.baseEdge.at(0).y);
    ROS_INFO("zig_rotatedDir.baseEdge.at(0).x:%f",rotatedDir.baseEdge.at(0).x);
    ROS_INFO("zig_rotatedDir.baseEdge.at(1).y:%f",rotatedDir.baseEdge.at(1).y);
    ROS_INFO("zig_rotatedDir.baseEdge.at(1).x:%f",rotatedDir.baseEdge.at(1).x);

    double verticalDistance = calculateDistance(rotatedDir.baseEdge, rotatedDir.opposedVertex);
    ROS_INFO_STREAM("zig_verticalDistance:" << verticalDistance);

    if(verticalDistance < step_)
            return true;
    else{
        if(fabs(polygonArea(polygon_convex_hull,polygon_convex_hull.size())) < 0.4)
            return true;
        else
            return false;
    }
}
void selectPolygon(PolygonListCgal& src_,PolygonListCgal& dst_)
{
    std::list<PolygonCgal>::iterator iter;
    for(iter = src_.begin();iter != src_.end();iter++){
        PointVector polygon_bcd;

        PolygonCgal poly_cell = *iter;
        for(int j = 0;j < poly_cell.size();j++){
            geometry_msgs::Point point_divide_bcd;

            PointCgal p = poly_cell.vertex(j);

            point_divide_bcd.x = p.x();
            point_divide_bcd.y = p.y();
                
            polygon_bcd.push_back(point_divide_bcd);
       }

       if(!isBadPolygon(polygon_bcd,0.3))
             dst_.push_back(*iter);
    }

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
	
    double delta_point = 6;
	std::vector<cv::Point2i> vertices_point;
	vertices_point = makeOIP(bin,delta_point);

    PolygonCgal poly_cgal;
    
	for(int j = 0;j < vertices_point.size();j++){
		double point_x = vertices_point[j].x*req.map_resolution+req.map_origin_x;
		double point_y = vertices_point[j].y*req.map_resolution+req.map_origin_y;
				
		geometry_msgs::Pose pose_;
		pose_.position.x = point_x;
		pose_.position.y = point_y;

		//res.pose.push_back(pose_);	
        poly_cgal.push_back(Point_2(point_x,point_y));
	}
    
    if(poly_cgal.is_clockwise_oriented())
        poly_cgal.reverse_orientation();

    PolygonWithHolesCgal polyHoles(poly_cgal);
    polyHoles.outer_boundary() = poly_cgal;
    
    SsPtr iss = CGAL::create_interior_straight_skeleton_2(polyHoles);
    ROS_INFO_STREAM("iss.size_of_vertices():" << iss->size_of_vertices());
    ROS_INFO_STREAM("iss.size_of_halfedges():" << iss->size_of_halfedges());
    ROS_INFO_STREAM("iss.size_of_faces():" << iss->size_of_faces());

    if(iss){
        for(auto face = iss->faces_begin();face != iss->faces_end();face++){
            Ss::Halfedge_const_handle begin = face->halfedge();
            Ss::Halfedge_const_handle edge = begin;

            do{
                geometry_msgs::Pose p_t;
                
                const Vertex_const_handle& v = edge->vertex();
                //p_t.x = edge->vertex()->point().x();
                //p_t.y = edge->vertex()->point().y();

                if(v->is_skeleton())
                {
                    p_t.position.x = v->point().x();
                    p_t.position.y = v->point().y();
                    p_t.position.z = 0.f;

                    res.point_skeleton.push_back(p_t);
                }

                edge = edge->prev();
            }while(edge != begin);
        }
    }
	/*std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(bin,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,cv::Point());
    ROS_INFO_STREAM("contours_size:" << contours.size());
	
    
    int external_contour_id;
	int max_external_contour = 0;
	//ext
	for(int i = 0;i < contours.size();i++){
        ROS_INFO_STREAM("contour_obj:" << contours[i].size());
		if(contours[i].size() > max_external_contour){
			max_external_contour = contours[i].size();
			external_contour_id = i;
		}
	}
    
    std::vector<cv::Point> contour_ext_dp;
    cv::approxPolyDP(contours[external_contour_id],contour_ext_dp,12.0,true);

    PolygonCgal poly_cgal;
	for(int j = 0;j < contour_ext_dp.size();j++){
		double point_x = contour_ext_dp[j].x*req.map_resolution+req.map_origin_x;
		double point_y = contour_ext_dp[j].y*req.map_resolution+req.map_origin_y;
				
		geometry_msgs::Pose pose_;
		pose_.position.x = point_x;
		pose_.position.y = point_y;

		res.pose.push_back(pose_);	
        poly_cgal.push_back(Point_2(point_x,point_y));
	}
    
    PolygonWithHolesCgal polyHoles(poly_cgal);
    polyHoles.outer_boundary() = poly_cgal;*/
#if 0
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
        PolygonCgal holes;
        
        cv::Rect rect = cv::boundingRect(valid_internal_contours[i]);
        
        double point_top_left_x = rect.x*req.map_resolution+req.map_origin_x;
        double point_top_left_y = rect.y*req.map_resolution+req.map_origin_y;
        double point_top_right_x = (rect.x+rect.width)*req.map_resolution+req.map_origin_x;
        double point_top_right_y = rect.y*req.map_resolution+req.map_origin_y;
        double point_bot_left_x = rect.x*req.map_resolution+req.map_origin_x;
        double point_bot_left_y = (rect.y+rect.height)*req.map_resolution+req.map_origin_y;
        double point_bot_right_x = (rect.x+rect.width)*req.map_resolution+req.map_origin_x;
        double point_bot_right_y = (rect.y+rect.height)*req.map_resolution+req.map_origin_y;

        holes.push_back(PointCgal(point_top_left_x,point_top_left_y));
        holes.push_back(PointCgal(point_top_right_x,point_top_right_y));
        holes.push_back(PointCgal(point_bot_right_x,point_bot_right_y));
        holes.push_back(PointCgal(point_bot_left_x,point_bot_left_y));

        /*cv::RotatedRect rRect = cv::minAreaRect(valid_internal_contours[i]);
		cv::Point2f vertices[4];
		rRect.points(vertices);

        for(int i = 0;i < 4;i++){
			double point_x = vertices[i].x*req.map_resolution+req.map_origin_x;
			double point_y = vertices[i].y*req.map_resolution+req.map_origin_y;

            holes.push_back(PointCgal(point_x,point_y));
		}*/

		/*std::vector<cv::Point> convex_contour_poly_p;
		cv::convexHull(valid_internal_contours[i],convex_contour_poly_p,false,true);

		for(int i = 0;i < convex_contour_poly_p.size();i++){
			double point_x = convex_contour_poly_p[i].x*req.map_resolution+req.map_origin_x;
			double point_y = convex_contour_poly_p[i].y*req.map_resolution+req.map_origin_y;
            
            holes.push_back(PointCgal(point_x,point_y));
		}*/

        ROS_INFO("hole_size:%d",holes.size());
        polyHoles.add_hole(holes);
	}
#endif

    //PolygonListCgal partition_polys_;
    //PolygonListCgal partition_polys;
    
    /*std::vector<PointVector> subPolygons;
    std::vector<PointVector> final_path;
	nav_msgs::Path plan_path;
    std_msgs::Float64 footprintLength, footprintWidth, horizontalOverwrap, verticalOverwrap;
    footprintLength.data = step_length;
    footprintWidth.data = step_length; 
    horizontalOverwrap.data = step_overlap;
    verticalOverwrap.data = step_overlap;*/

	// Generate trajectory
  	/*if (true)
  	{
        std::cout << __FILE__ << __LINE__ << std::endl;
        CGAL::Polygon_vertical_decomposition_2<Kernel,ContainerCgal> obj;
        std::cout << __FILE__ << __LINE__ << std::endl;
        obj(polyHoles,std::back_inserter(partition_polys));
        std::cout << __FILE__ << __LINE__ << std::endl;

        //partition_polys.sort(compare_polygon);

        std::list<PolygonCgal>::iterator iter;
        for(iter = partition_polys.begin();iter != partition_polys.end();iter++){
            geometry_msgs::Polygon partial_polygon;
            PointVector polygon_bcd,candidatePath;

            PolygonCgal poly_cell = *iter;
            for(int j = 0;j < poly_cell.size();j++){
                geometry_msgs::Point32 point_32;
                geometry_msgs::Point point_divide_bcd;

                PointCgal p = poly_cell.vertex(j);

                point_32.x = p.x();
                point_32.y = p.y();
                point_divide_bcd.x = p.x();
                point_divide_bcd.y = p.y();
                
                polygon_bcd.push_back(point_divide_bcd);

                partial_polygon.points.push_back(point_32);
             }
             res.polygon.push_back(partial_polygon);
                
        }
  	}*/

    /*for(int index_i = 0;index_i < final_path.size();index_i++){
		for(int index_j = 0;index_j < final_path[index_i].size();index_j++){
			geometry_msgs::PoseStamped current_pose;
			current_pose.pose.position.x = final_path[index_i][index_j].x;
			current_pose.pose.position.y = final_path[index_i][index_j].y;
			current_pose.pose.position.z = final_path[index_i][index_j].z;

			plan_path.poses.push_back(current_pose);
		}
	}

    res.path.push_back(plan_path);*/

    /*for(int index_i = 0;index_i < final_path.size();index_i++){
		nav_msgs::Path path_bcd;

		for(int index_j = 0;index_j < final_path[index_i].size();index_j++){
			geometry_msgs::PoseStamped current_pose;
			current_pose.pose.position.x = final_path[index_i][index_j].x;
			current_pose.pose.position.y = final_path[index_i][index_j].y;
			current_pose.pose.position.z = final_path[index_i][index_j].z;

			path_bcd.poses.push_back(current_pose);
		}
		res.path.push_back(path_bcd);
	}*/


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
