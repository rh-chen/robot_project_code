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
#include "acd2d.h"
#include "acd2d_stat.h"
#include "acd2d_concavity.h"
#include "acd2d_Point.h"


#define delta_rect 1

typedef vtkSmartPointer<vtkPolyData> Polygon;
typedef std::vector<Polygon> PolygonVector;
typedef std::vector<PolygonVector> Layer;

double g_alpha = 0;
double g_beta = 1;
std::string g_concavity_measure = "hybrid1";
//std::string g_concavity_measure = "shortestpath";
//std::string g_concavity_measure = "hybrid2";
double g_tau = 0.1;

namespace Cpp{

using namespace acd2d;

void createPolys(Polygon_list& polygon, cd_2d& cd,float& scale_)
{
    //read polygon
    {
		cd_polygon poly;
        
        int polygon_count = 0;
		std::list<Polygon_2>::iterator iter;
        for(iter = polygon.begin();iter != polygon.end();iter++){
            
            Polygon_2 poly_cell = *iter;

            if(polygon_count == 0){
                cd_poly cd_poly_cell(cd_poly::POUT);
                cd_poly_cell.beginPoly();
                for(int j = 0;j < poly_cell.size();j++){
                    Point_2 p = poly_cell.vertex(j);
                    cd_poly_cell.addVertex(p.x(),p.y());
                }
                cd_poly_cell.endPoly();
                poly.push_back(cd_poly_cell);
            }
            else{
                cd_poly cd_poly_cell(cd_poly::PIN);
                cd_poly_cell.beginPoly();
                for(int j = 0;j < poly_cell.size();j++){
                    Point_2 p = poly_cell.vertex(j);
                    cd_poly_cell.addVertex(p.x(),p.y());
                }
                cd_poly_cell.endPoly();
                poly.push_back(cd_poly_cell);
            }

            polygon_count++;
        }

		scale_ = poly.normalize();
        ROS_INFO_STREAM("scale_" << scale_);
		cd.addPolygon(poly);

        ROS_INFO_STREAM("cd_poly_size:" << polygon_count);
    }
    
    if(cd.getTodoList().empty())
        return;
}

void decomposeAll(cd_2d& cd,Polygon_list& polygon,float scale)
{
    IConcavityMeasure * measure=
    ConcavityMeasureFac::createMeasure(g_concavity_measure);
    cd.updateCutDirParameters(g_alpha,g_beta);

    if(g_concavity_measure=="hybrid2")
        ((HybridMeasurement2*)measure)->setTau(g_tau);
    else if(g_concavity_measure=="shortestpath")
        ((HybridMeasurement2*)measure)->setTau(g_tau);
    else
        ((HybridMeasurement2*)measure)->setTau(g_tau);

    clock_t start=clock();
    cd.decomposeAll(g_tau,measure);
    int time=clock()-start;
    ROS_INFO_STREAM("Decompose All Takes:" << ((double)(time))/CLOCKS_PER_SEC);

    std::list<cd_polygon> acd_polygons_done = cd.getDoneList();
    ROS_INFO_STREAM("acd_polygons_size:" << acd_polygons_done.size());

    std::list<cd_polygon>::iterator iter;
    std::list<cd_poly>::iterator iter_;

    for(iter = acd_polygons_done.begin();iter != acd_polygons_done.end();iter++){
        ROS_INFO_STREAM("acd_polygons_cell_size:" << (*iter).size());
        
        Polygon_2 temp_polygon;
        for(iter_ = (*iter).begin();iter_ != (*iter).end();iter_++){
            ROS_INFO_STREAM("acd_poly_cell_size:" << (*iter_).getSize());
            cd_vertex* list_start = (*iter_).getHead();
            
            int tmp_cnt = (*iter_).getSize();
            while(tmp_cnt > 0){
                Point2d temp_p = list_start->getPos();
                temp_polygon.push_back(Point_2(temp_p[0]*scale,temp_p[1]*scale));
                list_start = list_start->getNext();
                tmp_cnt--;
            }
        }
        polygon.push_back(temp_polygon);
    }
    
    delete measure;
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

double polygonArea(PointVector& pv_,int n){
	if(n < 3)
		return 0;

	double sum = pv_[0].y * (pv_[n-1].x - pv_[1].x);
	for(int i = 1;i < n;i++)
		sum += pv_[i].y * (pv_[i-1].x - pv_[(i+1) % n].x);
	
	sum *= 0.5;

    ROS_INFO_STREAM("fabs_area_sum:" << fabs(sum));
	
    return sum;
}

int minEdgeLength(PointVector& polygon,double deposited_width){
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    
    int lessCount = 0;
    for(int i = 0;i < polygon.size();i++){
        if(i == 0){
            p1 = polygon[polygon.size()-1];
            p2 = polygon[0];
        }
        else{
            p1 = polygon[i-1];
            p2 = polygon[i];
        }
                
        ROS_INFO_STREAM("minELCell:" << std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)));
        if(std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)) < deposited_width)
            lessCount++;
    }
    
    ROS_INFO_STREAM("lessCount:" << lessCount);

    return lessCount;
}
bool isBadPolygon(std::vector<cv::Point2f>& polygon,double step_){
    
    PointVector polygon_;
    for(int i = 0;i < polygon.size();i++){
        geometry_msgs::Point p;
        p.x = polygon[i].x;
        p.y = polygon[i].y;
        p.z = 0.0;

        polygon_.push_back(p);
    }

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
        if(fabs(polygonArea(polygon_convex_hull,polygon_convex_hull.size())) < step_*30){
            return true;
        }
        else{
            if(minEdgeLength(rotatedPolygon,1.0) >= 2 )
                return true;
            else
                return false;
        }
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
	
	//rectlinear polygon
	//std::vector<cv::Point2i> vertices_point;
	//vertices_point = makeOIP(bin,delta_rect);
    //ROS_INFO_STREAM("vertices_point_size:" << vertices_point.size());

    Polygon_list polygon_holes;
    Polygon_2 polygon_ext;
	/*for(int j = 0;j < vertices_point.size();j++){
		double point_x = vertices_point[j].x*req.map_resolution+req.map_origin_x;
		double point_y = vertices_point[j].y*req.map_resolution+req.map_origin_y;
				
		geometry_msgs::Pose pose_;
		pose_.position.x = point_x;
		pose_.position.y = point_y;

		res.pose.push_back(pose_);	
        polygon_ext.push_back(Point_2(point_x,point_y));
	}
    
    if(polygon_ext.is_clockwise_oriented())
        polygon_ext.reverse_orientation();

    polygon_holes.push_back(polygon_ext);*/
	//Find  contour
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point> > valid_internal_contours;

	cv::findContours(bin,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_NONE,cv::Point());

	ROS_INFO_STREAM("find_contours:" << contours.size());
	int external_contour_id;
	int max_external_contour = 0;

	double epsilon_approx_poly = 2.0;	
	//ext 
	for(int i = 0;i < contours.size();i++){
		if(contours[i].size() > max_external_contour){
			max_external_contour = contours[i].size();
			external_contour_id = i;
		}
	}
    ROS_INFO_STREAM("max_external_contour:" << max_external_contour);

    //int
	for(int i = 0;i < contours.size();i++){
        if(i != external_contour_id){
			if(contours[i].size() > req.internal_contour_threshold)
            {
				valid_internal_contours.push_back(contours[i]);
				ROS_INFO("valid_internal_contour_size:%d",contours[i].size());
			}
        }
        else{
             std::vector<cv::Point> contour_dp;
             cv::approxPolyDP(contours[i],contour_dp,epsilon_approx_poly,true);
             for(int j = 0;j < contour_dp.size();j++){
                double point_x = contour_dp[j].x*req.map_resolution+req.map_origin_x;
                double point_y = contour_dp[j].y*req.map_resolution+req.map_origin_y;
                        
                geometry_msgs::Pose pose_;
                pose_.position.x = point_x;
                pose_.position.y = point_y;

                res.pose.push_back(pose_);	
                polygon_ext.push_back(Point_2(point_x,point_y));
            }
            
            if(polygon_ext.is_clockwise_oriented())
                polygon_ext.reverse_orientation();

            polygon_holes.push_back(polygon_ext);       
        }
	}

	/*ROS_INFO("valid_internal_contour_number:%d",valid_internal_contours.size());
	for(int i = 0;i < valid_internal_contours.size();i++){
        Polygon_2 polygon_hole;

        cv::RotatedRect rRect = cv::minAreaRect(valid_internal_contours[i]);
		cv::Point2f vertices[4];
		rRect.points(vertices);

        for(int i = 0;i < 4;i++){
			double point_x = vertices[i].x*req.map_resolution+req.map_origin_x;
			double point_y = vertices[i].y*req.map_resolution+req.map_origin_y;
            
            polygon_hole.push_back(Point_2(point_x,point_y));
		}

        if(polygon_hole.is_counterclockwise_oriented())
            polygon_hole.reverse_orientation();

        polygon_holes.push_back(polygon_hole);
	}*/
   
    cd_2d cd_obj;
    Polygon_list polygon_res;
    float scale = 0;
    createPolys(polygon_holes,cd_obj,scale);
    decomposeAll(cd_obj,polygon_res,scale);

    ROS_INFO_STREAM("polygon_res_size:" << polygon_res.size());
    
    std::list<Polygon_2>::iterator iter;
    for(iter = polygon_res.begin();iter != polygon_res.end();iter++)
    {
        geometry_msgs::Polygon partial_polygon;
        Polygon_2 poly_cell = *iter;
        
        //PointVector partial_point_vector;

        vtkSmartPointer<vtkPolyData> polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
	    vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
	    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
        vtkSmartPointer<vtkPolygon> polygon_cell = vtkSmartPointer<vtkPolygon>::New();
        
        std::vector<cv::Point2f> contour_in;
        std::vector<cv::Point2f> contour_out;
            
        for(int j = 0;j < poly_cell.size();j++){
            geometry_msgs::Point32 point_32;

            Point_2 p = poly_cell.vertex(j);

            point_32.x = p.x();
            point_32.y = p.y();
            
            partial_polygon.points.push_back(point_32);
            contour_in.push_back(cv::Point2f(p.x(),p.y()));
            
            /*geometry_msgs::Point point;
            point.x = p.x();
            point.y = p.y();
            point.z = 0.0;

            partial_point_vector.push_back(point);*/
         }

        res.polygon.push_back(partial_polygon);

        //cv::approxPolyDP(cv::Mat(contour_in), contour_out,0.2,true);
        cv::convexHull(contour_in,contour_out,false,true);

        ROS_INFO_STREAM("contour_out_size:" << contour_out.size());
        
        /*if(isBadPolygon(contour_out,req.deposited_material_width))
            continue;*/


        for(auto p_out:contour_out){
            polygon_cell->GetPointIds()->InsertNextId(pts->GetNumberOfPoints());
            pts->InsertNextPoint(p_out.x,p_out.y,0.0);
        }

        cells->InsertNextCell(polygon_cell);
        
        polygonPolyData->SetPoints(pts);
	    polygonPolyData->SetPolys(cells);

  	    ram_path_planning::AdditiveManufacturingTrajectory msg;
  	    DonghongDing dhd;

	    PolygonVector polygon_vector_;
  	    Layer current_layer_;

	    polygon_vector_.push_back(polygonPolyData);
	    current_layer_.push_back(polygon_vector_);
    
        /*if(current_layer_.size() > 0)
  	    {
    	    std::string error_message;
    	    error_message = dhd.generateOneLayerTrajectory(
													10, 50,polygonPolyData,current_layer_,
                                                    req.deposited_material_width,
                                                    req.contours_filtering_tolerance, M_PI / 6,
                                                    false,
                                                    false);


    	    if(error_message.empty())
    	    {
      		    dhd.connectYamlLayers( 50,90,current_layer_, msg,req.number_of_layers,req.height_between_layers);
    	    }
    	    else
    	    {
      		    ROS_ERROR_STREAM(error_message);
      		    return false;
    	    }
  	    }

        ROS_INFO_STREAM("Trajectory size:" << msg.poses.size());
	    nav_msgs::Path path_cell;
	    for(int i = 0;i < msg.poses.size();i++){
		    geometry_msgs::PoseStamped current_pose;
		    current_pose.pose.position.x = msg.poses[i].pose.position.x;
		    current_pose.pose.position.y = msg.poses[i].pose.position.y;
		    current_pose.pose.position.z = msg.poses[i].pose.position.z;

		    path_cell.poses.push_back(current_pose);
	    }
	    res.path.push_back(path_cell);*/
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
