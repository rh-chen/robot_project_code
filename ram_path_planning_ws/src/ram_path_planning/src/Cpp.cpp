#include <string>
#include <strings.h>
#include <eigen_conversions/eigen_msg.h>
#include <ram_path_planning/donghong_ding.hpp>
#include <ram_path_planning/AdditiveManufacturingTrajectory.h>
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

#define DEFECT_LIMIT 32
typedef vtkSmartPointer<vtkPolyData> Polygon;
typedef std::vector<Polygon> PolygonVector;
typedef std::vector<PolygonVector> Layer;

bool use_gui = false;

namespace Cpp{
#if 0
typedef struct segment{
		int start;
		int end;
		int direction;
		int height;
		bool visited;
		struct segment* next;
		struct segment* prev;
		
		segment(int start_,int end_,int direction_,int height_){
			start = start_;
			end = end_;
			direction = direction_;
			visited = false;
			next = NULL;
			prev = NULL;
		}
	
}str_seg;

typedef struct vertex{
	int x;
	int y;
	str_seg seg;
	int orientation;
	
	vertex(int x_,int y_,str_seg& seg_,int orientation_):seg(seg_){
		x = x_;
		y = y_;
		orientation = orientation_;
	}
}str_ver;
void getParallelContours(cv::Mat& vv_,int direction_,std::vector<str_seg>& con_){
	int n = vv_.rows;
	int m = vv_.cols;

	bool a = false;
	bool b = false;
	bool c = false;
	bool d = false;

	int x0 = 0;
	int j = 0;
	int i = 0;

	for(j = 0;j < m;j++){
		b = vv_.at<unsigned char>(j,0);
		if(b == a){
			continue;
		}
		
		if(a){
			con_.push_back(str_seg(x0,j,direction_,0));
		}

		if(b){
			x0 = j;
		}

		a = b;
	}

	if(a){
		con_.push_back(str_seg(x0,j,direction_,0));
	}

	for(i = 1;i < n;i++){
		a = false;
		b = false;
		
		x0 = 0;
		for(j = 0;j < m;j++){
			c = vv_.at<unsigned char>(j,i-1);
			d = vv_.at<unsigned char>(j,i);

			if((c == a) && (d == b)){
				continue;
			}

			if(a != b){
				if(a){
					con_.push_back(str_seg(j,x0,direction_,i));
				}
				else{
					con_.push_back(str_seg(x0,j,direction_,i));
				}
			}

			if(c != d){
				x0 = j;
			}
			
			a = c;
			b = d;
		}

		if(a != b){
			if(a){
				con_.push_back(str_seg(j,x0,direction_,i));
			}
			else{
				con_.push_back(str_seg(x0,j,direction_,i));
			}
		}
	}

	a = false;
	x0 = 0;
	for(j = 0;j < m;j++){
		b = vv_.at<unsigned char>(j,n-1);

		if(b == a){
			continue;
		}

		if(a){
			con_.push_back(str_seg(j,x0,direction_,n));
		}

		if(b){
			x0 = j;
		}

		a = b;
	}

	if(a){
		con_.push_back(str_seg(j,x0,direction_,n));
	}
}

void getVertices(std::vector<str_seg>& con_,std::vector<str_ver>& ver_){
	for(int i = 0;i < con_.size();i++){
		str_seg h = con_[i];
		if(h.direction == 0){
			//ver_[2*i] = str_ver(h.start,h.height,h,0);
			ver_.push_back(str_ver(h.start,h.height,h,0));
			//ver_[2*i+1] = str_ver(h.end,h.height,h,1);
			ver_.push_back(str_ver(h.end,h.height,h,1));
		}
		else{
			//ver_[2*i] = str_ver(h.height,h.start,h,0);
			ver_.push_back(str_ver(h.height,h.start,h,0));
			//ver_[2*i+1] = str_ver(h.height,h.end,h,1);
			ver_.push_back(str_ver(h.height,h.end,h,1));
		}
	}
}

void walk(str_seg& v_,bool clockwise,std::vector<cv::Point>& result){
	str_seg v = v_;
	while(!v.visited){
		v.visited = true;
		if(v.direction){
			result.push_back(cv::Point(v.height,v.end));
		}
		else{
			result.push_back(cv::Point(v.start,v.height));
		}

		if(clockwise){
			v = *(v.next);
		}
		else{
			v = *(v.prev);
		}
	}
}

bool compareVertex(str_ver& a,str_ver& b){
	int d = a.x-b.x;
	if(d > 0)
		return true;

	d = a.y-b.y;
	if(d > 0)
		return true;
	
	d = a.orientation-b.orientation;
	if(d > 0)
		return true;
	return false;
}

void getContours(cv::Mat& arr_,bool clockwise,std::vector<std::vector<cv::Point>>& res_){
	std::vector<str_seg> hcontours;
	getParallelContours(arr_,0,hcontours);

	std::vector<str_ver> hvertices;
	getVertices(hcontours,hvertices);
	
	std::sort(hvertices.begin(),hvertices.end(),compareVertex);	
	
	for(int i = 0;i < hvertices.size();i++){
		str_ver h = hvertices[i];
		str_ver v = hvertices[i];
		if(h.orientation){
			h.seg.next = &v.seg;
			v.seg.prev = &h.seg;
		}
		else{
			h.seg.prev = &v.seg;
			v.seg.next = &h.seg;
		}
	}

	for(int j = 0;j < hcontours.size();j++){
		str_seg h = hcontours[j];
		if(h.visited){
			std::vector<cv::Point> res;
			walk(h,clockwise,res);
			res_.push_back(res);
		}
	}
}
#endif

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
      if (p[j] == 0) {
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
      if (p[j] == 0) {
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
  cv::Point2i startpoint = getStartPoint(img, topleftpoint, gsize);
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
bool hasConvexDefects(std::vector<cv::Vec4i>& defects_,int start_,int end_,int& mid_){
	for(int i = 0;i < defects_.size();i++){
		if((defects_[i][0] == start_) && (defects_[i][1] == end_)){
			if(defects_[i][3]/256 > DEFECT_LIMIT){
				mid_ = defects_[i][2];
				return true;
			}
		}
	}
	return false;
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

	if(req.transform.size() != 3*3){
		ROS_ERROR_STREAM("Bad transform matrix...");
		return false;
	}

	if(req.external_contour_threshold <= 0){
		ROS_ERROR_STREAM("external contour threshold cannot be <=0");
	}
	
	for(int i = 0;i < req.map.data.size();i++){
		if(req.map.data[i] == -1)
			req.map.data[i] = 100;
	}

	cv::Mat map(req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());

	cv::Mat bin;
	cv::threshold(map,bin,req.occupancy_threshold,255,cv::THRESH_BINARY);
	
#if 1
	double delta_point = 15;
	cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(delta_point, delta_point));
	cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(delta_point+6, delta_point+6));
	cv::Mat bin_out_erode;
	cv::erode(bin,bin_out_erode,element_erode);
	bin_out_erode.copyTo(bin);
	cv::Mat bin_out_dilate; 
	cv::dilate(bin,bin_out_dilate, element_dilate);
 	bin_out_dilate.copyTo(bin);


	std::vector<cv::Point2i> vertices_point;
	vertices_point = makeOIP(bin,delta_point);

	vtkSmartPointer<vtkPolyData> polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();

	vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();

	for(int j = 0;j < vertices_point.size();j++){
		double point_x = vertices_point[j].x*req.map_resolution+req.map_origin_x;
		double point_y = vertices_point[j].y*req.map_resolution+req.map_origin_y;
				
		geometry_msgs::Pose pose_;
		pose_.position.x = point_x;
		pose_.position.y = point_y;

		res.pose.push_back(pose_);	

		polygon->GetPointIds()->InsertNextId(pts->GetNumberOfPoints());
		pts->InsertNextPoint(point_x,point_y,0.0);
	}

	cells->InsertNextCell(polygon);
	polygonPolyData->SetPoints(pts);
	polygonPolyData->SetPolys(cells);
	
	PolygonVector polygon_vector_;
  	Layer current_layer_;

	polygon_vector_.push_back(polygonPolyData);
	current_layer_.push_back(polygon_vector_);
#if 1
  	ram_path_planning::AdditiveManufacturingTrajectory msg;
  	DonghongDing dhd;
	// Generate trajectory
  	if (current_layer_.size() > 0)
  	{
    	std::string error_message;
    	error_message = dhd.generateOneLayerTrajectory(
													10, 50,polygonPolyData,current_layer_,
                                                    req.deposited_material_width,
                                                    req.contours_filtering_tolerance, M_PI / 6,
                                                    false,
                                                    use_gui);


    	if (error_message.empty())
    	{
      		dhd.connectYamlLayers( 50,90,current_layer_, msg,req.number_of_layers,req.height_between_layers);
    	}
    	else
    	{
      		ROS_ERROR_STREAM(error_message);
      		return false;
    	}
  	}

  	// Trajectory is now complete
  	if (msg.poses.size() <= 0)
  	{
   	  	ROS_ERROR_STREAM("Trajectory is empty");
      	return false;
  	}
	
	ROS_INFO("Trajectory size:%d",(int)msg.poses.size());

	for(int i = 0;i < msg.poses.size();i++){
		geometry_msgs::PoseStamped current_pose;
		current_pose.pose.position.x = msg.poses[i].pose.position.x;
		current_pose.pose.position.y = msg.poses[i].pose.position.y;
		current_pose.pose.position.z = msg.poses[i].pose.position.z;

		res.path.poses.push_back(current_pose);
	}

#endif
#endif

#if 0
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point> > valid_external_contours;
	std::vector<std::vector<cv::Point> > valid_internal_contours;
	std::vector<std::vector<cv::Point> > valid_contours;

	cv::findContours(bin,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_NONE,cv::Point());

	ROS_INFO("find contours:%d",(int)contours.size());
	
	int external_contour_id;
	int max_external_contour = 0;

	//find external contour and has subcontour 
	for(int i = 0;i < contours.size();i++){
		if(contours[i].size() > max_external_contour){
			max_external_contour = contours[i].size();
			external_contour_id = i;
		}
	}

	valid_external_contours.push_back(contours[external_contour_id]);
	valid_contours.push_back(contours[external_contour_id]);
	ROS_INFO("valid_external_contour_size:%d",contours[external_contour_id].size());
	//find subcontour
	for(int i = 0;i < contours.size();i++){
		if(hierarchy[i][3] == external_contour_id){
			if(contours[i].size() > req.external_contour_threshold){
				valid_internal_contours.push_back(contours[i]);
				valid_contours.push_back(contours[i]);
				ROS_INFO("valid_internal_contour_size:%d",contours[i].size());
			}
		}
	}

	ROS_INFO("valid_external_contour_number:%d",valid_external_contours.size());
	ROS_INFO("valid_internal_contour_number:%d",valid_internal_contours.size());
	
	vtkSmartPointer<vtkPolyData> polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	
	double epsilon_approx_poly = 3.0;	
	//add external contour data
	for(int i = 0;i < valid_contours.size();i++){

		vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();

		if(i == 0){
			std::vector<cv::Point> contour_poly;
			std::vector<cv::Point> convex_contour_poly_P;
 			std::vector<int> convex_contour_poly_I;
			std::vector<cv::Vec4i> convex_defects;

			cv::approxPolyDP(cv::Mat(valid_contours[i]), contour_poly,epsilon_approx_poly,true);			
			//std::cout << "contour_poly_size:" << contour_poly.size() << std::endl;

			cv::convexHull(contour_poly,convex_contour_poly_P,false,true);
			//std::cout << "convex_contour_poly_P_size:" << convex_contour_poly_P.size() << std::endl;
			cv::convexHull(contour_poly,convex_contour_poly_I,false,false);
			//std::cout << "convex_contour_poly_I_size:" << convex_contour_poly_I.size() << std::endl;
			
			//std::cout << "convex_contour_poly_P:" << cv::Mat(convex_contour_poly_P) << std::endl;
			//std::cout << "convex_contour_poly_I:" << cv::Mat(convex_contour_poly_I) << std::endl;

			cv::convexityDefects(contour_poly,convex_contour_poly_I,convex_defects);
			//std::cout << "convex_defects_size:" << convex_defects.size() << std::endl;
			//std::cout << "convex_defects:" << cv::Mat(convex_defects) << std::endl;

			std::deque<int> final_point_index;
			int mid_index = -1;
			for(int i = convex_contour_poly_I.size()-1;i >= 0;i--){
				if(final_point_index.size() == 0){
					final_point_index.push_front(convex_contour_poly_I[i]);

					if(hasConvexDefects(convex_defects,convex_contour_poly_I[i],convex_contour_poly_I[i-1],mid_index)){
							final_point_index.push_front(mid_index);
					}

					final_point_index.push_front(convex_contour_poly_I[i-1]);
					i = i-1;
				}
				else{
					if(hasConvexDefects(convex_defects,convex_contour_poly_I[i+1],convex_contour_poly_I[i],mid_index)){
						final_point_index.push_front(mid_index);
					}

					final_point_index.push_front(convex_contour_poly_I[i]);
				}
			}

			if(hasConvexDefects(convex_defects,\
								convex_contour_poly_I[0],\
								convex_contour_poly_I[convex_contour_poly_I.size()-1],\
								mid_index)){
				final_point_index.push_back(mid_index);
			}
			
			/*for(int i = 0;i < final_point_index.size();i++){
				std::cout << "final_point_index:" << final_point_index[i] << std::endl;
			}*/
			
			std::vector<cv::Point> final_point;
			for(int i = 0;i < final_point_index.size();i++){
				final_point.push_back(contour_poly[final_point_index[i]]);
			}
			std::cout << "final_point:" << cv::Mat(final_point) << std::endl;

			for(int j = 0;j < final_point.size();j++){
				double point_x = final_point[j].x*req.map_resolution+req.map_origin_x;
				double point_y = final_point[j].y*req.map_resolution+req.map_origin_y;
				
				geometry_msgs::Pose pose_;
				pose_.position.x = point_x;
				pose_.position.y = point_y;

				res.pose.push_back(pose_);	

				polygon->GetPointIds()->InsertNextId(pts->GetNumberOfPoints());
				pts->InsertNextPoint(point_x,point_y,0.0);
			}
			cells->InsertNextCell(polygon);
		}
		else{
			cv::RotatedRect rRect = cv::minAreaRect(valid_contours[i]);
			cv::Point2f vertices[4];
			rRect.points(vertices);

			for(int i = 0;i < 4;i++){
				double point_x = vertices[i].x*req.map_resolution+req.map_origin_x;
				double point_y = vertices[i].y*req.map_resolution+req.map_origin_y;

				polygon->GetPointIds()->InsertNextId(pts->GetNumberOfPoints());
				pts->InsertNextPoint(point_x,point_y,0.0);
			}
			cells->InsertNextCell(polygon);
		}
	}

	polygonPolyData->SetPoints(pts);
	polygonPolyData->SetPolys(cells);

  	ram_path_planning::AdditiveManufacturingTrajectory msg;
  	DonghongDing dhd;

	PolygonVector polygon_vector_;
  	Layer current_layer_;

	polygon_vector_.push_back(polygonPolyData);
	current_layer_.push_back(polygon_vector_);
#if 0
  	// Generate trajectory
  	if (valid_contours.size() > 0)
  	{
    	std::string error_message;
    	error_message = dhd.generateOneLayerTrajectory(
													10, 50,polygonPolyData,current_layer_,
                                                    req.deposited_material_width,
                                                    req.contours_filtering_tolerance, M_PI / 6,
                                                    false,
                                                    use_gui);


    	if (error_message.empty())
    	{
      		dhd.connectYamlLayers( 50,90,current_layer_, msg,req.number_of_layers,req.height_between_layers);
    	}
    	else
    	{
      		ROS_ERROR_STREAM(error_message);
      		return false;
    	}
  	}

  	// Trajectory is now complete
  	if (msg.poses.size() <= 0)
  	{
   	  	ROS_ERROR_STREAM("Trajectory is empty");
      	return false;
  	}
	
	ROS_INFO("Trajectory size:%d",(int)msg.poses.size());

	for(int i = 0;i < msg.poses.size();i++){
		geometry_msgs::PoseStamped current_pose;
		current_pose.pose.position.x = msg.poses[i].pose.position.x;
		current_pose.pose.position.y = msg.poses[i].pose.position.y;
		current_pose.pose.position.z = msg.poses[i].pose.position.z;

		res.path.poses.push_back(current_pose);
	}
#endif
#endif
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
}
