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

#define DEFECT_LIMIT 12
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

double polygonArea(PointVector& pv_,int n){
	if(n < 3)
		return 0;

	double sum = pv_[0].y * (pv_[n-1].x - pv_[1].x);
	for(int i = 1;i < n;i++)
		sum += pv_[i].y * (pv_[i-1].x - pv_[(i+1) % n].x);
	
	sum = fabs(sum)/2.0;

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

	if(req.external_contour_threshold <= 0){
		ROS_ERROR_STREAM("external contour threshold cannot be <=0");
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
#if 0
	//Find  contour
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
		ROS_INFO("contours[%d]:%d",i,contours[i].size());
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
	
	//add external contour data
	for(int i = 0;i < valid_contours.size();i++){

		vtkSmartPointer<vtkPolygon> polygon_ = vtkSmartPointer<vtkPolygon>::New();
		if(i == 0)
			continue;
		else{
			//cv::RotatedRect rRect = cv::minAreaRect(valid_contours[i]);
			//cv::Point2f vertices[4];
			//rRect.points(vertices);
			std::vector<cv::Point> convex_contour_poly_P;
			cv::convexHull(valid_contours[i],convex_contour_poly_P,false,true);
			/*for(int i = 0;i < 4;i++){
				double point_x = vertices[i].x*req.map_resolution+req.map_origin_x;
				double point_y = vertices[i].y*req.map_resolution+req.map_origin_y;

				polygon_->GetPointIds()->InsertNextId(pts->GetNumberOfPoints());
				pts->InsertNextPoint(point_x,point_y,0.0);
			}*/

			for(int i = 0;i < convex_contour_poly_P.size();i++){
				double point_x = convex_contour_poly_P[i].x*req.map_resolution+req.map_origin_x;
				double point_y = convex_contour_poly_P[i].y*req.map_resolution+req.map_origin_y;

				polygon_->GetPointIds()->InsertNextId(pts->GetNumberOfPoints());
				pts->InsertNextPoint(point_x,point_y,0.0);
			}
			cells->InsertNextCell(polygon_);
		}
	}
#endif
	polygonPolyData->SetPoints(pts);
	polygonPolyData->SetPolys(cells);

	PolygonVector polygon_vector_;
  	Layer current_layer_;

	polygon_vector_.push_back(polygonPolyData);
	current_layer_.push_back(polygon_vector_);
#if 1
  	ram_path_planning::AdditiveManufacturingTrajectory msg;
  	DonghongDing dhd;
	std::vector<PointVector> final_path;
	// Generate trajectory
  	if (current_layer_.size() > 0)
  	{
    	std::string error_message;
    	error_message = dhd.generateOneLayerTrajectory(
													0, 0,polygonPolyData,current_layer_,
                                                    req.deposited_material_width,
                                                    req.contours_filtering_tolerance, M_PI/3,
                                                    false,
                                                    use_gui);

		ROS_INFO("current_layer_size:%d",current_layer_[0].size());

		std_msgs::Float64 footprintLength, footprintWidth, horizontalOverwrap, verticalOverwrap;
		footprintLength.data = 0.3;
		footprintWidth.data = 0.3;
		horizontalOverwrap.data = 0.1;
		verticalOverwrap.data = 0.1;

		for(int i = 0;i < current_layer_[0].size();i++)
		{
			vtkSmartPointer<vtkIdList> cellPointIds = vtkSmartPointer<vtkIdList>::New();
			vtkIdType cellId = 0;

			vtkPolyData* pData = current_layer_[0][i];
			pData->GetCellPoints(cellId,cellPointIds);
			ROS_INFO("cell_number_id:%d",cellPointIds->GetNumberOfIds());
			geometry_msgs::Polygon polygon_divide;
			PointVector polygon_bcd,candidatePath;

			for(vtkIdType j = 0;j < cellPointIds->GetNumberOfIds();j++){
				double point_[3];
				vtkIdType point_id_ = cellPointIds->GetId(j);
				pData->GetPoints()->GetPoint(point_id_,point_);
				geometry_msgs::Point32 point_divide;
				geometry_msgs::Point point_divide_bcd;

				point_divide.x = point_[0];
				point_divide.y = point_[1];
				point_divide.z = point_[2];

				point_divide_bcd.x = point_[0];
				point_divide_bcd.y = point_[1];
				point_divide_bcd.z = point_[2];

				polygon_divide.points.push_back(point_divide);
				polygon_bcd.push_back(point_divide_bcd);
			}
			res.polygon.push_back(polygon_divide);
			double polygon_area = polygonArea(polygon_bcd,polygon_bcd.size());
			ROS_INFO("polygon_area:%f",polygon_area);
			
			bool isOptimal = computeConvexCoverage(polygon_bcd, footprintWidth.data, horizontalOverwrap.data, candidatePath);
			
			ROS_INFO("isOptimal:%d",isOptimal);
			if(isOptimal){
				geometry_msgs::Point start = polygon_bcd[0];
				PointVector optimal_path = identifyOptimalAlternative(polygon_bcd, candidatePath, start);
				//optimal_path.insert(optimal_path.begin(),start);
				//final_path.push_back(optimal_path);
				final_path.push_back(candidatePath);
			}
			else{
				ROS_INFO("zigzag path plannning fail...");
				//return false;
			}
		}
    	/*if (error_message.empty())
    	{
      		dhd.connectYamlLayers( 50,90,current_layer_, msg,req.number_of_layers,req.height_between_layers);
    	}
    	else
    	{
      		ROS_ERROR_STREAM(error_message);
      		return false;
    	}*/
  	}

  	// Trajectory is now complete
  	/*if (msg.poses.size() <= 0)
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
	}*/

#endif
#endif

#if 0
	cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
	cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
	cv::Mat bin_out_erode;
	cv::erode(bin,bin_out_erode,element_erode);
	bin_out_erode.copyTo(bin);
	cv::Mat bin_out_dilate; 
	cv::dilate(bin,bin_out_dilate, element_dilate);
 	bin_out_dilate.copyTo(bin);

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
	
	//double epsilon_approx_poly = 3.0;	
	//add external contour data
	for(int i = 0;i < valid_contours.size();i++){

		vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();

		if(i == 0){
			//std::vector<cv::Point> contour_poly;
			std::vector<cv::Point> convex_contour_poly_P;
 			std::vector<int> convex_contour_poly_I;
			std::vector<cv::Vec4i> convex_defects;

			//cv::approxPolyDP(cv::Mat(valid_contours[i]), contour_poly,epsilon_approx_poly,true);			
			//std::cout << "contour_poly_size:" << contour_poly.size() << std::endl;

			//cv::convexHull(contour_poly,convex_contour_poly_P,false,true);
			cv::convexHull(valid_contours[i],convex_contour_poly_P,false,true);
			//std::cout << "convex_contour_poly_P_size:" << convex_contour_poly_P.size() << std::endl;
			//cv::convexHull(contour_poly,convex_contour_poly_I,false,false);
			cv::convexHull(valid_contours[i],convex_contour_poly_I,false,false);
			//std::cout << "convex_contour_poly_I_size:" << convex_contour_poly_I.size() << std::endl;
			
			//std::cout << "convex_contour_poly_P:" << cv::Mat(convex_contour_poly_P) << std::endl;
			//std::cout << "convex_contour_poly_I:" << cv::Mat(convex_contour_poly_I) << std::endl;

			//cv::convexityDefects(contour_poly,convex_contour_poly_I,convex_defects);
			cv::convexityDefects(valid_contours[i],convex_contour_poly_I,convex_defects);
			//std::cout << "convex_defects_size:" << convex_defects.size() << std::endl;
			//std::cout << "convex_defects:" << cv::Mat(convex_defects) << std::endl;

			std::deque<int> final_point_index;
			int mid_index = -1;
			for(int k = convex_contour_poly_I.size()-1;k >= 0;k--){
				if(final_point_index.size() == 0){
					final_point_index.push_front(convex_contour_poly_I[k]);

					if(hasConvexDefects(convex_defects,convex_contour_poly_I[k],convex_contour_poly_I[k-1],mid_index)){
							final_point_index.push_front(mid_index);
					}

					final_point_index.push_front(convex_contour_poly_I[k-1]);
					k = k-1;
				}
				else{
					if(hasConvexDefects(convex_defects,convex_contour_poly_I[k+1],convex_contour_poly_I[k],mid_index)){
						final_point_index.push_front(mid_index);
					}

					final_point_index.push_front(convex_contour_poly_I[k]);
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
			for(int k = 0;k < final_point_index.size();k++){
				//final_point.push_back(contour_poly[final_point_index[i]]);
				final_point.push_back(valid_contours[i][final_point_index[k]]);
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

			for(int k = 0;k < 4;k++){
				double point_x = vertices[k].x*req.map_resolution+req.map_origin_x;
				double point_y = vertices[k].y*req.map_resolution+req.map_origin_y;

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


    	/*if (error_message.empty())
    	{
      		dhd.connectYamlLayers( 50,90,current_layer_, msg,req.number_of_layers,req.height_between_layers);
    	}
    	else
    	{
      		ROS_ERROR_STREAM(error_message);
      		return false;
    	}*/
  	}

  	// Trajectory is now complete
  	/*if (msg.poses.size() <= 0)
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
	}*/
#endif
#endif
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
}
