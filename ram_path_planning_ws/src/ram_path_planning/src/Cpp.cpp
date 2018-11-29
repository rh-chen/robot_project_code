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

typedef vtkSmartPointer<vtkPolyData> Polygon;
typedef std::vector<Polygon> PolygonVector;
typedef std::vector<PolygonVector> Layer;

bool use_gui = false;

namespace Cpp{

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
	cv::threshold(map,bin,req.occupancy_threshold,255,cv::THRESH_BINARY_INV);

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
			std::vector<cv::Point> convex_contour_poly;

			cv::approxPolyDP(cv::Mat(valid_contours[i]), contour_poly,epsilon_approx_poly,true);			
			std::cout << "contour_poly_size:" << contour_poly.size() << std::endl;

			cv::convexHull(contour_poly,convex_contour_poly,false,true);
			std::cout << "convex_contour_poly_size:" << convex_contour_poly.size() << std::endl;

			for(int j = 0;j < convex_contour_poly.size();j++){
				double point_x = convex_contour_poly[j].x*req.map_resolution+req.map_origin_x;
				double point_y = convex_contour_poly[j].y*req.map_resolution+req.map_origin_y;
				
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
#if 1
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
