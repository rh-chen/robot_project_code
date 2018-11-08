#include <string>
#include <strings.h>
#include <eigen_conversions/eigen_msg.h>
#include <ram_path_planning/donghong_ding.hpp>
#include <ram_path_planning/AdditiveManufacturingTrajectory.h>
#include <ros/ros.h>
#include <unique_id/unique_id.h>
#include <visualization_msgs/Marker.h>
#include <ram_path_planning/Cpp.h>


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

  ram_path_planning::AdditiveManufacturingTrajectory msg;
  DonghongDing dhd;
  std::string goal_file = "";
  Layer current_layer_;
  // Generate trajectory
  if (true)
  {
    std::string error_message;
    error_message = dhd.generateOneLayerTrajectory(
													10, 50, goal_file,current_layer_,
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
  // Fill response and publish trajectory
  if (msg.poses.size() == 0)
  {
   	  ROS_ERROR_STREAM("Trajectory is empty");
      return false;
  }
  // Add UUID
  for (auto &pose : msg.poses)
    pose.unique_id = unique_id::toMsg(unique_id::fromRandom());
}

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "cpp_node");
  ros::NodeHandle private_nh("~");

  ros::ServiceServer zigzag_cpp_srv = private_nh.advertiseService(
      "/sweeper/zigzag_cpp",
      Cpp::ZigZagCpp);

  ROS_INFO("Ready to make zigzag cpp...");
  ros::spin();
}
