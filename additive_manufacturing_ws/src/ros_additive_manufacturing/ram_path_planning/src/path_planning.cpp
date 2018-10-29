#include <string>
#include <strings.h>

#include <eigen_conversions/eigen_msg.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_path_planning/contours.hpp>
#include <ram_path_planning/donghong_ding.hpp>
#include <ram_path_planning/mesh_slicer.hpp>
#include <ram_path_planning/vtkRenderUpdaterTimer.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <actionlib/server/action_server.h>
#include <ram_path_planning/follow_poses.hpp>
#include <ram_path_planning/ContoursAction.h>
#include <ram_path_planning/DonghongDingAction.h>
#include <ram_path_planning/FollowPosesAction.h>

#include <unique_id/unique_id.h>

typedef vtkSmartPointer<vtkPolyData> Polygon;
typedef std::vector<Polygon> PolygonVector;
typedef std::vector<PolygonVector> Layer;

bool use_gui;

std::unique_ptr<ros::Publisher> traj_pub;

ram_path_planning::DonghongDing<ram_path_planning::DonghongDingAction> donghongding_generator;
ram_path_planning::Contours<ram_path_planning::ContoursAction> contour_generator;
ram_path_planning::FollowPoses<ram_path_planning::FollowPosesAction> follow_poses_generator;

//allow to visualize the generation of trajectory
vtkSmartPointer<vtkRendererUpdaterTimer> cb = vtkSmartPointer<vtkRendererUpdaterTimer>::New();

std::string fileExtension(const std::string full_path)
{
  size_t last_index = full_path.find_last_of("/");
  std::string file_name = full_path.substr(last_index + 1, full_path.size());

  last_index = file_name.find_last_of("\\");
  file_name = file_name.substr(last_index + 1, file_name.size());

  last_index = file_name.find_last_of(".");
  if (last_index == std::string::npos)
    return "";

  return file_name.substr(last_index + 1, file_name.size());
}

// Donghong Ding algorithm
void donghongDingAlgorithmGoalCb(actionlib::ServerGoalHandle<ram_path_planning::DonghongDingAction> gh)
{

  // Change status to ACTIVE
  gh.setAccepted();

  // Action elements
  ram_path_planning::DonghongDingGoal goal;
  ram_path_planning::DonghongDingResult result;
  goal = *gh.getGoal();

  // Verify parameters
  if (goal.file.empty())
  {
    result.error_msg = "File name cannot be empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.height_between_layers <= 0)
  {
    result.error_msg = "Height between layers cannot be <= 0";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.deposited_material_width <= 0)
  {
    result.error_msg = "Deposited material width cannot be <= 0";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }
  if (goal.contours_filtering_tolerance < 0)
  {
    result.error_msg = "Contours filtering tolerance cannot be < 0";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Verify file extension
  bool is_yaml_file(false);
  const std::string file_extension(fileExtension(goal.file));
  if (!strcasecmp(file_extension.c_str(), "yaml") || !strcasecmp(file_extension.c_str(), "yml"))
  {
    is_yaml_file = true;
    if (goal.number_of_layers < 1)
    {
      result.error_msg = "Number of layers cannot be < 1";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }
  else if (strcasecmp(file_extension.c_str(), "obj") && strcasecmp(file_extension.c_str(), "ply")
      && strcasecmp(file_extension.c_str(), "stl"))
  {
    result.error_msg = "File is not a YAML, YML, OBJ, PLY or STL file: " + goal.file;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Create Trajectory message
  ram_msgs::AdditiveManufacturingTrajectory msg;
  msg.file = goal.file;

  // Add request parameter to the trajectory
  msg.generation_info = "Deposited material width = " + std::to_string(goal.deposited_material_width * 1000.0)
      + " mm\n";
  msg.generation_info += "Contours filtering tolerance = " + std::to_string(goal.contours_filtering_tolerance * 1000.0)
      + " mm\n";
  msg.generation_info += "Height between layers = " + std::to_string(goal.height_between_layers * 1000.0) + " mm\n";

  if (!is_yaml_file)
  {
    msg.generation_info += "Slicing direction = " + std::to_string(goal.slicing_direction.x) + ", "
        + std::to_string(goal.slicing_direction.y) + ", " + std::to_string(goal.slicing_direction.z);
  }
  else
  {
    msg.generation_info += "Number of layers = " + std::to_string(goal.number_of_layers);
  }

  msg.similar_layers = is_yaml_file;

  std::array<double, 3> slicing_direction;
  slicing_direction[0] = goal.slicing_direction.x;
  slicing_direction[1] = goal.slicing_direction.y;
  slicing_direction[2] = goal.slicing_direction.z;

  std::vector<Layer> additive_traj;
  cb->current_layer_.clear();

  // Generate trajectory

  // Action feedback
  if (!donghongding_generator.publishStatusPercentageDone("Generating trajectory", 10, gh))
    return;

  if (is_yaml_file)
  {
    std::string error_message;
    error_message = donghongding_generator.generateOneLayerTrajectory(gh, 10, 50, goal.file, cb->current_layer_,
                                                                      goal.deposited_material_width,
                                                                      goal.contours_filtering_tolerance, M_PI / 6,
                                                                      false,
                                                                      use_gui);

    if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
      return;

    if (error_message.empty())
    {
      // Action feedback
      if (!donghongding_generator.publishStatusPercentageDone(
          "Trajectory has been generated for one layer.\n Duplicating and connecting layers", 50, gh))
        return;

      donghongding_generator.connectYamlLayers(gh, 50, 90, cb->current_layer_, msg, goal.number_of_layers,
                                               goal.height_between_layers);

    }
    else
    {
      result.error_msg = error_message;
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }
  else
  {
    // Slice the mesh
    const unsigned nb_layers(
        ram_path_planning::sliceMesh(additive_traj, goal.file, cb->mesh_, cb->stripper_, goal.height_between_layers,
                                     slicing_direction,
                                     use_gui));

    if (nb_layers < 1)
    {
      result.error_msg = "Slicing failed, zero slice generated";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
    // Action feedback
    if (!donghongding_generator.publishStatusPercentageDone("Slicing completed", 20, gh))
      return;

    // Generate trajectory on each layer
    for (unsigned i(0); i < additive_traj.size(); ++i)
    {
      Polygon contours(additive_traj[i][0][0]);
      contours->DeepCopy(additive_traj[i][0][0]);

      int current_progress_value = 20 + i * 30 / nb_layers; // first value is 20%
      int next_progress_value = 20 + (i + 1) * 30 / nb_layers; // last value is 50%

      std::string error_message;
      error_message = donghongding_generator.generateOneLayerTrajectory(gh, current_progress_value, next_progress_value,
                                                                        contours,
                                                                        cb->current_layer_,
                                                                        goal.deposited_material_width,
                                                                        goal.contours_filtering_tolerance,
                                                                        slicing_direction,
                                                                        M_PI / 6,
                                                                        false,
                                                                        use_gui);

      if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
        return;

      if (error_message.empty())
      {
        // Action feedback
        if (!donghongding_generator.publishStatusPercentageDone(
            "Trajectory of layer " + std::to_string(i) + " has been generated", next_progress_value, gh)) // percentage between 20% and 50%
          return;
      }
      else
      {
        result.error_msg = error_message + "\n" + "Could not generate layer " + std::to_string(i);
        ROS_ERROR_STREAM(result.error_msg);
        if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
          gh.setAborted(result);
        return;
      }
      additive_traj[i] = cb->current_layer_;
    }

    std::string return_message;
    return_message = donghongding_generator.connectMeshLayers(gh, 50, 90, additive_traj, msg);

    if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
      return;

    if (!return_message.empty())
    {
      result.error_msg = return_message + "\n" + "Error connecting layers";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }

  // Trajectory is now complete
  // Create a message
  // Fill response and publish trajectory

  // Action feedback
  if (!donghongding_generator.publishStatusPercentageDone(
      "Layers have been connected\nCreating message to publish trajectory", 90, gh))

    if (msg.poses.size() == 0)
    {
      result.error_msg = "Trajectory is empty";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  result.number_of_poses = msg.poses.size();

  // Add UUID
  for (auto &pose : msg.poses)
    pose.unique_id = unique_id::toMsg(unique_id::fromRandom());

  // Set the time to zero, the trajectory is incomplete at this stage
  // It will be published on trajectory_tmp, completed by fill_trajectory and then published on trajectory
  msg.generated.nsec = 0;
  msg.generated.sec = 0;
  msg.modified = msg.generated;

  if (!donghongding_generator.publishStatusPercentageDone("Trajectory has been generated and published", 100, gh))
    return;

  if (traj_pub->getNumSubscribers() == 0)
    ROS_WARN_STREAM("No subscriber on topic " << traj_pub->getTopic());

  traj_pub->publish(msg);

  if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE
      && gh.getGoalStatus().status != actionlib_msgs::GoalStatus::PREEMPTING)
    return;

  gh.setSucceeded(result);
}

void donghongDingAlgorithmCancelCb(actionlib::ServerGoalHandle<ram_path_planning::DonghongDingAction> gh)
{
  if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE
      || gh.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING)
    gh.setCanceled();
}

// Contours algorithm
void contoursAlgorithmGoalCb(actionlib::ServerGoalHandle<ram_path_planning::ContoursAction> gh)
{
  // Change status to ACTIVE
  gh.setAccepted();

  // Action elements
  ram_path_planning::ContoursGoal goal;
  ram_path_planning::ContoursResult result;
  goal = *gh.getGoal();

  // Verify parameters
  if (goal.file.empty())
  {
    result.error_msg = "File name cannot be empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.height_between_layers <= 0)
  {
    result.error_msg = "Height between layers cannot be <= 0";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.deposited_material_width <= 0)
  {
    result.error_msg = "Deposited material width cannot be <= 0";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Verify file extension
  bool is_yaml_file(false);
  const std::string file_extension(fileExtension(goal.file));
  if (!strcasecmp(file_extension.c_str(), "yaml") || !strcasecmp(file_extension.c_str(), "yml"))
  {
    is_yaml_file = true;
    if (goal.number_of_layers < 1)
    {
      result.error_msg = "Number of layers cannot be < 1";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }
  else if (strcasecmp(file_extension.c_str(), "obj") && strcasecmp(file_extension.c_str(), "ply")
      && strcasecmp(file_extension.c_str(), "stl"))
  {
    result.error_msg = "File is not a YAML, YML, OBJ, PLY or STL file: " + goal.file;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Create Trajectory message
  ram_msgs::AdditiveManufacturingTrajectory msg;
  msg.file = goal.file;

  // Add request parameter to the trajectory
  msg.generation_info = "Deposited material width = " + std::to_string(goal.deposited_material_width * 1000.0)
      + " mm\n";
  msg.generation_info += "Height between layers = " + std::to_string(goal.height_between_layers * 1000.0) + " mm\n";

  if (!is_yaml_file)
  {
    msg.generation_info += "Slicing direction = " + std::to_string(goal.slicing_direction.x) + ", "
        + std::to_string(goal.slicing_direction.y) + ", " + std::to_string(goal.slicing_direction.z);
  }
  else
  {
    msg.generation_info += "Number of layers = " + std::to_string(goal.number_of_layers);
  }

  msg.similar_layers = is_yaml_file;
  std::array<double, 3> slicing_direction;
  slicing_direction[0] = goal.slicing_direction.x;
  slicing_direction[1] = goal.slicing_direction.y;
  slicing_direction[2] = goal.slicing_direction.z;

  std::vector<Layer> additive_traj;
  cb->current_layer_.clear();

  // Generate trajectory

  // Action feedback
  if (!contour_generator.publishStatusPercentageDone("Generating trajectory", 10, gh))
    return;

  if (is_yaml_file)
  {
    std::string error_message;
    error_message = contour_generator.generateOneLayerTrajectory(gh, goal.file, cb->current_layer_,
                                                                 goal.deposited_material_width,
                                                                 use_gui);

    if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
      return;

    if (error_message.empty())
    {
      // Action feedback
      if (!contour_generator.publishStatusPercentageDone("Trajectory has been generated.\n Connecting Layers", 50, gh))
        return;

      contour_generator.connectYamlLayers(gh, 50, 90, cb->current_layer_, msg, goal.number_of_layers,
                                          goal.height_between_layers);
    }
    else
    {
      result.error_msg = error_message;
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }
  else
  {
    // Slice the mesh
    const unsigned nb_layers(
        ram_path_planning::sliceMesh(additive_traj, goal.file, cb->mesh_, cb->stripper_, goal.height_between_layers,
                                     slicing_direction,
                                     use_gui));

    if (nb_layers < 1)
    {
      result.error_msg = "Slicing failed, zero slice generated";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
    // Action feedback
    if (!contour_generator.publishStatusPercentageDone("Slicing completed", 20, gh))
      return;

    // Generate trajectory on each layer
    for (unsigned i(0); i < additive_traj.size(); ++i)
    {
      Polygon contours(additive_traj[i][0][0]);
      contours->DeepCopy(additive_traj[i][0][0]);

      std::string error_message;
      error_message = contour_generator.generateOneLayerTrajectory(gh, contours, cb->current_layer_,
                                                                   goal.deposited_material_width,
                                                                   slicing_direction,
                                                                   use_gui);

      if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
        return;

      if (error_message.empty())
      {
        // Action feedback
        if (!contour_generator.publishStatusPercentageDone(
            "Trajectory of layer " + std::to_string(i) + " has been generated", 20 + i * 30 / nb_layers, gh)) // percentage between 20% and 50%
          return;
      }
      else
      {
        result.error_msg = error_message + "\n" + "Could not generate layer " + std::to_string(i);
        ROS_ERROR_STREAM(result.error_msg);
        if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
          gh.setAborted(result);
        return;
      }
      additive_traj[i] = cb->current_layer_;
    }

    std::string return_message;
    return_message = contour_generator.connectMeshLayers(gh, 50, 90, additive_traj, msg);

    if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
      return;

    if (!return_message.empty())
    {
      result.error_msg = return_message + "\n" + "Error connecting layers";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }
  // Trajectory is now complete
  // Create a message
  // Fill response and publish trajectory
  // Action feedback
  if (!contour_generator.publishStatusPercentageDone("Creating message to publish trajectory", 90, gh))
    return;

  if (msg.poses.size() == 0)
  {
    result.error_msg = "Trajectory is empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }
  result.number_of_poses = msg.poses.size();

  // Add UUID
  for (auto &pose : msg.poses)
    pose.unique_id = unique_id::toMsg(unique_id::fromRandom());

  // Set the time to zero, the trajectory is incomplete at this stage
  // It will be published on trajectory_tmp, completed by fill_trajectory and then published on trajectory
  msg.generated.nsec = 0;
  msg.generated.sec = 0;
  msg.modified = msg.generated;

  if (!contour_generator.publishStatusPercentageDone("Trajectory has been generated and published", 100, gh))
    return;

  if (traj_pub->getNumSubscribers() == 0)
    ROS_WARN_STREAM("No subscriber on topic " << traj_pub->getTopic());

  traj_pub->publish(msg);

  if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE
      && gh.getGoalStatus().status != actionlib_msgs::GoalStatus::PREEMPTING)
    return;

  gh.setSucceeded(result);
}

void contoursAlgorithmCancelCb(actionlib::ServerGoalHandle<ram_path_planning::ContoursAction> gh)
{
  if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE
      || gh.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING)
    gh.setCanceled();
}

// Follow poses algorithm
void FollowPosesAlgorithmGoalCb(actionlib::ServerGoalHandle<ram_path_planning::FollowPosesAction> gh)
{
  // Change status to ACTIVE
  gh.setAccepted();
  // Action elements
  ram_path_planning::FollowPosesGoal goal;
  ram_path_planning::FollowPosesResult result;
  goal = *gh.getGoal();

  // Action feedback
  if (!follow_poses_generator.publishStatusPercentageDone("Checking goal parameters", 10, gh))
    return;

  if (goal.file.empty())
  {
    result.error_msg = "File name cannot be empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  // Verify file extension
  const std::string file_extension(fileExtension(goal.file));
  if (strcasecmp(file_extension.c_str(), "yaml") && strcasecmp(file_extension.c_str(), "yml"))
  {
    result.error_msg = "File is not a YAML, YML, " + goal.file;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }
  // Action feedback
  if (!follow_poses_generator.publishStatusPercentageDone("Generating trajectory from YAML file", 40, gh))
    return;

  // Create Trajectory message
  ram_msgs::AdditiveManufacturingTrajectory msg;
  msg.file = goal.file;

  // Generate trajectory
  std::string error_message;
  error_message = follow_poses_generator.generateTrajectory(goal.file, msg);
  if (!error_message.empty())
  {
    result.error_msg = error_message;
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (msg.poses.empty())
  {
    result.error_msg = "Generated trajectory is empty";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }
  if (goal.duplicate_layer && msg.poses.back().layer_index != 0)
  {
    result.error_msg = "Trajectory contains multiple layers, cannot duplicate layers";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  unsigned polygon_count(0);
  for (auto pose : msg.poses)
  {
    if (pose.polygon_start)
      ++polygon_count;
  }

  if (goal.duplicate_layer && polygon_count == 0)
  {
    result.error_msg = "Trajectory contains zero polygon!";
    ROS_ERROR_STREAM(result.error_msg);
    if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
      gh.setAborted(result);
    return;
  }

  if (goal.duplicate_layer && goal.number_of_layers > 1)
  {
    error_message = follow_poses_generator.duplicateLayers(msg, goal.number_of_layers, goal.height_between_layers,
                                                           goal.invert_one_of_two_layers);
    if (!error_message.empty())
    {
      result.error_msg = error_message + "\nError duplicating layers in the trajectory";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  }

  // Trajectory is now complete
  // Create a message
  // Fill response and publish trajectory
  // Action feedback
  if (!follow_poses_generator.publishStatusPercentageDone("Creating message to publish trajectory from file", 70, gh))

    if (msg.poses.size() == 0)
    {
      result.error_msg = "Generated trajectory is empty";
      ROS_ERROR_STREAM(result.error_msg);
      if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        gh.setAborted(result);
      return;
    }
  result.number_of_poses = msg.poses.size();

  // The trajectory is incomplete at this stage
  // It will be published on trajectory_tmp, completed by fill_trajectory and then published on trajectory

  // Action feedback
  if (!follow_poses_generator.publishStatusPercentageDone("Trajectory has been generated", 100, gh))
    return;

  if (traj_pub->getNumSubscribers() == 0)
    ROS_ERROR_STREAM("No subscriber on topic " << traj_pub->getTopic() << ": trajectory is lost");

  traj_pub->publish(msg);

  if (gh.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE
      && gh.getGoalStatus().status != actionlib_msgs::GoalStatus::PREEMPTING)
    return;

  gh.setSucceeded(result);
}

void FollowPosesAlgorithmCancelCb(actionlib::ServerGoalHandle<ram_path_planning::FollowPosesAction> gh)
{
  if (gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE
      || gh.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING)
    gh.setCanceled();
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "path_planing");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  traj_pub.reset(new ros::Publisher);
  *traj_pub = nh.advertise<ram_msgs::AdditiveManufacturingTrajectory>("ram/trajectory_tmp", 1, true);

  // Action servers
  actionlib::ActionServer<ram_path_planning::DonghongDingAction> action_server_1(nh,
                                                                                 donghongding_generator.service_name_,
                                                                                 &donghongDingAlgorithmGoalCb,
                                                                                 &donghongDingAlgorithmCancelCb,
                                                                                 false);
  action_server_1.start();

  actionlib::ActionServer<ram_path_planning::ContoursAction> action_server_2(nh, contour_generator.service_name_,
                                                                             &contoursAlgorithmGoalCb,
                                                                             &contoursAlgorithmCancelCb, false);
  action_server_2.start();

  actionlib::ActionServer<ram_path_planning::FollowPosesAction> action_server_3(nh,
                                                                                follow_poses_generator.service_name_,
                                                                                &FollowPosesAlgorithmGoalCb,
                                                                                &FollowPosesAlgorithmCancelCb,
                                                                                false);
  action_server_3.start();

  nh.param<bool>("use_gui", use_gui, false);
  if (use_gui)
  {
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> render_window = vtkSmartPointer<vtkRenderWindow>::New();
    render_window->AddRenderer(renderer);
    render_window->SetSize(800, 600);

    vtkSmartPointer<vtkRenderWindowInteractor> render_window_interactor =
        vtkSmartPointer<vtkRenderWindowInteractor>::New();
    render_window_interactor->SetRenderWindow(render_window);
    render_window_interactor->Initialize();

    vtkSmartPointer<vtkInteractorStyleTrackballCamera> image_style =
        vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    render_window_interactor->SetInteractorStyle(image_style);

    render_window_interactor->AddObserver(vtkCommand::TimerEvent, cb);
    render_window_interactor->CreateRepeatingTimer(500);

    render_window_interactor->Start();
  }

  ros::waitForShutdown();
  spinner.stop();
  traj_pub.reset();
  return 0;
}
