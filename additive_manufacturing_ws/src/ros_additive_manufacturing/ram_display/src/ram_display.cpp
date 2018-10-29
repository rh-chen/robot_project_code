#include <string>
#include <mutex>

#include <eigen_conversions/eigen_msg.h>
#include <interactive_markers/interactive_marker_server.h>
#include <ram_display/DeleteTrajectory.h>
#include <ram_display/DisplayTrajectory.h>
#include <ram_display/UpdateMeshColor.h>
#include <ram_display/UpdateSelection.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_utils/GetTool.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <std_msgs/ColorRGBA.h>

ros::Publisher mesh_pub;

rviz_visual_tools::RvizVisualToolsPtr rvt_trajectory;
rviz_visual_tools::RvizVisualToolsPtr rvt_selection;

std::string trajectory_frame;

// Get tool service client
ros::ServiceClient get_tool_client;

// Trajectory
std::recursive_mutex trajectory_mutex;
ram_msgs::AdditiveManufacturingTrajectory additive_traj;

// Display parameters
std::recursive_mutex display_params_mutex;
ram_display::DisplayTrajectory::Request display_params;

// Selection
std::recursive_mutex selection_params_mutex;
ram_display::UpdateSelection::Request selection;

bool updateSelection(bool display = true);

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

std_msgs::ColorRGBA intToColor(const unsigned int color)
{
  std_msgs::ColorRGBA rvalue;
  rvalue.a = 1;

  switch (color % 28)
  {
    case 0:
      rvalue.r = 225;
      rvalue.g = 11;
      rvalue.b = 29;
      break;
    case 1:
      rvalue.r = 0;
      rvalue.g = 136;
      rvalue.b = 31;
      break;
    case 2:
      rvalue.r = 143;
      rvalue.g = 63;
      rvalue.b = 250;
      break;
    case 3:
      rvalue.r = 0;
      rvalue.g = 172;
      rvalue.b = 197;
      break;
    case 4:
      rvalue.r = 255;
      rvalue.g = 128;
      rvalue.b = 207;
      break;
    case 5:
      rvalue.r = 139;
      rvalue.g = 254;
      rvalue.b = 60;
      break;
    case 6:
      rvalue.r = 113;
      rvalue.g = 8;
      rvalue.b = 78;
      break;
    case 7:
      rvalue.r = 255;
      rvalue.g = 165;
      rvalue.b = 66;
      break;
    case 8:
      rvalue.r = 0;
      rvalue.g = 13;
      rvalue.b = 154;
      break;
    case 9:
      rvalue.r = 136;
      rvalue.g = 112;
      rvalue.b = 105;
      break;
    case 10:
      rvalue.r = 0;
      rvalue.g = 73;
      rvalue.b = 66;
      break;
    case 11:
      rvalue.r = 0;
      rvalue.g = 253;
      rvalue.b = 208;
      break;
    case 12:
      rvalue.r = 82;
      rvalue.g = 42;
      rvalue.b = 14;
      break;
    case 13:
      rvalue.r = 188;
      rvalue.g = 183;
      rvalue.b = 252;
      break;
    case 14:
      rvalue.r = 146;
      rvalue.g = 180;
      rvalue.b = 125;
      break;
    case 15:
      rvalue.r = 200;
      rvalue.g = 18;
      rvalue.b = 182;
      break;
    case 16:
      rvalue.r = 0;
      rvalue.g = 103;
      rvalue.b = 160;
      break;
    case 17:
      rvalue.r = 41;
      rvalue.g = 6;
      rvalue.b = 64;
      break;
    case 18:
      rvalue.r = 224;
      rvalue.g = 179;
      rvalue.b = 176;
      break;
    case 19:
      rvalue.r = 255;
      rvalue.g = 245;
      rvalue.b = 152;
      break;
    case 20:
      rvalue.r = 81;
      rvalue.g = 69;
      rvalue.b = 90;
      break;
    case 21:
      rvalue.r = 168;
      rvalue.g = 124;
      rvalue.b = 35;
      break;
    case 22:
      rvalue.r = 255;
      rvalue.g = 114;
      rvalue.b = 106;
      break;
    case 23:
      rvalue.r = 51;
      rvalue.g = 129;
      rvalue.b = 111;
      break;
    case 24:
      rvalue.r = 136;
      rvalue.g = 7;
      rvalue.b = 21;
      break;
    case 25:
      rvalue.r = 166;
      rvalue.g = 124;
      rvalue.b = 177;
      break;
    case 26:
      rvalue.r = 49;
      rvalue.g = 78;
      rvalue.b = 19;
      break;
    default: // case 28
      rvalue.r = 144;
      rvalue.g = 228;
      rvalue.b = 253;
      break;
  }

  rvalue.r /= 255.0;
  rvalue.g /= 255.0;
  rvalue.b /= 255.0;
  return rvalue;
}

// Return true if the a pose in the trajectory is close to another pose
bool closeToPose(const unsigned pose_index,
                 const double tolerance)
{
  // Current point
  Eigen::Vector3d current_p(Eigen::Vector3d::Identity());
  tf::pointMsgToEigen(additive_traj.poses[pose_index].pose.position, current_p);
  for (unsigned i(0); i < pose_index; ++i)
  {
    // Points already published
    Eigen::Vector3d p(Eigen::Vector3d::Identity());
    tf::pointMsgToEigen(additive_traj.poses[i].pose.position, p);

    if ((current_p - p).norm() < tolerance)
      return true;
  }

  return false;
}

bool publishCylinders(const EigenSTL::vector_Vector3d &a_points,
                      const EigenSTL::vector_Vector3d &b_points,
                      const std::vector<std_msgs::ColorRGBA> &colors,
                      double radius)
{
  if (a_points.size() != b_points.size())
  {
    ROS_ERROR_STREAM("Skipping path because " << a_points.size() << " different from " << b_points.size());
    return false;
  }

  if (a_points.size() != colors.size())
  {
    ROS_ERROR_STREAM("Skipping path because " << a_points.size() << " different from " << colors.size());
    return false;
  }

  // Create the cylinders
  for (unsigned i(0); i < a_points.size(); ++i)
    rvt_trajectory->publishCylinder(a_points[i], b_points[i], colors[i], radius, "Cylinders");
  return true;

}

void displayMesh(const std_msgs::ColorRGBA color,
                 const std::string frame_id = "base",
                 const std::string mesh_file_name = "")
{
  visualization_msgs::Marker mesh_marker;
  mesh_marker.pose.orientation.w = 1;

  if (mesh_file_name.empty())
  {
    mesh_marker.header.frame_id = frame_id;
    mesh_marker.header.stamp = ros::Time();
    mesh_marker.ns = "mesh";
    mesh_marker.id = 0;
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.action = visualization_msgs::Marker::DELETE;
  }
  else
  {
    mesh_marker.header.frame_id = frame_id;
    mesh_marker.header.stamp = ros::Time();
    mesh_marker.ns = "mesh";
    mesh_marker.id = 0;
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.action = visualization_msgs::Marker::ADD;
    mesh_marker.pose.position.x = 0;
    mesh_marker.pose.position.y = 0;
    mesh_marker.pose.position.z = 0;
    mesh_marker.scale.x = 1;
    mesh_marker.scale.y = 1;
    mesh_marker.scale.z = 1;
    mesh_marker.color = color;
    mesh_marker.mesh_resource = "file://" + mesh_file_name;
    mesh_marker.frame_locked = true;
  }

  if (mesh_pub.getNumSubscribers() == 0)
    ROS_ERROR_STREAM("No subscriber on topic " << mesh_pub.getTopic() << ": mesh will not be displayed");
  mesh_pub.publish(mesh_marker);
}

void deleteTrajectory()
{
  std_msgs::ColorRGBA color;
  displayMesh(color);
  rvt_trajectory->deleteAllMarkers();
  rvt_trajectory->trigger();
  updateSelection(false); // Don't clear the selection, but don't display it!
}

std::string displayTrajectory()
{
  std::lock_guard<std::recursive_mutex> lock_1(trajectory_mutex);
  std::lock_guard<std::recursive_mutex> lock_2(display_params_mutex);

  if (additive_traj.poses.empty())
    return "Trajectory is empty";

  // Validate parameters
  if (display_params.color_mode > 4)
    return "color_mode must be between 0 and 4. color_mode = " + std::to_string(display_params.color_mode);

  if (display_params.display_mode > 3)
    return "display_mode must be between 0 and 3. display_mode = " + std::to_string(display_params.display_mode);

  if (display_params.wire_size <= 0)
    return "wire_size cannot be zero/negative. wire_size = " + std::to_string(display_params.wire_size);

  if (display_params.cylinder_size <= 0)
    return "cylinder_size cannot be zero/negative. cylinder_size = " + std::to_string(display_params.cylinder_size);

  if (display_params.axis_size <= 0)
    return "axis_size cannot be zero/negative. axis_size = " + std::to_string(display_params.axis_size);

  if (display_params.display_labels && (display_params.labels_size <= 0))
    return "labels_size cannot be zero/negative. labels_size = " + std::to_string(display_params.labels_size);

  if (display_params.display_range_of_layers)
  {
    if (display_params.last_layer < display_params.first_layer)
      return "first_layer must be less than last_layer";

    unsigned highest_level(additive_traj.poses.front().layer_level);
    for (auto ram_pose : additive_traj.poses)
    {
      if (ram_pose.layer_level > highest_level)
        highest_level = ram_pose.layer_level;
    }

    if (display_params.last_layer > highest_level)
      return "Cannot display this range of layers, last layer do not exist in the trajectory.\n"
          "Try with a lower \"Last layer\" value.";
  }

  deleteTrajectory();

  std::string file_extension(fileExtension(additive_traj.file));
  if (!strcasecmp(file_extension.c_str(), "yaml") || !strcasecmp(file_extension.c_str(), "yml"))
  {
    std_msgs::ColorRGBA color;
    displayMesh(color);
  }
  else if (!strcasecmp(file_extension.c_str(), "obj") ||
      !strcasecmp(file_extension.c_str(), "ply") ||
      !strcasecmp(file_extension.c_str(), "stl"))
  {
    displayMesh(display_params.mesh_color,
                trajectory_frame,
                additive_traj.file);
  }

  std::vector<std_msgs::ColorRGBA> colors_vector;
  std::map<double, std_msgs::ColorRGBA> color_map;
  unsigned color(0);
  EigenSTL::vector_Affine3d path;
  EigenSTL::vector_Vector3d path_positions;

  // Get the highest level of the trajectory
  // Warning: This is NOT necessarily the last pose level!
  unsigned maximum_level(0);
  for (auto pose : additive_traj.poses)
    if (pose.layer_level > maximum_level)
      maximum_level = pose.layer_level;

  std::vector<unsigned> pose_index_within_layer_count(maximum_level + 1);
  std::fill(pose_index_within_layer_count.begin(), pose_index_within_layer_count.end(), 0);

  EigenSTL::vector_Vector3d a_points;
  EigenSTL::vector_Vector3d b_points;

  Eigen::Affine3d pose_affine(Eigen::Affine3d::Identity());
  Eigen::Affine3d previous_pose_affine(Eigen::Affine3d::Identity());
  unsigned previous_i(0);

  for (unsigned i(0); i < additive_traj.poses.size(); ++i)
  {
    if ((additive_traj.poses[i].layer_level < display_params.first_layer
        || additive_traj.poses[i].layer_level > display_params.last_layer) && display_params.display_range_of_layers)
      continue;
    previous_pose_affine = pose_affine;
    tf::poseMsgToEigen(additive_traj.poses[i].pose, pose_affine);
    path.push_back(pose_affine);
    path_positions.push_back(pose_affine.translation());

    // Verify if the current pose and the previous pose are contiguous
    if (i != 0 && previous_i + 1 == i)
    {
      b_points.push_back(pose_affine.translation());
      a_points.push_back(previous_pose_affine.translation());

      // Define the path colors
      if (display_params.color_mode == 0)
      {
        // Display one color per layer level
        colors_vector.push_back(intToColor(additive_traj.poses[i].layer_level));

      }
      else if (display_params.color_mode == 1)
      {
        // Display one color per layer index
        colors_vector.push_back(intToColor(additive_traj.poses[i].layer_index));
      }
      else if (display_params.color_mode == 2)
      {
        // Display one color per speed
        if (color_map.find(additive_traj.poses[i].params.speed) == color_map.end()) // Cannot find color
          color_map[additive_traj.poses[i].params.speed] = intToColor(color++);
        colors_vector.push_back(color_map[additive_traj.poses[i].params.speed]);
      }
      else if (display_params.color_mode == 3)
      {
        if (additive_traj.poses[i].entry_pose == true)
          colors_vector.push_back(intToColor(0));
        else if (additive_traj.poses[i].exit_pose == true)
          colors_vector.push_back(intToColor(1));
        else if (additive_traj.poses[i].polygon_start == true)
          colors_vector.push_back(intToColor(2));
        else if (additive_traj.poses[i].polygon_end == true)
          colors_vector.push_back(intToColor(3));
        else
          colors_vector.push_back(intToColor(4));
      }
      else
      {
        // Display one color per laser power
        if (color_map.find(additive_traj.poses[i].params.laser_power) == color_map.end()) // Cannot find color
          color_map[additive_traj.poses[i].params.laser_power] = intToColor(color++);
        colors_vector.push_back(color_map[additive_traj.poses[i].params.laser_power]);
      }

    }
    previous_i = i;

    // Display labels
    if (display_params.display_labels)
    {
      double z_text_offset = 1e-4;
      if (display_params.display_mode == 1 || display_params.display_mode == 3)
        z_text_offset += (display_params.cylinder_size / 2);
      else
        z_text_offset += (display_params.wire_size / 2);

      Eigen::Affine3d upper_pose(pose_affine);
      upper_pose.translate(Eigen::Vector3d(0, 0, z_text_offset));

      // Pose label is close to another pose, change color and translate
      double label_tol = (0.5e-3);
      rviz_visual_tools::colors label_color = rviz_visual_tools::colors::WHITE;
      if (closeToPose(i, label_tol))
      {
        label_color = rviz_visual_tools::colors::DARK_GREY;
        upper_pose.translate(Eigen::Vector3d(0, 0, label_tol / 2.0));
      }

      geometry_msgs::Vector3 scale_vector;
      scale_vector.x = display_params.labels_size;
      scale_vector.y = display_params.labels_size;
      scale_vector.z = display_params.labels_size;

      if (display_params.label_type == 0)
      {
        // Pose number
        rvt_trajectory->publishText(upper_pose, std::string(std::to_string(i)), label_color, scale_vector, false);
      }
      else if (display_params.label_type == 1)
      {
        // Layer level
        rvt_trajectory->publishText(upper_pose, std::string(std::to_string(additive_traj.poses[i].layer_level)),
                                    label_color,
                                    scale_vector, false);
      }
      else if (display_params.label_type == 2)
      {
        // Layer index
        rvt_trajectory->publishText(upper_pose, std::string(std::to_string(additive_traj.poses[i].layer_index)),
                                    label_color,
                                    scale_vector, false);
      }
      else // Pose number within layer
      {
        rvt_trajectory->publishText(
            upper_pose,
            std::string(std::to_string(pose_index_within_layer_count[additive_traj.poses[i].layer_level]++)),
            label_color, scale_vector, false);
      }
    }
  }

  // Transform pose orientation to take account of tool orientation
  // Check that service exists
  if (!get_tool_client.waitForExistence(ros::Duration(0.5)))
    return "Cannot get tool, service does not exist";

  // Call service to get start pose
  ram_utils::GetTool srv;
  if (!get_tool_client.call(srv))
    return "Cannot get tool, call to service failed";

  Eigen::Affine3d tool;
  tf::poseMsgToEigen(srv.response.pose, tool);
  for (auto &pose : path)
    pose = pose * tool;

  // Display modes
  if (display_params.display_mode == 0 || display_params.display_mode == 2) // Wire frame mode and wire + axis mode
  {

    std::vector<geometry_msgs::Point> a_points_geom, b_points_geom;
    for (Eigen::Vector3d point : a_points)
    {
      geometry_msgs::Point p;
      p.x = point[0];
      p.y = point[1];
      p.z = point[2];
      a_points_geom.push_back(p);
    }
    for (Eigen::Vector3d point : b_points)
    {
      geometry_msgs::Point p;
      p.x = point[0];
      p.y = point[1];
      p.z = point[2];
      b_points_geom.push_back(p);
    }

    geometry_msgs::Vector3 scale_vector;
    scale_vector.x = display_params.wire_size;
    scale_vector.y = display_params.wire_size;
    scale_vector.z = display_params.wire_size;

    rvt_trajectory->publishLines(a_points_geom, b_points_geom, colors_vector, scale_vector);

    if (display_params.display_mode == 2)
      rvt_trajectory->publishAxisPath(path, display_params.axis_size, 0.1 * display_params.axis_size);
  }

  if (display_params.display_mode == 1 || display_params.display_mode == 3) // Cylinders mode and cylinders + axis mode
  {
    // rvt_trajectory->publishPath(path_positions, colors_vector, display_params.cylinder_size);
    publishCylinders(a_points, b_points, colors_vector, display_params.cylinder_size);
    if (display_params.display_mode == 3)
      rvt_trajectory->publishAxisPath(path, display_params.axis_size, 0.1 * display_params.axis_size);
  }
  rvt_trajectory->trigger();
  updateSelection();
  return "";
}

void trajectoryCallback(const ram_msgs::AdditiveManufacturingTrajectory::ConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(trajectory_mutex);
  additive_traj = *msg;
  if (!displayTrajectory().empty())
  {
    display_params.display_range_of_layers = false;
    displayTrajectory();
  }
}

bool displayTrajectoryCallback(ram_display::DisplayTrajectory::Request &req,
                               ram_display::DisplayTrajectory::Response &res)
{
  std::lock_guard<std::recursive_mutex> lock(display_params_mutex);
  display_params = req;
  res.error = displayTrajectory();
  return true;
}

bool deleteTrajectoryCallback(ram_display::DeleteTrajectory::Request &,
                              ram_display::DeleteTrajectory::Response &)
{
  deleteTrajectory();
  return true;
}

bool updateSelection(const bool display)
{
  std::lock_guard<std::recursive_mutex> lock_1(display_params_mutex);
  std::lock_guard<std::recursive_mutex> lock_2(selection_params_mutex);

  rvt_selection->deleteAllMarkers();
  rvt_selection->trigger();

  if (!display)
    return true;

  std_msgs::ColorRGBA color;
  color.r = 0.9;
  color.g = 0.9;
  color.b = 0.9;
  if (selection.temporary)
    color.a = 0.5;
  else
    color.a = 1.0;

  geometry_msgs::Vector3 size;
  // Cylinders
  if (display_params.display_mode == 1 || display_params.display_mode == 3)
  {
    size.x = display_params.cylinder_size * 1.4;
    size.y = size.x;
    size.z = size.x;
  }
  else // Wire
  {
    size.x = display_params.wire_size / 600.0;
    size.y = size.x;
    size.z = size.x;
  }

  for (auto ram_pose : selection.selected_poses)
  {
    geometry_msgs::Pose geometry_pose;
    geometry_pose.position = ram_pose.pose.position;
    geometry_pose.orientation.w = 1;
    rvt_selection->publishSphere(geometry_pose, color, size);
  }

  rvt_selection->trigger();
  return true;
}

bool updateSelectionCallback(ram_display::UpdateSelection::Request &req,
                             ram_display::UpdateSelection::Response &)
{
  std::lock_guard<std::recursive_mutex> lock_1(display_params_mutex);
  std::lock_guard<std::recursive_mutex> lock_2(selection_params_mutex);
  selection = req;
  return updateSelection();
}

bool updateMeshColorCallback(ram_display::UpdateMeshColor::Request &req,
                             ram_display::UpdateMeshColor::Response &)
{
  std::lock_guard<std::recursive_mutex> lock_1(display_params_mutex);

  display_params.mesh_color = req.color;

  displayMesh(display_params.mesh_color,
              trajectory_frame,
              additive_traj.file);
  return true;
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "ram_display");
  ros::NodeHandle nh;
  nh.param<std::string>("ram/trajectory_frame", trajectory_frame, "base");
  std::string trajectory_marker_topic, selection_marker_topic;
  nh.param<std::string>("ram/display/trajectory_marker_topic", trajectory_marker_topic, "/rvt_trajectory");
  nh.param<std::string>("ram/display/selection_marker_topic", selection_marker_topic, "/rvt_selection");
  get_tool_client = nh.serviceClient<ram_utils::GetTool>("ram/get_tool");

  // Default parameters
  display_params.display_mode = 1;
  display_params.cylinder_size = 0.001;
  display_params.display_labels = false;
  display_params.wire_size = 3;
  display_params.labels_size = 1;
  display_params.mesh_color.r = 0.7;
  display_params.mesh_color.g = 0.7;
  display_params.mesh_color.b = 0.7;
  display_params.mesh_color.a = 1;

  mesh_pub = nh.advertise<visualization_msgs::Marker>("ram/display/mesh", 1, true);

  // Allow the action server to receive and send ROS messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  rvt_trajectory = std::make_shared<rviz_visual_tools::RvizVisualTools>(trajectory_frame, trajectory_marker_topic);
  rvt_selection = std::make_shared<rviz_visual_tools::RvizVisualTools>(trajectory_frame, selection_marker_topic);

  // Dynamic markers
  rvt_trajectory->enableFrameLocking();
  rvt_selection->enableFrameLocking();

  // Latch publish
  rvt_trajectory->loadMarkerPub(false, true);
  rvt_selection->loadMarkerPub(false, true);

  ros::Subscriber sub = nh.subscribe("ram/trajectory", 5, trajectoryCallback);

  ros::ServiceServer service_1 = nh.advertiseService("ram/display/add_trajectory", displayTrajectoryCallback);
  ros::ServiceServer service_2 = nh.advertiseService("ram/display/delete_trajectory", deleteTrajectoryCallback);
  ros::ServiceServer service_3 = nh.advertiseService("ram/display/update_selection", updateSelectionCallback);
  ros::ServiceServer service_4 = nh.advertiseService("ram/display/update_mesh_color", updateMeshColorCallback);

  ros::waitForShutdown();
  spinner.stop();
  return 0;
}

