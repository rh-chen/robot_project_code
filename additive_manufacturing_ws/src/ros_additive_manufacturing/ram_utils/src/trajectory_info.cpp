#include <mutex>

#include <eigen_conversions/eigen_msg.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_msgs/AdditiveManufacturingTrajectoryInfo.h>
#include <ram_utils/GetLayerSize.h>
#include <ram_utils/GetNumberOfLayersLevels.h>
#include <ram_utils/GetTrajectoryInfos.h>
#include <ram_utils/GetTrajectorySize.h>
#include <ros/ros.h>

std::mutex trajectory_mutex;
ram_msgs::AdditiveManufacturingTrajectory layers;
ros::Publisher pub_traj_info;
std::mutex traj_info_mutex;
ram_msgs::AdditiveManufacturingTrajectoryInfo traj_info;

void trajReceived(const ram_msgs::AdditiveManufacturingTrajectory::Ptr& msg)
{
  std::lock_guard<std::mutex> lock_1(trajectory_mutex);
  std::lock_guard<std::mutex> lock_2(traj_info_mutex);

  layers = *msg;

  traj_info.file = msg->file;
  traj_info.generation_info = msg->generation_info;
  traj_info.generated = msg->generated;
  traj_info.modified = msg->modified;
  traj_info.similar_layers = msg->similar_layers;
  traj_info.number_of_poses = msg->poses.size();

  traj_info.number_of_layers_levels = 0;
  traj_info.number_of_layers_indices = 0;
  traj_info.trajectory_length = 0;
  traj_info.execution_time = 0;
  traj_info.wire_length = 0;
  if (msg->poses.empty())
  {
    pub_traj_info.publish(traj_info);
    return;
  }

  traj_info.number_of_layers_levels = msg->poses.back().layer_level + 1;
  traj_info.number_of_layers_indices = msg->poses.back().layer_index + 1;

  unsigned polygon_count(0);
  for (auto pose : msg->poses)
  {
    if (pose.polygon_start == true)
      ++polygon_count;
  }
  traj_info.number_of_polygons = polygon_count;

  Eigen::Vector3d last_position(msg->poses.front().pose.position.x,
                                msg->poses.front().pose.position.y,
                                msg->poses.front().pose.position.z);
  double last_speed(msg->poses.front().params.speed);
  double last_feed_rate(msg->poses.front().params.feed_rate);
  for (auto pose : msg->poses)
  {
    // Compute Euclidian distance
    Eigen::Vector3d position(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    Eigen::Vector3d difference(position - last_position);
    traj_info.trajectory_length += difference.norm(); // Result is in meters

    // Compute time to go from pose A to pose B - we assume a mean speed
    double speed(pose.params.speed);
    // Divide speed at pose by 2 if approach type is 'FINE': this is arbitrary to simulate
    // the fact that the robot will stop and then continue going
    if (pose.params.approach_type == 0)
      speed /= 2.0;

    const double mean_speed((last_speed + speed) / 2.0);
    const double segment_time(difference.norm() / mean_speed);
    traj_info.execution_time += segment_time; // Result is in seconds

    // Warning: This code is specific to the "feed_rate" unit!
    // Institut Maupertuis: Wire deposition is in meters/second
    double mean_feed_rate((last_feed_rate + pose.params.feed_rate) / 2.0);
    last_feed_rate = pose.params.feed_rate;
    traj_info.wire_length += segment_time * mean_feed_rate;

    last_position = position;
    last_speed = speed;
  }

  pub_traj_info.publish(traj_info);
}

bool getTrajectorySizeCallback(ram_utils::GetTrajectorySize::Request &,
                               ram_utils::GetTrajectorySize::Response &res)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex);
  res.trajectory_size = layers.poses.size();
  return true;
}

bool getNumberOfLayersLevelsCallback(ram_utils::GetNumberOfLayersLevels::Request &,
                                     ram_utils::GetNumberOfLayersLevels::Response &res)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex);

  if (layers.poses.empty())
    return false;

  // Even if a layer has been completely removed, the layer count will NOT change
  unsigned max_layer_level = 0;
  for (auto pose : layers.poses)
  {
    if (max_layer_level < pose.layer_level)
      max_layer_level = pose.layer_level;
  }

  res.number_of_layers = max_layer_level + 1;  // The level of the first layer is zero
  return true;
}

bool getLayerSizeCallback(ram_utils::GetLayerSize::Request &req,
                          ram_utils::GetLayerSize::Response &res)
{
  std::lock_guard<std::mutex> lock(trajectory_mutex);

  // Empty trajectory
  if (layers.poses.empty())
    return false;

  if (req.layer_level > layers.poses.back().layer_level)
    return false;

  unsigned pose_count = 0;
  for (auto &pose : layers.poses)
    if (pose.layer_level == req.layer_level)
      ++pose_count;

  res.layer_size = pose_count;

  return true;
}

bool getTrajectoryInfosCallback(ram_utils::GetTrajectoryInfos::Request &,
                               ram_utils::GetTrajectoryInfos::Response &res)
{
  std::lock_guard<std::mutex> lock(traj_info_mutex);
  res.informations = traj_info;
  return true;
}

int main(int argc,
         char** argv)
{
  ros::init(argc, argv, "ram_utils_trajectory_info");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Subscribe to trajectory topic
  ros::Subscriber traj = nh.subscribe("ram/trajectory", 10, trajReceived);

  // Publish trajectory info
  pub_traj_info = nh.advertise<ram_msgs::AdditiveManufacturingTrajectoryInfo>("ram/information/trajectory", 1, true);

  ros::ServiceServer service_1 = nh.advertiseService("ram/information/get_trajectory_size", getTrajectorySizeCallback);
  ros::ServiceServer service_2 = nh.advertiseService("ram/information/get_number_of_layers_levels",
                                                     getNumberOfLayersLevelsCallback);
  ros::ServiceServer service_3 = nh.advertiseService("ram/information/get_layer_size", getLayerSizeCallback);
  ros::ServiceServer service_4 = nh.advertiseService("ram/information/get_trajectory_infos", getTrajectoryInfosCallback);

  ros::waitForShutdown();
  spinner.stop();
  return 0;
}
