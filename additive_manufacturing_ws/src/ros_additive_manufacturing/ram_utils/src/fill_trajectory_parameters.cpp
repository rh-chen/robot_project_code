#include <mutex>

#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_utils/GetFillParameters.h>
#include <ros/ros.h>

#include <ram_utils/AddEntryExitStrategies.h>

std::mutex params_mutex;
ram_msgs::AdditiveManufacturingParams params;
std::mutex trajectory_mutex;
ram_msgs::AdditiveManufacturingTrajectory trajectory;

ros::ServiceClient entry_exit_strategies_client;
ros::Publisher pub_trajectory;

void publishModifiedTrajectory()
{
  if (trajectory.poses.empty())
    return;

  for (auto &pose : trajectory.poses)
    pose.params = params;

  // Add exit and entry Strategies
  ram_utils::AddEntryExitStrategies srv;
  srv.request.poses = trajectory.poses;
  bool success = entry_exit_strategies_client.call(srv);
  if (success)
    trajectory.poses = srv.response.poses;

  // Now that the trajectory is complete, we set the time stamp to now and publish
  trajectory.generated = ros::Time::now();
  trajectory.modified = trajectory.generated;
  pub_trajectory.publish(trajectory);
}

void updateParams(const ram_msgs::AdditiveManufacturingParams::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock_1(params_mutex);
  std::lock_guard<std::mutex> lock_2(trajectory_mutex);
  params = *msg;
}

void tempTrajReceived(const ram_msgs::AdditiveManufacturingTrajectory::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock_1(params_mutex);
  std::lock_guard<std::mutex> lock_2(trajectory_mutex);
  trajectory = *msg;
  publishModifiedTrajectory();
}

bool getFillParams(ram_utils::GetFillParametersRequest &, ram_utils::GetFillParametersResponse &res)
{
  std::lock_guard<std::mutex> lock(params_mutex);
  res.parameters = params;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fill_trajectory_parameters");
  ros::NodeHandle nh;

  // Default parameters
  params.approach_type = 1;
  params.laser_power = 4000;
  params.feed_rate = 2.0 / 60.0; // Wire - 2 meters / minute
  params.movement_type = 1;
  params.blend_radius = 0;
  params.speed = 0.01; // meters / second

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Subscribe to update internal parameters
  ros::Subscriber sub_params = nh.subscribe("ram/fill_trajectory/parameters", 5, updateParams);

  // Subscribe to receive trajectory from path_planning
  ros::Subscriber sub_traj = nh.subscribe("ram/trajectory_tmp", 5, tempTrajReceived);

  // Publish "ram/trajectory" to every node that listens
  pub_trajectory = nh.advertise<ram_msgs::AdditiveManufacturingTrajectory>("ram/trajectory", 5, true);

  // Service to add strategies
  entry_exit_strategies_client = nh.serviceClient<ram_utils::AddEntryExitStrategies>(
      "ram/information/add_entry_exit_strategies");

// Get fill parameters stored internally
  ros::ServiceServer server = nh.advertiseService("ram/fill_trajectory/get_parameters", getFillParams);

  ros::waitForShutdown();
  spinner.stop();
  return 0;
}

