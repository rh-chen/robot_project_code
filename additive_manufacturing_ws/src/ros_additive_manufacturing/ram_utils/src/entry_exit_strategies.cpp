#include <mutex>
#include <math.h>

#include <eigen_conversions/eigen_msg.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ros/ros.h>

#include <ram_utils/AddEntryExitStrategies.h>
#include <ram_utils/EntryExitParameters.h>

#include <unique_id/unique_id.h>

ram_utils::EntryExitParameters entry_params;
ram_utils::EntryExitParameters exit_params;

std::mutex params_mutex;

bool entryParametersCallback(ram_utils::EntryExitParameters::Request &req,
                             ram_utils::EntryExitParameters::Response &)
{
  std::lock_guard<std::mutex> lock(params_mutex);

  entry_params.request.angle = req.angle;
  entry_params.request.distance = req.distance;
  entry_params.request.number_of_poses = req.number_of_poses;
  return true;
}

bool exitParametersCallback(ram_utils::EntryExitParameters::Request &req,
                            ram_utils::EntryExitParameters::Response &)
{
  std::lock_guard<std::mutex> lock(params_mutex);

  exit_params.request.angle = req.angle;
  exit_params.request.distance = req.distance;
  exit_params.request.number_of_poses = req.number_of_poses;
  return true;
}

bool entryStrategy(unsigned ref_pose_id,
                   std::vector<ram_msgs::AdditiveManufacturingPose> req_poses,
                   std::vector<ram_msgs::AdditiveManufacturingPose>&res_poses)
{
  Eigen::Vector3d ref_vector(Eigen::Vector3d::Identity());

  if (req_poses.size() != 1)
  {
    ref_vector.x() = req_poses[ref_pose_id].pose.position.x - req_poses[ref_pose_id + 1].pose.position.x;
    ref_vector.y() = req_poses[ref_pose_id].pose.position.y - req_poses[ref_pose_id + 1].pose.position.y;
    ref_vector.z() = req_poses[ref_pose_id].pose.position.z - req_poses[ref_pose_id + 1].pose.position.z;
  }

  double rho;
  double phi;
  double theta;

  ram_msgs::AdditiveManufacturingPose pose(req_poses[ref_pose_id]);
  pose.params.laser_power = 0;
  pose.params.feed_rate = 0;
  pose.entry_pose = true;
  pose.polygon_start = false;
  pose.polygon_end = false;

  phi = atan2(ref_vector.y(), ref_vector.x());
  theta = M_PI / 2 - entry_params.request.angle;

  for (unsigned j(entry_params.request.number_of_poses); j > 0; --j)
  {
    rho = entry_params.request.distance * j / entry_params.request.number_of_poses;

    ram_msgs::AdditiveManufacturingPose pose_tmp(pose);
    //Spherical to Cartesian coordinates
    pose_tmp.pose.position.x += rho * sin(theta) * cos(phi);
    pose_tmp.pose.position.y += rho * sin(theta) * sin(phi);
    pose_tmp.pose.position.z += rho * cos(theta);

    pose_tmp.unique_id = unique_id::toMsg(unique_id::fromRandom());
    res_poses.push_back(pose_tmp);
  }
  return true;
}

bool exitStrategy(unsigned ref_pose_id,
                  std::vector<ram_msgs::AdditiveManufacturingPose> req_poses,
                  std::vector<ram_msgs::AdditiveManufacturingPose>&res_poses)
{
  Eigen::Vector3d ref_vector(Eigen::Vector3d::Identity());

  // Make sure it is not the only pose
  // Make sure it is not the first pose
  if (req_poses.size() != 1 && ref_pose_id != 0)
  {
    ref_vector.x() = req_poses[ref_pose_id].pose.position.x - req_poses[ref_pose_id - 1].pose.position.x;
    ref_vector.y() = req_poses[ref_pose_id].pose.position.y - req_poses[ref_pose_id - 1].pose.position.y;
    ref_vector.z() = req_poses[ref_pose_id].pose.position.z - req_poses[ref_pose_id - 1].pose.position.z;
  }
  double rho;
  double phi;
  double theta;

  ram_msgs::AdditiveManufacturingPose pose(req_poses[ref_pose_id]);
  pose.params.laser_power = 0;
  pose.params.feed_rate = 0;
  pose.exit_pose = true;
  pose.polygon_start = false;
  pose.polygon_end = false;

  phi = atan2(ref_vector.y(), ref_vector.x());
  theta = M_PI / 2 - exit_params.request.angle;

  for (unsigned j(1); j <= exit_params.request.number_of_poses; ++j)
  {
    rho = exit_params.request.distance * j / exit_params.request.number_of_poses;

    ram_msgs::AdditiveManufacturingPose pose_tmp(pose);
    //Spherical to Cartesian coordinates
    pose_tmp.pose.position.x += rho * sin(theta) * cos(phi);
    pose_tmp.pose.position.y += rho * sin(theta) * sin(phi);
    pose_tmp.pose.position.z += rho * cos(theta);

    pose_tmp.unique_id = unique_id::toMsg(unique_id::fromRandom());
    res_poses.push_back(pose_tmp);
  }
  return true;
}

int poseInPolygon(std::vector<ram_msgs::AdditiveManufacturingPose> poses,
                  unsigned pose_id)
{
  int pose_in_polygon = 0;
  for (unsigned i(0); i < poses.size(); ++i)
  {
    if (i == pose_id)
      return pose_in_polygon;

    if (poses[i].polygon_start && poses[i].polygon_end)
      continue;

    if (poses[i].polygon_start)
      pose_in_polygon++;
    if (poses[i].polygon_end)
      pose_in_polygon--;
  }
  return -1;
}

bool addEntryExitStrategiesCallback(ram_utils::AddEntryExitStrategies::Request &req,
                                    ram_utils::AddEntryExitStrategies::Response &res)
{
  std::lock_guard<std::mutex> lock(params_mutex);

  for (unsigned i(0); i < req.poses.size(); ++i)
  {
    bool limit_between_polygons = false;
    bool add_entry_strategy = false;
    bool add_exit_strategy = false;

    // * Special case. limit between to polygons
    if (req.poses[i].polygon_start && req.poses[i].polygon_end)
    {
      int pose_in_polygon = poseInPolygon(req.poses, i);
      if (pose_in_polygon < 0 || pose_in_polygon > 1)
        return false;
      if (pose_in_polygon == 1) // limit between to polygons .there should be no strategies
      {
        limit_between_polygons = true;

        if (i != 0 && req.poses[i - 1].entry_pose == true)
          return false;
        if (i != (req.poses.size() - 1) && req.poses[i + 1].exit_pose == true)
          return false;
        //
      }
    }

    // find polygon_start
    if (req.poses[i].polygon_start)
    {
      // Add Entry Strategy
      if (i == 0) //First Element
        add_entry_strategy = true;
      if (i != 0 && req.poses[i - 1].entry_pose == false)
        add_entry_strategy = true;
    }

    // find polygon_end
    if (req.poses[i].polygon_end)
    {
      // Add Exit Pose
      if (i == (req.poses.size() - 1)) //Last Element
        add_exit_strategy = true;
      if (i != (req.poses.size() - 1) && req.poses[i + 1].exit_pose == false)
        add_exit_strategy = true;
    }

    // ADD STRATEGIES
    if (limit_between_polygons)
    {
      res.poses.push_back(req.poses[i]);
      res.poses.back().polygon_start = false;

      exitStrategy(i, req.poses, res.poses);
      add_exit_strategy = false;

      entryStrategy(i, req.poses, res.poses);
      add_entry_strategy = false;

      res.poses.push_back(req.poses[i]);
      res.poses.back().polygon_end = false;
      res.poses.back().unique_id = unique_id::toMsg(unique_id::fromRandom());
    }
    else
    {
      if (add_entry_strategy)
      {
        entryStrategy(i, req.poses, res.poses);
        add_entry_strategy = false;
      }

      res.poses.push_back(req.poses[i]);

      if (add_exit_strategy)
      {
        exitStrategy(i, req.poses, res.poses);
        add_exit_strategy = false;
      }

    }
  }

  return true;
}

int main(int argc,
         char** argv)
{
  ros::init(argc, argv, "entry_exit_strategies");
  ros::NodeHandle nh;

  entry_params.request.angle = M_PI / 4; // 45 degrees (phi in spherical coordinates)
  entry_params.request.distance = 0.05; // 5 cm
  entry_params.request.number_of_poses = 1;

  exit_params.request.angle = M_PI / 4; // 45 degrees (phi in spherical coordinates)
  exit_params.request.distance = 0.05; // 5 cm
  exit_params.request.number_of_poses = 1;

  ros::ServiceServer service_1 = nh.advertiseService("ram/information/add_entry_exit_strategies",
                                                     addEntryExitStrategiesCallback);

  ros::ServiceServer service_2 = nh.advertiseService("ram/information/entry_parameters", entryParametersCallback);
  ros::ServiceServer service_3 = nh.advertiseService("ram/information/exit_parameters", exitParametersCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
