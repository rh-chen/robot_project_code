#include <mutex>
#include <string>
#include <strings.h>

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <ram_modify_trajectory/AddPoses.h>
#include <ram_modify_trajectory/DeleteSelectedPoses.h>
#include <ram_modify_trajectory/ModifySelectedPoses.h>
#include <ram_modify_trajectory/ReflectSelectedPoses.h>
#include <ram_modify_trajectory/ResetSelectedPoses.h>
#include <ram_modify_trajectory/RotateSelectedPoses.h>
#include <ram_modify_trajectory/ScaleSelectedPoses.h>
#include <ram_modify_trajectory/ShiftPoses.h>
#include <ram_msgs/AdditiveManufacturingPose.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_utils/UnmodifiedTrajectory.h>
#include <ram_utils/AddEntryExitStrategies.h>

#include <unique_id/unique_id.h>
#include <uuid_msgs/UniqueID.h>

ros::Publisher trajectory_pub;
ros::ServiceClient unmodified_trajectory_client;
ros::ServiceClient entry_exit_strategies_client;

std::mutex layers_mutex;
ram_msgs::AdditiveManufacturingTrajectory layers;

// Functions

bool verifyPolygonLimits(ram_msgs::AdditiveManufacturingTrajectory trajectory)
{
  bool pose_in_polygon = false;

  for (auto current_pose : trajectory.poses)
  {
    if ((current_pose.polygon_start || current_pose.polygon_end) && (current_pose.entry_pose || current_pose.exit_pose))
      return false;

    if (current_pose.polygon_start && current_pose.polygon_end)
      continue;

    if (current_pose.polygon_start)
    {
      if (!pose_in_polygon)
        pose_in_polygon = true;
      else
        return false;
    }
    if (current_pose.polygon_end)
    {
      if (pose_in_polygon)
        pose_in_polygon = false;
      else
        return false;
    }
  }
  if (pose_in_polygon)
    return false;

  return true;
}

bool deletePoses(ram_msgs::AdditiveManufacturingTrajectory &trajectory,
                 std::vector<ram_msgs::AdditiveManufacturingPose> poses)
{
  for (unsigned i(0); i < poses.size(); ++i)
  {
    std::string selected_pose_uuid = unique_id::toHexString(poses[i].unique_id);
    bool pose_is_deleted = false;

    // Poses in the trajectory
    for (unsigned j(0); j < trajectory.poses.size(); ++j)
    {
      std::string current_uuid = unique_id::toHexString(trajectory.poses[j].unique_id);
      if (selected_pose_uuid.compare(current_uuid) != 0) // uuid are not equals
        continue;

      trajectory.poses.erase(trajectory.poses.begin() + j); // Delete pose
      pose_is_deleted = true;
      break;
    }

    if (!pose_is_deleted) // Selected pose is not in the trajectory
    {
      ROS_ERROR_STREAM("Pose to be deleted could not be found, UUID = " << selected_pose_uuid);
      return false;
    }
  }

  trajectory.similar_layers = false;
  return true;
}

bool findPose(std::string pose_uuid, std::vector<ram_msgs::AdditiveManufacturingPose> poses,
              ram_msgs::AdditiveManufacturingPose &pose)
{
  for (auto current_pose : poses)
  {
    std::string current_pose_uuid = unique_id::toHexString(current_pose.unique_id);
    if (pose_uuid.compare(current_pose_uuid) != 0) // uuid are not equals
      continue;
    // uuid are equals
    pose = current_pose;
    return true;
  }
  return false;
}

void saveTrajectoryCallback(const ram_msgs::AdditiveManufacturingTrajectoryConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  layers = *msg;
}

bool modifySelectedPosesCallback(ram_modify_trajectory::ModifySelectedPoses::Request &req,
                                 ram_modify_trajectory::ModifySelectedPoses::Response &)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  // Make a copy and modify this copy
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty())
  {
    ROS_ERROR_STREAM("Request is empty");
    return false;
  }

  if (trajectory.poses.empty())
  {
    ROS_ERROR_STREAM("Trajectory is empty");
    return false;
  }

  std::vector<ram_msgs::AdditiveManufacturingPose> strategies_to_delete; // Strategies to delete when a polygon_start or a polygon_end is modified

  std::vector<ram_msgs::AdditiveManufacturingParams::_movement_type_type> move_types;
  for (auto pose : req.poses)
    move_types.push_back(pose.params.movement_type);

  if (std::adjacent_find(move_types.begin(), move_types.end(), std::not_equal_to<int>()) != move_types.end())
  {
    // Cannot modify speed if not all the poses have the same motion type
    if (req.speed)
    {
      ROS_ERROR_STREAM("Cannot modify speed if not all the poses have the same motion type");
      return false;
    }
  }

  // Modify poses
  for (unsigned i(0); i < req.poses.size(); ++i)
  {
    std::string selected_pose_uuid = unique_id::toHexString(req.poses[i].unique_id);
    bool pose_in_trajectory = false;

    for (unsigned j(0); j < trajectory.poses.size(); ++j)
    {
      std::string current_pose_uuid = unique_id::toHexString(trajectory.poses[j].unique_id);

      if (current_pose_uuid.compare(selected_pose_uuid) != 0) // uuid are not equals
        continue;

      // uuid are equals. Modify this pose
      pose_in_trajectory = true;
      // Booleans
      //
      // Numerical value:
      // True = Absolute
      // False = Relative
      //
      // Non numerical value:
      // True = Modify
      // False = Keep value

      // Replace and check values
      if (req.pose)
        trajectory.poses[j].pose = req.pose_reference.pose;
      else
      {
        Eigen::Affine3d current_pose_eigen, reference_pose;
        tf::poseMsgToEigen(trajectory.poses[j].pose, current_pose_eigen);
        tf::poseMsgToEigen(req.pose_reference.pose, reference_pose);

        current_pose_eigen.matrix() *= reference_pose.matrix();
        tf::poseEigenToMsg(current_pose_eigen, trajectory.poses[j].pose);
      }

      // Non numerical values
      if (req.polygon_start)
      {
        if (trajectory.poses[j].polygon_start && !req.pose_reference.polygon_start)
        {
          // Save the entry strategy in the vector
          for (int strategy_id(j - 1); strategy_id >= 0; --strategy_id)
          {
            if (trajectory.poses[strategy_id].entry_pose)
              strategies_to_delete.push_back(trajectory.poses[strategy_id]);
            else
              break;
          }
        }

        trajectory.poses[j].polygon_start = req.pose_reference.polygon_start;
      }

      if (req.polygon_end)
      {
        // Save the exit strategy in the vector
        if (trajectory.poses[j].polygon_end && !req.pose_reference.polygon_end)
        {
          for (unsigned strategy_id(j + 1); strategy_id < trajectory.poses.size(); ++strategy_id)
          {
            if (trajectory.poses[strategy_id].exit_pose)
              strategies_to_delete.push_back(trajectory.poses[strategy_id]);
            else
              break;
          }
        }
        trajectory.poses[j].polygon_end = req.pose_reference.polygon_end;
      }

      if (req.movement_type)
        trajectory.poses[j].params.movement_type = req.pose_reference.params.movement_type;

      if (req.approach_type)
        trajectory.poses[j].params.approach_type = req.pose_reference.params.approach_type;

      // Numerical values
      if (req.blend_radius)
        trajectory.poses[j].params.blend_radius = req.pose_reference.params.blend_radius;
      else
      {
        trajectory.poses[j].params.blend_radius += req.pose_reference.params.blend_radius;

        if (trajectory.poses[j].params.blend_radius < 0)
        {
          ROS_ERROR_STREAM("Blend radius cannot be < 0");
          return false;
        }

        if (trajectory.poses[j].params.blend_radius > 100)
        {
          ROS_ERROR_STREAM("Blend radius cannot be > 100");
          return false;
        }
      }

      if (req.speed)
        trajectory.poses[j].params.speed = req.pose_reference.params.speed;
      else
      {
        trajectory.poses[j].params.speed += req.pose_reference.params.speed;

        if (trajectory.poses[j].params.speed < 1e-12) // Cannot be zero
        {
          ROS_ERROR_STREAM("Robot speed cannot be < 1e-12");
          return false;
        }
      }

      if (req.laser_power)
        trajectory.poses[j].params.laser_power = req.pose_reference.params.laser_power;
      else
      {
        trajectory.poses[j].params.laser_power += req.pose_reference.params.laser_power;

        if (trajectory.poses[j].params.laser_power < 0)
        {
          ROS_ERROR_STREAM("Laser power cannot be < 0");
          return false;
        }
      }

      if (req.feed_rate)
        trajectory.poses[j].params.feed_rate = req.pose_reference.params.feed_rate;
      else
      {
        trajectory.poses[j].params.feed_rate += req.pose_reference.params.feed_rate;

        if (trajectory.poses[j].params.feed_rate < 0)
        {
          ROS_ERROR_STREAM("Feed rate cannot be < 0");
          return false;
        }
      }

      break; // Exit to for loop
    }
    if (!pose_in_trajectory)
    {
      ROS_ERROR_STREAM("There are pose in the request that are not in the trajectory");
      return false;
    }
  }

  // Delete the strategies
  if (!deletePoses(trajectory, strategies_to_delete))
  {
    ROS_ERROR_STREAM("Errer deleting strategies_to_delete");
    return false;
  }

  // Verify polygon_start and polygon_end
  if (!verifyPolygonLimits(trajectory))
  {
    ROS_ERROR_STREAM("Error in verifyPolygonLimits");
    return false;
  }

  // Add exit and entry Strategies
  ram_utils::AddEntryExitStrategies srv;
  srv.request.poses = trajectory.poses;
  bool success = entry_exit_strategies_client.call(srv);
  if (success)
    trajectory.poses = srv.response.poses;

  // Publish trajectory
  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);
  return true;
}

// Reset the poses in the trajectory with the poses in the unmodified trajectory.
// the service request contains a list of uuid
bool resetSelectedPosesCallback(ram_modify_trajectory::ResetSelectedPoses::Request &req,
                                ram_modify_trajectory::ResetSelectedPoses::Response &)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  // Make a copy and modify this copy
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty()) // Request is empty
    return false;

  if (trajectory.poses.empty()) // Trajectory is empty
    return false;

  //Call buffer service
  ram_utils::UnmodifiedTrajectory unmodified_trajectory;
  unmodified_trajectory.request.generated = trajectory.generated;
  if (!unmodified_trajectory_client.call(unmodified_trajectory))
    return false;

  // Reset poses
  // - Poses in the current layer added after the generation
  // - Poses in the strategies to delete
  std::vector<ram_msgs::AdditiveManufacturingPose> poses_to_delete;

  for (unsigned i(0); i < req.poses.size(); ++i)
  {
    std::string selected_uuid = unique_id::toHexString(req.poses[i].unique_id);
    bool pose_in_current_traj = false;
    for (unsigned j(0); j < trajectory.poses.size(); ++j)
    {
      std::string current_pose_uuid = unique_id::toHexString(trajectory.poses[j].unique_id);

      if (current_pose_uuid.compare(selected_uuid) != 0) // uuid are not equals
        continue;

      // uuid are equals.
      pose_in_current_traj = true;
      //Find pose in unmodified trajectory
      ram_msgs::AdditiveManufacturingPose unmodified_pose;
      ram_msgs::AdditiveManufacturingPose p;

      if (findPose(current_pose_uuid, unmodified_trajectory.response.trajectory.poses, unmodified_pose))
      {
        // Save the entry strategy in the vector
        if (trajectory.poses[j].polygon_start && !unmodified_pose.polygon_start)
        {
          // Save the entry strategy in the vector
          for (int strategy_id(j - 1); strategy_id >= 0; --strategy_id)
          {
            if (trajectory.poses[strategy_id].entry_pose)
            {
              std::string strategy_uuid = unique_id::toHexString(trajectory.poses[strategy_id].unique_id);
              if (!findPose(current_pose_uuid, poses_to_delete, p))
                poses_to_delete.push_back(trajectory.poses[strategy_id]);
            }
            else
              break;
          }
        }
        // Save the exit strategy in the vector
        for (unsigned strategy_id(j + 1); strategy_id < trajectory.poses.size(); ++strategy_id)
        {
          if (trajectory.poses[strategy_id].exit_pose)
          {
            std::string strategy_uuid = unique_id::toHexString(trajectory.poses[strategy_id].unique_id);
            if (!findPose(current_pose_uuid, poses_to_delete, p))
              poses_to_delete.push_back(trajectory.poses[strategy_id]);
          }
          else
            break;
        }
        //update pose value
        trajectory.poses[j] = unmodified_pose;
      }
      else
      {
        if (!findPose(current_pose_uuid, poses_to_delete, p))
          poses_to_delete.push_back(trajectory.poses[j]); // Pose in request service does not exist in unmodified trajectory
      }

      break;
    }
    if (!pose_in_current_traj) // Pose in the request that are not in the current trajectory
      return false;
  }
  //Delete poses
  if (!deletePoses(trajectory, poses_to_delete))
    return false;
//Verify polygon_start and polygon_end
  if (!verifyPolygonLimits(trajectory))
    return false;

  // Add exit and entry Strategies
  ram_utils::AddEntryExitStrategies srv;
  srv.request.poses = trajectory.poses;
  bool success = entry_exit_strategies_client.call(srv);
  if (success)
    trajectory.poses = srv.response.poses;

// Publish trajectory
  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);

  return true;
}

// Add a new pose before each pose in the request
bool addPosesCallback(ram_modify_trajectory::AddPoses::Request &req, ram_modify_trajectory::AddPoses::Response &)
{
  std::lock_guard<std::mutex> lock(layers_mutex);

  // Make a copy and modify this copy
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty()) // Request is empty
    return false;

  if (trajectory.poses.empty())
    return false;

  // Find the selected pose
  for (unsigned i(0); i < req.poses.size(); ++i)
  {
    std::string selected_uuid = unique_id::toHexString(req.poses[i].unique_id);

    bool pose_in_trajectory = false;
    for (unsigned j(0); j < trajectory.poses.size(); ++j)
    {
      std::string layers_uuid = unique_id::toHexString(trajectory.poses[j].unique_id);
      if (selected_uuid.compare(layers_uuid) != 0) // uuid are not equals
        continue;

      // uuid are equals. Add new pose
      pose_in_trajectory = true;
      ram_msgs::AdditiveManufacturingPose new_pose = trajectory.poses[j];

      new_pose.unique_id = unique_id::toMsg(unique_id::fromRandom());

      new_pose.polygon_start = false;
      new_pose.polygon_end = false;

      // Exit strategy
      if (trajectory.poses[j].polygon_end)
        trajectory.poses[j].exit_pose = true;

      if ((j + 1) == trajectory.poses.size()) // Last element in the trajectory
      {
        trajectory.poses.push_back(new_pose); // Insert new pose
      }
      else
      {
        // Modify position values
        new_pose.pose.position.x = (trajectory.poses[j].pose.position.x + trajectory.poses[j + 1].pose.position.x) / 2;
        new_pose.pose.position.y = (trajectory.poses[j].pose.position.y + trajectory.poses[j + 1].pose.position.y) / 2;
        new_pose.pose.position.z = (trajectory.poses[j].pose.position.z + trajectory.poses[j + 1].pose.position.z) / 2;

        trajectory.poses.insert(trajectory.poses.begin() + j + 1, new_pose); // Insert new pose
      }
      break;
    }
    if (!pose_in_trajectory) // Pose in the request that are not in the trajectory
      return false;
  }

  trajectory.similar_layers = false;
  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);
  return true;
}

bool deleteSelectedPosesCallback(ram_modify_trajectory::DeleteSelectedPoses::Request &req,
                                 ram_modify_trajectory::DeleteSelectedPoses::Response &)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  // Make a copy and modify this copy
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty()) // Request is empty
    return false;

  if (trajectory.poses.empty())
    return false;

  std::vector<ram_msgs::AdditiveManufacturingPose> strategies_to_delete; // Strategies to delete when a polygon_start or a polygon_end is modified

  // Delete poses
  for (unsigned i(0); i < req.poses.size(); ++i)
  {
    std::string selected_pose_uuid = unique_id::toHexString(req.poses[i].unique_id);
    bool pose_is_deleted = false;

    // Poses in the trajectory
    for (unsigned j(0); j < trajectory.poses.size(); ++j)
    {
      std::string current_uuid = unique_id::toHexString(trajectory.poses[j].unique_id);
      if (selected_pose_uuid.compare(current_uuid) != 0) // uuid are not equals
        continue;

      // uuid are equals. Delete pose
      ram_msgs::AdditiveManufacturingPose p;

      if (trajectory.poses[j].polygon_start)
      {
        // We also want to delete the entry strategy
        for (int strategy_id(j - 1); strategy_id >= 0; --strategy_id)
        {
          if (trajectory.poses[strategy_id].entry_pose)
          {
            std::string strategy_uuid = unique_id::toHexString(trajectory.poses[strategy_id].unique_id);
            // Do not add the pose if it is already in the req.poses vector: it will be deleted
            // This avoids duplicates between the strategies_to_delete vector and req.poses
            if (!findPose(strategy_uuid, req.poses, p))
              if (!findPose(strategy_uuid, strategies_to_delete, p)) // Avoid duplicates in the strategies to delete
                strategies_to_delete.push_back(trajectory.poses[strategy_id]);
          }

          else
            break;
        }
      }

      if (trajectory.poses[j].polygon_end)
      {
        // We also want to delete the exit strategy
        for (unsigned strategy_id(j + 1); strategy_id < trajectory.poses.size(); ++strategy_id)
        {
          if (trajectory.poses[strategy_id].exit_pose)
          {
            std::string strategy_uuid = unique_id::toHexString(trajectory.poses[strategy_id].unique_id);
            // Do not add the pose if it is already in the req.poses vector: it will be deleted
            // This avoids duplicates between the strategies_to_delete vector and req.poses
            if (!findPose(strategy_uuid, req.poses, p))
              if (!findPose(strategy_uuid, strategies_to_delete, p)) // Avoid duplicates in the strategies to delete
                strategies_to_delete.push_back(trajectory.poses[strategy_id]);
          }
          else
            break;
        }
      }

      if (trajectory.poses[j].polygon_start && !trajectory.poses[j].polygon_end) // Verify polygon_start value
      {
        if ((j + 1) != trajectory.poses.size()) // Is not the last pose
          trajectory.poses[j + 1].polygon_start = true; // The next pose is the new polygon_start
      }
      else if (trajectory.poses[j].polygon_end && !trajectory.poses[j].polygon_start) // Verify polygon_end value
      {
        if (j != 0) // Is not the first pose
          trajectory.poses[j - 1].polygon_end = true; // The previous pose is the new polygon_end
      }
      // If pose is a polygon start AND a polygon end, do nothing

      trajectory.poses.erase(trajectory.poses.begin() + j); // Delete pose
      pose_is_deleted = true;
      break;
    }

    if (!pose_is_deleted) // Selected pose are not in the trajectory
      return false;
  }

  if (trajectory.poses.empty())
    return false;

  // Delete poses
  if (!deletePoses(trajectory, strategies_to_delete))
    return false;

  if (trajectory.poses.empty())
    return false;

  bool trajectory_ok(false);
  for (auto pose : trajectory.poses)
  {
    if (!pose.entry_pose && !pose.exit_pose)
    {
      trajectory_ok = true;
      break;
    }
  }

  if (!trajectory_ok)
    return false;

  // Verify polygon_start and polygon_end
  if (!verifyPolygonLimits(trajectory))
    return false;

  // Add exit and entry Strategies
  ram_utils::AddEntryExitStrategies srv;
  srv.request.poses = trajectory.poses;
  bool success = entry_exit_strategies_client.call(srv);
  if (success)
    trajectory.poses = srv.response.poses;

  // Publish trajectory
  trajectory.similar_layers = false;
  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);

  return true;
}

bool rotateSelectedPosesCallback(ram_modify_trajectory::RotateSelectedPoses::Request &req,
                                 ram_modify_trajectory::RotateSelectedPoses::Response &)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  if (req.poses.empty()) // Request is empty
    return false;

  if (layers.poses.empty())
    return false;

  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);
  // transform center of rotation
  Eigen::Vector3d center_affine;
  tf::pointMsgToEigen(req.center_of_rotation, center_affine);

  std::vector<ram_msgs::AdditiveManufacturingPose> selected_poses(req.poses);

  //Create transformation
  //tf: trajectory frame
  //cr: center of rotation
  // P'_tf = [T_tf_cr * R(z,angle) * T_cr_tf]*P_tf
  Eigen::Affine3d translation_tf_cr(Eigen::Affine3d::Identity());
  translation_tf_cr.translate(center_affine);

  Eigen::Affine3d rot_z(Eigen::Affine3d::Identity());
  rot_z.rotate(Eigen::AngleAxisd(req.rotation_angle, Eigen::Vector3d::UnitZ()));

  Eigen::Affine3d transformation(translation_tf_cr * rot_z * translation_tf_cr.inverse());

  //Rotate selection
  for (auto &selected_pose : req.poses)
  {
    std::string selected_uuid = unique_id::toHexString(selected_pose.unique_id);
    bool pose_in_trajectory = false;

    for (auto &current_pose : trajectory.poses)
    {

      std::string current_uuid = unique_id::toHexString(current_pose.unique_id);

      if (current_uuid.compare(selected_uuid) != 0) // uuid are not equals
        continue;

      // uuid are equals. Update geometry pose
      pose_in_trajectory = true;
      Eigen::Affine3d current_pose_affine;
      tf::poseMsgToEigen(current_pose.pose, current_pose_affine);

      Eigen::Affine3d new_pose = Eigen::Affine3d::Identity();
      new_pose = transformation * current_pose_affine;

      geometry_msgs::Pose new_pose_msg;
      tf::poseEigenToMsg(new_pose, new_pose_msg);
      current_pose.pose = new_pose_msg; // Update current ram pose

      break;
    }
    if (!pose_in_trajectory) // Pose in the request that are not in the trajectory
      return false;
  }

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);

  return true;

}

bool reflectSelectedPosesCallback(ram_modify_trajectory::ReflectSelectedPoses::Request &req,
                                  ram_modify_trajectory::ReflectSelectedPoses::Response &)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty())
  {
    ROS_ERROR_STREAM("Request pose selection is empty");
    return false;
  }

  if (trajectory.poses.empty())
  {
    ROS_ERROR_STREAM("Trajectory is empty");
    return false;
  }

  // Transform center of rotation
  Eigen::Vector3d p_plane;
  tf::pointMsgToEigen(req.point_on_plane, p_plane);

  Eigen::Vector3d n_plane;
  tf::vectorMsgToEigen(req.normal_vector, n_plane);
  n_plane.normalize();

  // Create transformation
  // o: origin
  Eigen::Affine3d trans_o_plane(Eigen::Affine3d::Identity());
  trans_o_plane.translate(p_plane);

  // Householder transformation (Using to reflection across a plane)
  Eigen::Affine3d householder(Eigen::Affine3d::Identity());
  householder.matrix().topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() - 2 * n_plane * n_plane.transpose();

  Eigen::Affine3d reflect(trans_o_plane * householder * trans_o_plane.inverse());

  // Reflect selection
  for (auto &selected_pose : req.poses)
  {
    std::string selected_uuid = unique_id::toHexString(selected_pose.unique_id);
    bool pose_in_trajectory = false;

    for (auto &current_pose : trajectory.poses)
    {

      std::string current_uuid = unique_id::toHexString(current_pose.unique_id);

      if (current_uuid.compare(selected_uuid) != 0) // uuid are not equals
        continue;

      // uuid are equals. Update geometry pose
      pose_in_trajectory = true;

      Eigen::Affine3d current_pose_affine;
      tf::poseMsgToEigen(current_pose.pose, current_pose_affine);

      //  Reflect the pose position
      Eigen::Affine3d new_pose = current_pose_affine;
      new_pose.translation() = reflect * current_pose_affine.translation();

      geometry_msgs::Pose new_pose_msg;
      tf::poseEigenToMsg(new_pose, new_pose_msg);
      current_pose.pose = new_pose_msg; // Update current ram pose

      break;
    }
    if (!pose_in_trajectory) // Pose in the request that are not in the trajectory
    {
      ROS_ERROR_STREAM("Some poses in the request are not in the trajectory");
      return false;
    }
  }

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);

  return true;
}

bool scaleSelectedPosesCallback(ram_modify_trajectory::ScaleSelectedPoses::Request &req,
                                ram_modify_trajectory::ScaleSelectedPoses::Response &)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty())
  {
    ROS_ERROR_STREAM("Request pose selection is empty");
    return false;
  }

  if (trajectory.poses.empty())
  {
    ROS_ERROR_STREAM("Trajectory is empty");
    return false;
  }

  // Transform center of rotation
  Eigen::Vector3d center_of_scaling;
  tf::pointMsgToEigen(req.center_of_scaling, center_of_scaling);

  // Create transformation
  // o: origin
  Eigen::Affine3d translation(Eigen::Affine3d::Identity());
  translation.translate(center_of_scaling);

  Eigen::Affine3d scaling(Eigen::Affine3d::Identity());
  scaling.scale(Eigen::Vector3d(req.scale_factor, req.scale_factor, 1));

  Eigen::Affine3d transformation(translation * scaling * translation.inverse());
  // Scale selection
  for (auto &selected_pose : req.poses)
  {
    std::string selected_uuid = unique_id::toHexString(selected_pose.unique_id);
    bool pose_in_trajectory = false;

    for (auto &current_pose : trajectory.poses)
    {

      std::string current_uuid = unique_id::toHexString(current_pose.unique_id);

      if (current_uuid.compare(selected_uuid) != 0) // uuid are not equals
        continue;

      // uuid are equals. Update geometry pose
      pose_in_trajectory = true;

      Eigen::Affine3d current_pose_affine;
      tf::poseMsgToEigen(current_pose.pose, current_pose_affine);

      // Scale the pose position
      Eigen::Affine3d new_pose = current_pose_affine;
      new_pose.translation() = transformation * current_pose_affine.translation();

      geometry_msgs::Pose new_pose_msg;
      tf::poseEigenToMsg(new_pose, new_pose_msg);
      current_pose.pose = new_pose_msg; // Update current ram pose

      break;
    }
    if (!pose_in_trajectory) // Pose in the request that are not in the trajectory
    {
      ROS_ERROR_STREAM("Some poses in the request are not in the trajectory");
      return false;
    }
  }

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);

  return true;
}

bool shiftPosesCallback(ram_modify_trajectory::ShiftPoses::Request &req,
                        ram_modify_trajectory::ShiftPoses::Response &)
{
  std::lock_guard<std::mutex> lock(layers_mutex);
  ram_msgs::AdditiveManufacturingTrajectory trajectory(layers);

  if (req.poses.empty())
  {
    ROS_ERROR_STREAM("Request pose selection is empty");
    return false;
  }

  if (trajectory.poses.empty())
  {
    ROS_ERROR_STREAM("Trajectory is empty");
    return false;
  }

  for (auto &selected_pose : req.poses)
  {
    const std::string selected_uuid = unique_id::toHexString(selected_pose.unique_id);
    bool pose_in_trajectory = false;

    for (auto &current_pose : trajectory.poses)
    {
      const std::string current_uuid = unique_id::toHexString(current_pose.unique_id);
      if (current_uuid.compare(selected_uuid) != 0) // uuid are not equals
        continue;

      pose_in_trajectory = true;

      //  Modify pose
      // How much to offset
      double offset = current_pose.pose.position.z * tan(req.angle_z);

      // Compute real offset given the offset direction
      current_pose.pose.position.x += offset * cos(req.direction_angle);
      current_pose.pose.position.y += offset * sin(req.direction_angle);
      break;
    }

    if (!pose_in_trajectory) // Pose in the request that are not in the trajectory
    {
      ROS_ERROR_STREAM("Some poses in the request are not in the trajectory");
      return false;
    }
  }

  trajectory.modified = ros::Time::now();
  trajectory_pub.publish(trajectory);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "modify_trajectory");
  ros::NodeHandle nh;

  // Publish on "ram/trajectory"
  trajectory_pub = nh.advertise<ram_msgs::AdditiveManufacturingTrajectory>("ram/trajectory", 10, true);

  // Get unmodified trajectory from buffer node
  unmodified_trajectory_client = nh.serviceClient<ram_utils::UnmodifiedTrajectory>(
      "ram/buffer/get_unmodified_trajectory");
  // Service to add strategies
  entry_exit_strategies_client = nh.serviceClient<ram_utils::AddEntryExitStrategies>(
      "ram/information/add_entry_exit_strategies");

  // Subscribe on "ram/trajectory"
  ros::Subscriber sub = nh.subscribe("ram/trajectory", 10, saveTrajectoryCallback);

// services to modify trajectory
  ros::ServiceServer service_1 = nh.advertiseService("ram/modify_trajectory/modify_selected_poses",
                                                     modifySelectedPosesCallback);
  ros::ServiceServer service_2 = nh.advertiseService("ram/modify_trajectory/reset_selected_poses",
                                                     resetSelectedPosesCallback);
  ros::ServiceServer service_3 = nh.advertiseService("ram/modify_trajectory/add_poses", addPosesCallback);

  ros::ServiceServer service_4 = nh.advertiseService("ram/modify_trajectory/rotate_selected_poses",
                                                     rotateSelectedPosesCallback);
  ros::ServiceServer service_5 = nh.advertiseService("ram/modify_trajectory/reflect_selected_poses",
                                                     reflectSelectedPosesCallback);
  ros::ServiceServer service_6 = nh.advertiseService("ram/modify_trajectory/delete_selected_poses",
                                                     deleteSelectedPosesCallback);
  ros::ServiceServer service_7 = nh.advertiseService("ram/modify_trajectory/scale_selected_poses",
                                                     scaleSelectedPosesCallback);
  ros::ServiceServer service_8 = nh.advertiseService("ram/modify_trajectory/shift_poses",
                                                     shiftPosesCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
