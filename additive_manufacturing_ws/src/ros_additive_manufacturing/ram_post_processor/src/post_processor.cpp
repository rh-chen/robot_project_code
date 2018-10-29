#include <ram_post_processor/post_processor.hpp>

namespace ram_post_processor
{
PostProcessor::PostProcessor(const std::string name,
                             const std::string description,
                             const std::string service_name) :
        name_(name),
        description_(description),
        service_name_(service_name)
{
  get_start_pose_client_ = nh_.serviceClient<ram_utils::GetStartPose>("ram/get_start_pose");
  get_tool_client_ = nh_.serviceClient<ram_utils::GetTool>("ram/get_tool");
  get_trajectory_infos_client_ = nh_.serviceClient<ram_utils::GetTrajectoryInfos>(
      "ram/information/get_trajectory_infos");
}

void PostProcessor::addComment(const std::string comment)
{
  programs_.front().second.append("\n" + comment);
}

void PostProcessor::addPose(const bool,
                            const bool,
                            const bool,
                            const bool,
                            const bool,
                            const bool)
{
  std::stringstream buffer;
  buffer << currentPose()->pose;
  std::string pose_str(buffer.str());
  programs_.front().second.append("\n Pose: \n" + buffer.str());
}

void PostProcessor::clearProgram()
{
  programs_.clear();
}

void PostProcessor::beforeGenerating()
{
}

void PostProcessor::afterGenerating()
{
}

/// Polygons
void PostProcessor::startPolygonBefore()
{
  programs_.front().second.append("\nStart polygon");
}

void PostProcessor::startPolygonAfter()
{
}

void PostProcessor::finishPolygonBefore()
{
}

void PostProcessor::finishPolygonAfter()
{
  programs_.front().second.append("\nFinish polygon");
}

/// Layer change
void PostProcessor::layerIndexChanged()
{
  if (verbose_comments_)
  {
    programs_.front().second.append("\nLayer index " + std::to_string(currentPose()->layer_index));
    programs_.front().second.append("\nLayer level " + std::to_string(currentPose()->layer_level));
  }
}

/// Feed rate
void PostProcessor::startFeedRateBefore()
{
  programs_.front().second.append("\nStart feed rate: " + std::to_string(currentPose()->params.feed_rate));
}

void PostProcessor::startFeedRateAfter()
{
}

void PostProcessor::changeFeedRateBefore()
{
  programs_.front().second.append("\nChange feed rate: " + std::to_string(currentPose()->params.feed_rate));
}

void PostProcessor::changeFeedRateAfter()
{
}

void PostProcessor::stopFeedRateBefore()
{
  programs_.front().second.append("\nStop feed rate");
}

void PostProcessor::stopFeedRateAfter()
{
  programs_.front().second.append("\nStop feed rate");
}

/// Laser power
void PostProcessor::startLaserPowerBefore()
{
  programs_.front().second.append("\nStart laser_power: " + std::to_string(currentPose()->params.laser_power));
}

void PostProcessor::startLaserPowerAfter()
{
}

void PostProcessor::changeLaserPowerBefore()
{
  programs_.front().second.append("\nChange laser power: " + std::to_string(currentPose()->params.laser_power));
}

void PostProcessor::changeLaserPowerAfter()
{
}

void PostProcessor::stopLaserPowerBefore()
{
  programs_.front().second.append("\nStop laser power");
}

void PostProcessor::stopLaserPowerAfter()
{
}

void PostProcessor::generatePrograms(const ram_msgs::AdditiveManufacturingTrajectory &trajectory,
                                     std::vector<std::pair<std::string, std::string>> &programs)
{
  ram_msgs::AdditiveManufacturingPose last_pose;

  /// Clear program
  clearProgram();

  /// Append program
  programs_.push_back(std::pair<std::string, std::string>(program_name_, ""));

  if (trajectory.poses.empty())
    throw std::runtime_error("Trajectory is empty");

  trajectory_ = trajectory;

  /// Get start pose
  // Check that service exists
  if (!get_start_pose_client_.waitForExistence(ros::Duration(0.5)))
    throw std::runtime_error("Cannot get start pose, service does not exist");

  // Call service to get start pose
  ram_utils::GetStartPose srv_1;
  if (!get_start_pose_client_.call(srv_1))
    throw std::runtime_error("Cannot get start pose, call to service failed");

  tf::poseMsgToEigen(srv_1.response.pose, start_pose_);

  /// Get tool orientation
  // Check that service exists
  if (!get_tool_client_.waitForExistence(ros::Duration(0.5)))
    throw std::runtime_error("Cannot get tool, service does not exist");

  // Call service to get start pose
  ram_utils::GetTool srv_2;
  if (!get_tool_client_.call(srv_2))
    throw std::runtime_error("Cannot get tool, call to service failed");

  tf::poseMsgToEigen(srv_2.response.pose, tool_);

  programs_.front().second.append("Program comment: " + program_comment_);
  if (verbose_comments_)
  {
    unsigned highest_layer(0);
    for (auto ram_pose : trajectory_.poses)
      if (ram_pose.layer_level > highest_layer)
        highest_layer = ram_pose.layer_level;
    addComment("File = " + trajectory_.file);
    addComment("Number of layers = " + std::to_string(highest_layer + 1));
    addComment(trajectory_.generation_info);

    // Check that service exists
    if (!get_trajectory_infos_client_.waitForExistence(ros::Duration(0.5)))
      throw std::runtime_error("Cannot get trajectory infos, service does not exist");

    // Call service to get traj infos
    ram_utils::GetTrajectoryInfos srv_3;
    if (!get_trajectory_infos_client_.call(srv_3))
      throw std::runtime_error("Cannot get trajectory infos, call to service failed");
    else
    {
      uint16_t seconds(srv_3.response.informations.execution_time);
      unsigned hours(seconds / 3600);
      seconds -= hours * 3600;

      unsigned minutes(seconds / 60);
      seconds -= minutes * 60;

      addComment("Execution duration = " + std::to_string(hours) + ":" + std::to_string(minutes) + ":"
          + std::to_string(seconds));
      addComment("");
    }
  }

  beforeGenerating();

  /// Transform all poses using start pose and tool orientation
  for (auto &ram_pose : trajectory_.poses)
  {
    Eigen::Isometry3d pose;
    // geometry_msg to Eigen
    tf::poseMsgToEigen(ram_pose.pose, pose);
    // Transform pose
    pose = start_pose_ * pose * tool_;
    // Eigen to geometry
    tf::poseEigenToMsg(pose, ram_pose.pose);
  }

  bool first_pose(true);
  for (current_pose_ = trajectory_.poses.begin(); current_pose_ != trajectory_.poses.end(); ++current_pose_)
  {
    bool laser_power_start(false),
         laser_power_stop(false),
         laser_power_change(false),
         feed_rate_start(false),
         feed_rate_stop(false),
         feed_rate_change(false);

    /// Actions BEFORE adding pose

    // (Layer index changed OR first pose) AND verbose comments
    if ((current_pose_->layer_index != last_pose.layer_index || first_pose))
      layerIndexChanged();

    if (current_pose_->polygon_start)
      startPolygonBefore();

    if (current_pose_->polygon_end)
      finishPolygonBefore();

    if (first_pose)
    {
      if (current_pose_->params.feed_rate != 0)
      {
        feed_rate_start = true;
        startFeedRateBefore();
      }

      if (last_pose.params.laser_power != 0)
      {
        laser_power_start = true;
        startLaserPowerBefore();
      }
    }
    else
    {
      // Feed rate changes
      if (current_pose_->params.feed_rate != last_pose.params.feed_rate)
      {
        if (last_pose.params.feed_rate != 0 && current_pose_->params.feed_rate == 0)
        {
          feed_rate_stop = true;
          stopFeedRateBefore();
        }
        else if (last_pose.params.feed_rate == 0)
        {
          feed_rate_start = true;
          startFeedRateBefore();
        }
        else
        {
          feed_rate_change = true;
          changeFeedRateBefore();
        }
      }

      // Laser power changes
      if (current_pose_->params.laser_power != last_pose.params.laser_power)
      {
        if (last_pose.params.laser_power != 0 && current_pose_->params.laser_power == 0)
        {
          laser_power_stop = true;
          stopLaserPowerBefore();
        }
        else if (last_pose.params.laser_power == 0)
        {
          laser_power_start = true;
          startLaserPowerBefore();
        }
        else
        {
          laser_power_change = true;
          changeLaserPowerBefore();
        }
      }
    }

    /// Add transformed pose
    addPose(laser_power_start,
            laser_power_stop,
            laser_power_change,
            feed_rate_start,
            feed_rate_stop,
            feed_rate_change);

    /// Actions AFTER adding pose
    if (current_pose_->polygon_start)
      startPolygonAfter();

    if (current_pose_->polygon_end)
      finishPolygonAfter();

    if (first_pose)
    {
      if (current_pose_->params.feed_rate != 0)
        startFeedRateAfter();

      if (last_pose.params.laser_power != 0)
        startLaserPowerAfter();
    }
    else
    {
      // Feed rate changes
      if (current_pose_->params.feed_rate != last_pose.params.feed_rate)
      {
        if (last_pose.params.feed_rate != 0 && current_pose_->params.feed_rate == 0)
          stopFeedRateAfter();
        else if (last_pose.params.feed_rate == 0)
          startFeedRateAfter();
        else
          changeFeedRateAfter();
      }

      // Laser power changes
      if (current_pose_->params.laser_power != last_pose.params.laser_power)
      {
        if (last_pose.params.laser_power != 0 && current_pose_->params.laser_power == 0)
          stopLaserPowerAfter();
        else if (last_pose.params.laser_power == 0)
          startLaserPowerAfter();
        else
          changeLaserPowerAfter();
      }
    }

    first_pose = false;
    last_pose = *current_pose_;
  }

  afterGenerating();
  programs = programs_;
}

void PostProcessor::saveToFiles(const std::string directory,
                                const std::string file_extension)
{
  std::ofstream program_file;
  for (auto &program : programs_)
  {
    const std::string file_name(directory + "/" + program.first + file_extension);
    program_file.open(file_name, std::ofstream::out);
    if (!program_file.is_open())
      throw std::runtime_error("Could not open file " + file_name + " for writing");

    program_file << program.second;
    program_file.close();
  }
  return;
}

}
