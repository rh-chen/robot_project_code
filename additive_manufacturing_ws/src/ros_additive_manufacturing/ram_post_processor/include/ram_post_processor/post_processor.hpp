#ifndef RAM_PATH_PLANNING_POST_PROCESSOR_HPP
#define RAM_PATH_PLANNING_POST_PROCESSOR_HPP

#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_utils/GetStartPose.h>
#include <ram_utils/GetTool.h>
#include <ram_utils/GetTrajectoryInfos.h>
#include <ros/ros.h>

namespace ram_post_processor
{
class PostProcessor
{
public:
  PostProcessor(const std::string name = "Generic",
                const std::string description = "A human readable textual output.\n"
                    "Not meant to be used by any kind of hardware.",
                const std::string service_name = "ram/post_processors/generic");

  virtual ~PostProcessor()
  {
  }

  virtual void
  addComment(const std::string comment);

  virtual void
  addPose(const bool laser_power_start,
          const bool laser_power_stop,
          const bool laser_power_change,
          const bool feed_rate_start,
          const bool feed_rate_stop,
          const bool feed_rate_change);

  virtual void
  clearProgram();

  virtual void
  beforeGenerating();

  virtual void
  afterGenerating();

  /// Polygons
  virtual void
  startPolygonBefore();

  virtual void
  startPolygonAfter();

  virtual void
  finishPolygonBefore();

  virtual void
  finishPolygonAfter();

  /// Layer change
  virtual void
  layerIndexChanged();

  /// Feed rate
  virtual void
  startFeedRateBefore();
  virtual void
  startFeedRateAfter();
  virtual void
  changeFeedRateBefore();
  virtual void
  changeFeedRateAfter();
  virtual void
  stopFeedRateBefore();
  virtual void
  stopFeedRateAfter();

  /// Laser power
  virtual void
  startLaserPowerBefore();
  virtual void
  startLaserPowerAfter();
  virtual void
  changeLaserPowerBefore();
  virtual void
  changeLaserPowerAfter();
  virtual void
  stopLaserPowerBefore();
  virtual void
  stopLaserPowerAfter();

  void
  generatePrograms(const ram_msgs::AdditiveManufacturingTrajectory &trajectory,
                   std::vector<std::pair<std::string, std::string>> &programs);

  virtual void
  saveToFiles(const std::string directory, const std::string file_extension = "");

  const ram_msgs::AdditiveManufacturingTrajectory& traj()
  {
    return trajectory_;
  }

  typedef ram_msgs::AdditiveManufacturingTrajectory::_poses_type::const_iterator ram_pose_const_iterator;

  const ram_pose_const_iterator& currentPose()
  {
    return current_pose_;
  }

  const std::string name_;
  const std::string description_;
  const std::string service_name_;

  std::string program_name_ = "program";
  std::string program_comment_ = "comment";
  bool verbose_comments_ = false;

protected:
  ros::NodeHandle nh_;
  // Post-processor might generate multiple programs at once
  // Pair: <program name, program content>
  std::vector<std::pair<std::string, std::string>> programs_;

private:
  ram_msgs::AdditiveManufacturingTrajectory trajectory_;
  ram_pose_const_iterator current_pose_ = trajectory_.poses.begin();

  ros::ServiceClient get_start_pose_client_;
  ros::ServiceClient get_tool_client_;
  ros::ServiceClient get_trajectory_infos_client_;

  Eigen::Isometry3d start_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tool_ = Eigen::Isometry3d::Identity();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}
#endif
