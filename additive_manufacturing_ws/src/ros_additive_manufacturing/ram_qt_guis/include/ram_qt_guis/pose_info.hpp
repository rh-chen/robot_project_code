#ifndef RAM_QT_GUIS_POSE_INFO_HPP
#define RAM_QT_GUIS_POSE_INFO_HPP

#ifndef Q_MOC_RUN
#include <eigen_conversions/eigen_msg.h>
#include <mutex>
#include <ram_modify_trajectory/GetPosesFromTrajectory.h>
#include <ram_msgs/AdditiveManufacturingTrajectory.h>
#include <ram_qt_guis/pose.hpp>
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <QDateTime>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>

namespace ram_qt_guis
{
class PoseInfo : public rviz::Panel
{
Q_OBJECT
  public:
  PoseInfo(QWidget* parent = NULL);
  virtual ~PoseInfo();

  void connectToServices();
  void checkForPublishers();
  void trajectoryCallback(const ram_msgs::AdditiveManufacturingTrajectoryConstPtr&);

Q_SIGNALS:
  void enable(const bool);

protected Q_SLOTS:
  void backButtonHandler();
  void forwardButtonHandler();
  void firstPoseButtonHandler();
  void lastPoseButtonHandler();
  void getPoseInformation();
  void updateGUIparameters();

   void load(const rviz::Config& config);
   void save(rviz::Config config) const;

protected:
  unsigned trajectory_size_;

  // Geometric pose
  Pose *pose_;

  // Info
  QLabel *layers_level_;
  QLabel *layer_index_;
  QLabel *polygon_start_;
  QLabel *polygon_end_;
  QLabel *entry_pose_;
  QLabel *exit_pose_;

  // Parameters
  QLabel *movement_type_;
  QLabel *approach_type_;
  QLabel *blend_radius_;
  QLabel *speed_;
  QLabel *laser_power_;
  QLabel *feed_rate_;

  // Buttons
  QPushButton *back_button_;
  QPushButton *forward_button_;
  QPushButton *first_pose_button_;
  QPushButton *last_pose_button_;

  // Index
  QSpinBox *pose_index_;

  // ROS
  ros::NodeHandle nh_;
  ros::ServiceClient get_poses_from_trajectory_client_;
  ros::Subscriber trajectory_sub_;
  std::recursive_mutex pose_params_mutex_;
  ram_modify_trajectory::GetPosesFromTrajectory pose_params_;
};

}

#endif
