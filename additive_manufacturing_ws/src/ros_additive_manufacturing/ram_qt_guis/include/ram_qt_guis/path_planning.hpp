#ifndef RAM_QT_GUIS_PATH_PLANNING_HPP
#define RAM_QT_GUIS_PATH_PLANNING_HPP

#ifndef Q_MOC_RUN
#include <eigen_conversions/eigen_msg.h>
#include <ram_path_planning/contours.hpp>
#include <ram_path_planning/donghong_ding.hpp>
#include <ram_path_planning/follow_poses.hpp>
#include <ram_qt_guis/path_planning_widgets/contours.hpp>
#include <ram_qt_guis/path_planning_widgets/donghong_ding.hpp>
#include <ram_qt_guis/path_planning_widgets/follow_poses.hpp>
#include <ram_qt_guis/path_planning_widgets/progress_dialog.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>

#include <actionlib/client/simple_action_client.h>
#include <ram_path_planning/ContoursAction.h>
#include <ram_path_planning/DonghongDingAction.h>
#include <ram_path_planning/FollowPosesAction.h>
#endif

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QStackedWidget>
#include <QtConcurrent/QtConcurrentRun>

namespace ram_qt_guis
{
class PathPlanning : public rviz::Panel
{
Q_OBJECT

public:
  PathPlanning(QWidget* parent = NULL);
  virtual ~PathPlanning();

Q_SIGNALS:
  void enable(const bool);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

protected Q_SLOTS:
  void addAlgorithmsToGUI();
  void algorithmChanged();

  void donghongDingButtonHandler();
  void donghongDingDoneCb(const actionlib::SimpleClientGoalState &state,
                          const ram_path_planning::DonghongDingResultConstPtr &result);
  void donghongDingFeedbackCb(const ram_path_planning::DonghongDingFeedbackConstPtr &feedback);

  void contoursButtonHandler();
  void contoursDoneCb(const actionlib::SimpleClientGoalState &state,
                      const ram_path_planning::ContoursResultConstPtr &result);
  void contoursFeedbackCb(const ram_path_planning::ContoursFeedbackConstPtr &feedback);

  void FollowPosesButtonHandler();
  void FollowPosesDoneCb(const actionlib::SimpleClientGoalState &state,
                         const ram_path_planning::FollowPosesResultConstPtr &result);
  void FollowPosesFeedbackCb(const ram_path_planning::FollowPosesFeedbackConstPtr &feedback);

  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

private:
  void connectToActions();

  std::shared_ptr<ProgressDialog> progress_dialog_;

  ros::NodeHandle nh_;

  QStackedWidget * algorithm_stacked_widget_;
  QPushButton *generate_trajectory_button_;

  QComboBox *select_algorithm_;
  QLabel *algorithm_description_;
  std::vector<std::string> algorithm_descriptions_;
  unsigned loaded_last_algorithm_ = 0;

  DonghongDingWidget *donghong_ding_ui_;
  ContoursWidget *contours_ui_;
  FollowPosesWidget *follow_poses_ui_;

  // Action clients
  typedef actionlib::SimpleActionClient<ram_path_planning::DonghongDingAction> DonghongDingActionClient;
  typedef actionlib::SimpleActionClient<ram_path_planning::ContoursAction> ContoursActionClient;
  typedef actionlib::SimpleActionClient<ram_path_planning::FollowPosesAction> FollowPosesActionClient;

  std::unique_ptr<DonghongDingActionClient> donghong_ding_ac_;
  std::unique_ptr<ContoursActionClient> contours_ac_;
  std::unique_ptr<FollowPosesActionClient> follow_poses_ac_;

  // Goals
  ram_path_planning::DonghongDingGoal donghong_ding_goal_;
  ram_path_planning::ContoursGoal contours_goal_;
  ram_path_planning::FollowPosesGoal follow_poses_goal_;

};

}

#endif
