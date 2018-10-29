#include <ram_qt_guis/path_planning.hpp>

namespace ram_qt_guis
{

PathPlanning::PathPlanning(QWidget* parent) :
        rviz::Panel(parent)
{
  connect (this, SIGNAL(enable(const bool)), this, SLOT(setEnabled(const bool)));
  setObjectName("PathPlanning");
  setName(objectName());

  QHBoxLayout *select_algorithm_layout = new QHBoxLayout;
  select_algorithm_layout->addWidget(new QLabel("Generation algorithm:"));

  select_algorithm_ = new QComboBox;
  select_algorithm_layout->addWidget(select_algorithm_);

  algorithm_description_ = new QLabel;

  algorithm_stacked_widget_ = new QStackedWidget();

  generate_trajectory_button_ = new QPushButton("Generate trajectory");

  // Scroll area
  QVBoxLayout *scroll_widget_layout = new QVBoxLayout();
  QWidget *scroll_widget = new QWidget;
  scroll_widget->setLayout(scroll_widget_layout);
  QScrollArea *scroll_area = new QScrollArea;
  scroll_area->setWidget(scroll_widget);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);

  // Main layout
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->addWidget(scroll_area);
  scroll_widget_layout->addLayout(select_algorithm_layout);
  scroll_widget_layout->addWidget(algorithm_description_);
  scroll_widget_layout->addStretch(1);
  scroll_widget_layout->addWidget(algorithm_stacked_widget_);
  scroll_widget_layout->addStretch(2);
  scroll_widget_layout->addWidget(generate_trajectory_button_);

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, SIGNAL(displayMessageBox(const QString, const QString, const QString, const QMessageBox::Icon)),
          this, SLOT(displayMessageBoxHandler(const QString, const QString, const QString, const QMessageBox::Icon)));

  // Algorithm generators
  ram_path_planning::DonghongDing<ram_path_planning::DonghongDingAction> donghong_ding_generator;
  ram_path_planning::Contours<ram_path_planning::ContoursAction> contours_generator;
  ram_path_planning::FollowPoses<ram_path_planning::FollowPosesAction> follow_poses_generator;

  // Add algorithms names
  select_algorithm_->addItem(QString::fromStdString(donghong_ding_generator.name_));
  select_algorithm_->addItem(QString::fromStdString(contours_generator.name_));
  select_algorithm_->addItem(QString::fromStdString(follow_poses_generator.name_));

  // Add algorithms descriptions
  algorithm_descriptions_.clear();
  algorithm_descriptions_.push_back(donghong_ding_generator.description_);
  algorithm_descriptions_.push_back(contours_generator.description_);
  algorithm_descriptions_.push_back(follow_poses_generator.description_);

  // Algorithm widget
  donghong_ding_ui_ = new DonghongDingWidget();
  contours_ui_ = new ContoursWidget();
  follow_poses_ui_ = new FollowPosesWidget();

  // Add algorithm widget
  algorithm_stacked_widget_->addWidget(donghong_ding_ui_);
  algorithm_stacked_widget_->addWidget(contours_ui_);
  algorithm_stacked_widget_->addWidget(follow_poses_ui_);

  // Action clients
  donghong_ding_ac_.reset(new DonghongDingActionClient(donghong_ding_generator.service_name_, true));
  contours_ac_.reset(new ContoursActionClient(contours_generator.service_name_, true));
  follow_poses_ac_.reset(new FollowPosesActionClient(follow_poses_generator.service_name_, true));

  connect(donghong_ding_ui_, SIGNAL(valueChanged()), this, SIGNAL(configChanged()));
  connect(contours_ui_, SIGNAL(valueChanged()), this, SIGNAL(configChanged()));
  connect(follow_poses_ui_, SIGNAL(valueChanged()), this, SIGNAL(configChanged()));
}

PathPlanning::~PathPlanning()
{
}

void PathPlanning::connectToActions()
{
  Q_EMIT enable(false);
  // Generate trajectory

  donghong_ding_ac_->waitForServer();
  contours_ac_->waitForServer();
  follow_poses_ac_->waitForServer();

  ROS_INFO_STREAM("RViz panel " << getName().toStdString() << " actions connections have been made");

  connect(select_algorithm_, SIGNAL(currentIndexChanged(int)), this, SLOT(algorithmChanged()));
  connect(select_algorithm_, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));

  select_algorithm_->setCurrentIndex(loaded_last_algorithm_);
  Q_EMIT select_algorithm_->currentIndexChanged(loaded_last_algorithm_);
  Q_EMIT enable(true);
}

void PathPlanning::algorithmChanged()
{
  Q_EMIT enable(false);

  generate_trajectory_button_->disconnect();

  // Change description
  algorithm_description_->setText(QString::fromStdString(algorithm_descriptions_[select_algorithm_->currentIndex()]));
  // Change widget
  algorithm_stacked_widget_->setCurrentIndex(select_algorithm_->currentIndex());

  // Connect button
  switch (select_algorithm_->currentIndex())
  {
    case 0:
      {
      connect(generate_trajectory_button_, SIGNAL(clicked()), this, SLOT(donghongDingButtonHandler()));
      break;
    }
    case 1:
      {
      connect(generate_trajectory_button_, SIGNAL(clicked()), this, SLOT(contoursButtonHandler()));
      break;
    }
    case 2:
      {
      connect(generate_trajectory_button_, SIGNAL(clicked()), this, SLOT(FollowPosesButtonHandler()));
      break;
    }

    default:
      Q_EMIT displayMessageBox("Operation", "Error selecting algorithm", "", QMessageBox::Icon::Critical);
      break;
  }
  Q_EMIT enable(true);
}

// DonghongDing
void PathPlanning::donghongDingButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);

  std::string error = donghong_ding_ui_->fillGoal(donghong_ding_goal_);
  if (!error.empty())
  {
    Q_EMIT displayMessageBox("Error filling goal", QString::fromStdString(error), "", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  progress_dialog_ = std::make_shared<ProgressDialog>();

  donghong_ding_ac_->sendGoal(donghong_ding_goal_, boost::bind(&PathPlanning::donghongDingDoneCb, this, _1, _2), NULL,
                              boost::bind(&PathPlanning::donghongDingFeedbackCb, this, _1));

  if (!progress_dialog_->exec())
  {
    if (donghong_ding_ac_->getState().state_ == actionlib::SimpleClientGoalState::ACTIVE)
      donghong_ding_ac_->cancelAllGoals();
  }

  progress_dialog_.reset();

  Q_EMIT enable(true);
}

void PathPlanning::donghongDingDoneCb(const actionlib::SimpleClientGoalState &state,
                                      const ram_path_planning::DonghongDingResultConstPtr &result)
{
  ros::Duration(0.1).sleep(); // Otherwise progress window bugs

  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    if (result->error_msg.empty())
    {
      if (progress_dialog_)
        progress_dialog_->accept();
    }
  }

  if (state.state_ == actionlib::SimpleClientGoalState::ABORTED)
  {
    if (progress_dialog_)
      progress_dialog_->reject();

    Q_EMIT displayMessageBox("Path planning failed",
                                  QString::fromStdString(result->error_msg),
                                  "", QMessageBox::Icon::Critical);

  }

  if (state.state_ == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    if (progress_dialog_)
      progress_dialog_->reject();
  }

}

void PathPlanning::donghongDingFeedbackCb(const ram_path_planning::DonghongDingFeedbackConstPtr &feedback)
{
  if (progress_dialog_ && progress_dialog_->isVisible())
    Q_EMIT progress_dialog_->drawProgress(feedback->progress_value, QString::fromStdString(feedback->progress_msg));
}

// Contours
void PathPlanning::contoursButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);

  std::string error = contours_ui_->fillGoal(contours_goal_);
  if (!error.empty())
  {
    Q_EMIT displayMessageBox("Error filling goal", QString::fromStdString(error), "", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  progress_dialog_ = std::make_shared<ProgressDialog>();

  contours_ac_->sendGoal(contours_goal_, boost::bind(&PathPlanning::contoursDoneCb, this, _1, _2), NULL,
                         boost::bind(&PathPlanning::contoursFeedbackCb, this, _1));

  if (!progress_dialog_->exec())
  {
    if (contours_ac_->getState().state_ == actionlib::SimpleClientGoalState::ACTIVE)
      contours_ac_->cancelGoal();
  }

  progress_dialog_.reset();

  Q_EMIT enable(true);
}

void PathPlanning::contoursDoneCb(const actionlib::SimpleClientGoalState &state,
                                  const ram_path_planning::ContoursResultConstPtr &result)
{
  ros::Duration(0.1).sleep(); // Otherwise progress window bugs

  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    if (result->error_msg.empty())
    {
      if (progress_dialog_)
        progress_dialog_->accept();
    }
  }

  if (state.state_ == actionlib::SimpleClientGoalState::ABORTED)
  {
    if (progress_dialog_)
      progress_dialog_->reject();
    Q_EMIT displayMessageBox("Path planning failed",
                                  QString::fromStdString(result->error_msg),
                                  "", QMessageBox::Icon::Critical);
  }

  if (state.state_ == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    if (progress_dialog_)
      progress_dialog_->reject();
  }
  Q_EMIT enable(true);
}

void PathPlanning::contoursFeedbackCb(const ram_path_planning::ContoursFeedbackConstPtr &feedback)
{
  if (progress_dialog_ && progress_dialog_->isVisible())
    Q_EMIT progress_dialog_->drawProgress(feedback->progress_value, QString::fromStdString(feedback->progress_msg));
}

// Follow poses
void PathPlanning::FollowPosesButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);

  std::string error = follow_poses_ui_->fillGoal(follow_poses_goal_);
  if (!error.empty())
  {
    Q_EMIT displayMessageBox("Error filling goal", QString::fromStdString(error), "", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  progress_dialog_ = std::make_shared<ProgressDialog>();

  follow_poses_ac_->sendGoal(follow_poses_goal_, boost::bind(&PathPlanning::FollowPosesDoneCb, this, _1, _2), NULL,
                             boost::bind(&PathPlanning::FollowPosesFeedbackCb, this, _1));

  if (!progress_dialog_->exec())
  {
    if (follow_poses_ac_->getState().state_ == actionlib::SimpleClientGoalState::ACTIVE)
      follow_poses_ac_->cancelGoal();
  }

  progress_dialog_.reset();

  Q_EMIT enable(true);

}

void PathPlanning::FollowPosesDoneCb(const actionlib::SimpleClientGoalState &state,
                                     const ram_path_planning::FollowPosesResultConstPtr &result)
{
  ros::Duration(0.1).sleep(); // Otherwise progress window bugs

  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    if (result->error_msg.empty())
    {
      if (progress_dialog_)
        progress_dialog_->accept();
    }
  }

  if (state.state_ == actionlib::SimpleClientGoalState::ABORTED)
  {
    if (progress_dialog_)
      progress_dialog_->reject();
    Q_EMIT displayMessageBox("Path planning failed",
                                  QString::fromStdString(result->error_msg),
                                  "", QMessageBox::Icon::Critical);
  }

  if (state.state_ == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    if (progress_dialog_)
      progress_dialog_->reject();
  }
  Q_EMIT enable(true);
}

void PathPlanning::FollowPosesFeedbackCb(const ram_path_planning::FollowPosesFeedbackConstPtr &feedback)
{
  if (progress_dialog_ && progress_dialog_->isVisible())
    Q_EMIT progress_dialog_->drawProgress(feedback->progress_value, QString::fromStdString(feedback->progress_msg));
}

void PathPlanning::displayMessageBoxHandler(const QString title,
                                            const QString text,
                                            const QString info,
                                            const QMessageBox::Icon icon)
{
  const bool old_state(isEnabled());
  setEnabled(false);
  QMessageBox msg_box;
  msg_box.setWindowTitle(title);
  msg_box.setText(text);
  msg_box.setInformativeText(info);
  msg_box.setIcon(icon);
  msg_box.setStandardButtons(QMessageBox::Ok);
  msg_box.exec();
  setEnabled(old_state);
}

void PathPlanning::load(const rviz::Config& config)
{
  Q_EMIT configChanged();

  int tmp_int(0);
  if (config.mapGetInt("algorithm", &tmp_int))
    loaded_last_algorithm_ = tmp_int;

  donghong_ding_ui_->load(config);
  contours_ui_->load(config);
  follow_poses_ui_->load(config);
  rviz::Panel::load(config);

  // Check connection of client
  QtConcurrent::run(this, &PathPlanning::connectToActions);
}

void PathPlanning::save(rviz::Config config) const
                        {
  config.mapSetValue("algorithm", select_algorithm_->currentIndex());

  donghong_ding_ui_->save(config);
  contours_ui_->save(config);
  follow_poses_ui_->save(config);
  rviz::Panel::save(config);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ram_qt_guis::PathPlanning, rviz::Panel)
