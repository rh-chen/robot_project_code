#include <ram_qt_guis/pose_info.hpp>

namespace ram_qt_guis
{

PoseInfo::PoseInfo(QWidget* parent) :
        rviz::Panel(parent),
        trajectory_size_(0)
{
  connect(this, SIGNAL(enable(const bool)), this, SLOT(setEnabled(const bool)));
  setObjectName("Pose info");
  setName(objectName());
  pose_index_ = new QSpinBox;

  pose_ = new Pose("pose_info", "<b>Geometric pose:</b>", Pose::Mode::READ_ONLY);

  QGridLayout* info_layout = new QGridLayout;
  layers_level_ = new QLabel;
  info_layout->addWidget(new QLabel("Layer level:"), 0, 0);
  info_layout->addWidget(layers_level_, 0, 1);
  layer_index_ = new QLabel;
  info_layout->addWidget(new QLabel("Layer index:"));
  info_layout->addWidget(layer_index_);
  polygon_start_ = new QLabel;
  info_layout->addWidget(new QLabel("Polygon start:"));
  info_layout->addWidget(polygon_start_);
  polygon_end_ = new QLabel;
  info_layout->addWidget(new QLabel("Polygon end:"));
  info_layout->addWidget(polygon_end_);
  entry_pose_ = new QLabel;
  info_layout->addWidget(new QLabel("Entry pose:"));
  info_layout->addWidget(entry_pose_);
  exit_pose_ = new QLabel;
  info_layout->addWidget(new QLabel("Exit pose:"));
  info_layout->addWidget(exit_pose_);

  QGridLayout* params_layout = new QGridLayout;
  movement_type_ = new QLabel;
  params_layout->addWidget(new QLabel("Movement type:"), 0, 0);
  params_layout->addWidget(movement_type_, 0, 1);
  approach_type_ = new QLabel;
  params_layout->addWidget(new QLabel("Approach type:"));
  params_layout->addWidget(approach_type_);
  blend_radius_ = new QLabel;
  params_layout->addWidget(new QLabel("Blend radius:"));
  params_layout->addWidget(blend_radius_);
  speed_ = new QLabel;
  params_layout->addWidget(new QLabel("Speed:"));
  params_layout->addWidget(speed_);
  laser_power_ = new QLabel;
  params_layout->addWidget(new QLabel("Laser power:"));
  params_layout->addWidget(laser_power_);
  feed_rate_ = new QLabel;
  params_layout->addWidget(new QLabel("Feed rate:"));
  params_layout->addWidget(feed_rate_);

  QHBoxLayout* get_pose_layout = new QHBoxLayout;
  pose_index_ = new QSpinBox;
  pose_index_->setRange(0, trajectory_size_ - 1);
  get_pose_layout->addWidget(new QLabel("Pose index:"));
  get_pose_layout->addWidget(pose_index_);

  QHBoxLayout* buttons_layout = new QHBoxLayout;
  first_pose_button_ = new QPushButton("<<");
  first_pose_button_->setMinimumWidth(5);
  buttons_layout->addWidget(first_pose_button_);
  back_button_ = new QPushButton("<");
  back_button_->setMinimumWidth(5);
  buttons_layout->addWidget(back_button_);
  forward_button_ = new QPushButton(">");
  back_button_->setMinimumWidth(5);
  buttons_layout->addWidget(forward_button_);
  last_pose_button_ = new QPushButton(">>");
  last_pose_button_->setMinimumWidth(5);
  buttons_layout->addWidget(last_pose_button_);

  QVBoxLayout *scroll_widget_layout = new QVBoxLayout();
  QWidget *scroll_widget = new QWidget;
  scroll_widget->setLayout(scroll_widget_layout);
  QScrollArea *scroll_area = new QScrollArea;
  scroll_area->setWidget(scroll_widget);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->addWidget(scroll_area);

  scroll_widget_layout->addWidget(pose_);
  scroll_widget_layout->addStretch(1);
  scroll_widget_layout->addWidget(new QLabel("<b>Information</b>"));
  scroll_widget_layout->addLayout(info_layout);
  scroll_widget_layout->addStretch(1);

  scroll_widget_layout->addWidget(new QLabel("<b>Parameters</b>"));
  scroll_widget_layout->addLayout(params_layout);
  scroll_widget_layout->addStretch(2);
  scroll_widget_layout->addLayout(get_pose_layout);
  scroll_widget_layout->addLayout(buttons_layout);

  connect(pose_index_, SIGNAL(valueChanged(int)), this, SLOT(getPoseInformation()));

  connect(back_button_, SIGNAL(clicked()), this, SLOT(backButtonHandler()));
  connect(forward_button_, SIGNAL(clicked()), this, SLOT(forwardButtonHandler()));
  connect(first_pose_button_, SIGNAL(clicked()), this, SLOT(firstPoseButtonHandler()));
  connect(last_pose_button_, SIGNAL(clicked()), this, SLOT(lastPoseButtonHandler()));

  trajectory_sub_ = nh_.subscribe("ram/trajectory", 1, &PoseInfo::trajectoryCallback, this);
  QtConcurrent::run(this, &PoseInfo::checkForPublishers);

  get_poses_from_trajectory_client_ = nh_.serviceClient<ram_modify_trajectory::GetPosesFromTrajectory>(
      "ram/pose_selector/get_poses_from_trajectory");
  QtConcurrent::run(this, &PoseInfo::connectToServices);

  Q_EMIT enable(false);
}

PoseInfo::~PoseInfo()
{
}

void PoseInfo::checkForPublishers()
{
  while (nh_.ok())
  {
    ros::Duration(1).sleep();

    if (trajectory_sub_.getNumPublishers() != 0)
    {
      ROS_INFO_STREAM(
          "RViz panel " << getName().toStdString() << " topic " << trajectory_sub_.getTopic() << " has at least one publisher");
      break;
    }
    else
    {
      ROS_ERROR_STREAM(
          "RViz panel " << getName().toStdString() << " topic " << trajectory_sub_.getTopic() << " has zero publishers!");
    }
  }
}

void PoseInfo::trajectoryCallback(const ram_msgs::AdditiveManufacturingTrajectoryConstPtr& msg)
{
  trajectory_size_ = msg->poses.size();

  if (trajectory_size_ == 0)
  {
    Q_EMIT enable(false);
    return;
  }

  Q_EMIT enable(true);
  pose_index_->setRange(0, trajectory_size_ - 1);
  pose_index_->setValue(pose_index_->minimum());
  // Force update in case current value is zero
  Q_EMIT pose_index_->valueChanged(pose_index_->value());
}

void PoseInfo::connectToServices()
{
  while (nh_.ok())
  {
    if (get_poses_from_trajectory_client_.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM(
          "RViz panel " << getName().toStdString() << " connected to the service " << get_poses_from_trajectory_client_.getService());
      break;
    }
    else
    {
      ROS_ERROR_STREAM(
          "RViz panel " << getName().toStdString() << " could not connect to ROS service: " << get_poses_from_trajectory_client_.getService());
      ros::Duration(1).sleep();
    }
  }
  ROS_INFO_STREAM("RViz panel " << getName().toStdString() << " services connections have been made");

}

void PoseInfo::forwardButtonHandler()
{
  pose_index_->setValue(pose_index_->value() + 1);
}

void PoseInfo::backButtonHandler()
{
  pose_index_->setValue(pose_index_->value() - 1);
}

void PoseInfo::firstPoseButtonHandler()
{
  pose_index_->setValue(pose_index_->minimum());
}

void PoseInfo::lastPoseButtonHandler()
{
  pose_index_->setValue(pose_index_->maximum());
}

void PoseInfo::getPoseInformation()
{
  std::lock_guard<std::recursive_mutex> lock(pose_params_mutex_);

  Q_EMIT configChanged();

  pose_params_.request.pose_index_list.clear();
  pose_params_.request.pose_index_list.push_back(pose_index_->value());

  bool success(get_poses_from_trajectory_client_.call(pose_params_));
  if (success && (pose_params_.response.poses.size() == 1))
    Q_EMIT updateGUIparameters();
}

void PoseInfo::updateGUIparameters()
{
  std::lock_guard<std::recursive_mutex> lock(pose_params_mutex_);

  layers_level_->setText(QString::number(pose_params_.response.poses[0].layer_level));
  layer_index_->setText(QString::number(pose_params_.response.poses[0].layer_index));
  polygon_start_->setText((pose_params_.response.poses[0].polygon_start) ? "True" : "False");
  polygon_end_->setText((pose_params_.response.poses[0].polygon_end) ? "True" : "False");
  entry_pose_->setText((pose_params_.response.poses[0].entry_pose) ? "True" : "False");
  exit_pose_->setText((pose_params_.response.poses[0].exit_pose) ? "True" : "False");
  // Geometric pose
  Eigen::Isometry3d pose;
  tf::poseMsgToEigen(pose_params_.response.poses[0].pose, pose);
  pose_->setPose(pose);

  // Others parameters
  QString movement_type_str;
  QString speed_unit_str = "";
  double speed_conversion_factor;
  switch (pose_params_.response.poses[0].params.movement_type)
  {
    case 0:
      movement_type_str = "Joint";
      speed_conversion_factor = 1.0;
      break;
    case 1:
      movement_type_str = "Linear";
      speed_unit_str = " meters/min";
      speed_conversion_factor = 60.0; // meters/sec --> meters/min
      break;
    default:
      movement_type_str = "";
      speed_conversion_factor = 0;
      break;
  }
  movement_type_->setText(movement_type_str);

  QString approach_type_str;
  switch (pose_params_.response.poses[0].params.approach_type)
  {
    case 0:
      approach_type_str = "Stop/go";
      break;
    case 1:
      approach_type_str = "Blend radius";
      break;
    default:
      approach_type_str = "-";
      break;
  }
  approach_type_->setText(approach_type_str);

  blend_radius_->setText(QString::number(pose_params_.response.poses[0].params.blend_radius) + "%");
  speed_->setText(
      QString::number(pose_params_.response.poses[0].params.speed * speed_conversion_factor) + speed_unit_str);
  laser_power_->setText(QString::number(pose_params_.response.poses[0].params.laser_power) + " W");
  feed_rate_->setText(
                      QString::number(pose_params_.response.poses[0].params.feed_rate * 60.0) + " meters/min"); // meters/sec --> meters/min
}

void PoseInfo::load(const rviz::Config& config)
{
  int tmp_int(0);
  Q_EMIT configChanged();

  if (config.mapGetInt("pose_index_box", &tmp_int))
    pose_index_->setValue(tmp_int);

  pose_->load(config);
  const Eigen::Isometry3d identity(Eigen::Isometry3d::Identity());
  pose_->setPose(identity);
  rviz::Panel::load(config);
}

void PoseInfo::save(rviz::Config config) const
                    {
  config.mapSetValue("pose_index_box", pose_index_->value());
  pose_->save(config);
  rviz::Panel::save(config);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ram_qt_guis::PoseInfo, rviz::Panel)
