#include <ram_qt_guis/fill_trajectory.hpp>

namespace ram_qt_guis
{
FillTrajectory::FillTrajectory(QWidget* parent) :
        rviz::Panel(parent)
{
  connect(this, SIGNAL(enable(const bool)), this, SLOT(setEnabled(const bool)));
  setObjectName("Fill trajectory");
  setName(objectName());

  conversion_factor_ = 1;

  movement_type_ = new QComboBox;
  movement_type_->addItem("Joint");
  movement_type_->addItem("Linear");
  QHBoxLayout *movement_type_layout = new QHBoxLayout;
  movement_type_layout->addWidget(new QLabel("Movement type:"));
  movement_type_layout->addWidget(movement_type_);
  connect(movement_type_, SIGNAL(currentIndexChanged(int)), this, SLOT(movementTypeChanged()));

  approach_type_ = new QComboBox;
  approach_type_->addItem("Stop/go");
  approach_type_->addItem("Blend radius");
  QHBoxLayout *approach_type_layout = new QHBoxLayout;
  approach_type_layout->addWidget(new QLabel("Approach type:"));
  approach_type_layout->addWidget(approach_type_);
  connect(approach_type_, SIGNAL(currentIndexChanged(int)), this, SLOT(approachTypeChanged()));

  blend_radius_ = new QSpinBox;
  blend_radius_->setRange(0, 100);
  blend_radius_->setSingleStep(5);
  blend_radius_->setSuffix(" %");
  QHBoxLayout *blend_radius_layout = new QHBoxLayout;
  blend_radius_layout->addWidget(new QLabel("Blend radius:"));
  blend_radius_layout->addWidget(blend_radius_);

  speed_ = new QDoubleSpinBox;
  speed_->setRange(0.001, 100);
  speed_->setSingleStep(0.1);
  QHBoxLayout *speed_layout = new QHBoxLayout;
  speed_layout->addWidget(new QLabel("Speed:"));
  speed_layout->addWidget(speed_);

  laser_power_ = new QSpinBox;
  laser_power_->setRange(0, 32000);
  laser_power_->setSingleStep(100);
  laser_power_->setSuffix(" W");
  QHBoxLayout *laser_power_layout = new QHBoxLayout;
  laser_power_layout->addWidget(new QLabel("Laser power:"));
  laser_power_layout->addWidget(laser_power_);

  feed_rate_ = new QDoubleSpinBox;
  feed_rate_->setRange(0, 10);
  feed_rate_->setSingleStep(0.1);
  feed_rate_->setSuffix(" meters/min");
  QHBoxLayout *feed_rate_layout = new QHBoxLayout;
  feed_rate_layout->addWidget(new QLabel("Feed rate:"));
  feed_rate_layout->addWidget(feed_rate_);

  send_button_ = new QPushButton("Send");
  connect(send_button_, SIGNAL(clicked()), this, SLOT(sendInformationButtonHandler()));

  QVBoxLayout *scroll_widget_layout = new QVBoxLayout();
  QWidget *scroll_widget = new QWidget;
  scroll_widget->setLayout(scroll_widget_layout);
  QScrollArea *scroll_area = new QScrollArea;
  scroll_area->setWidget(scroll_widget);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->addWidget(scroll_area);

  scroll_widget_layout->addLayout(movement_type_layout);
  scroll_widget_layout->addLayout(approach_type_layout);
  scroll_widget_layout->addLayout(blend_radius_layout);
  scroll_widget_layout->addLayout(speed_layout);
  scroll_widget_layout->addLayout(laser_power_layout);
  scroll_widget_layout->addLayout(feed_rate_layout);
  scroll_widget_layout->addStretch(1);

  main_layout->addWidget(send_button_);

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, SIGNAL(displayMessageBox(const QString, const QString, const QString, const QMessageBox::Icon)),
          this, SLOT(displayMessageBoxHandler(const QString, const QString, const QString, const QMessageBox::Icon)));

  // connect with configChanged signal
  connect(movement_type_, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
  connect(approach_type_, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
  connect(blend_radius_, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
  connect(speed_, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
  connect(laser_power_, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
  connect(feed_rate_, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

  pub_ = nh_.advertise<ram_msgs::AdditiveManufacturingParams>("ram/fill_trajectory/parameters", 1);
}

FillTrajectory::~FillTrajectory()
{
}

void FillTrajectory::updateInternalParameters()
{
  params_.movement_type = movement_type_->currentIndex();
  params_.approach_type = approach_type_->currentIndex();
  params_.blend_radius = blend_radius_->value();
  params_.speed = speed_->value() * conversion_factor_;
  params_.laser_power = laser_power_->value();
  params_.feed_rate = feed_rate_->value() / 60.0; // meters/min > meters / sec
}

void FillTrajectory::movementTypeChanged()
{
  Q_EMIT enable(false);
  // Change speed
  switch (movement_type_->currentIndex())
  {
    case 0: //Joint
      speed_->setSuffix("");
      conversion_factor_ = 1;
      break;
    case 1: //Linear
      speed_->setSuffix(" meters/min");
      conversion_factor_ = 1 / 60.0; // meters/min > meters / sec
      break;
  }
  updateInternalParameters();
  Q_EMIT enable(true);
}

void FillTrajectory::approachTypeChanged()
{
  Q_EMIT enable(false);
  switch (approach_type_->currentIndex())
  {
    case 0: // Stop-go
      blend_radius_->setEnabled(false);
      break;
    case 1: // Blend radius
      blend_radius_->setEnabled(true);
      break;
  }
  Q_EMIT enable(true);
}

void FillTrajectory::sendInformationButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);
  updateInternalParameters();
  // Run in a separate thread
  QtConcurrent::run(this, &FillTrajectory::sendInformation);
}

void FillTrajectory::sendInformation()
{
  if (pub_.getNumSubscribers() == 0)
  {
    Q_EMIT displayMessageBox("Published message will be lost: ",
                             QString::fromStdString(pub_.getTopic() + " has no subscriber"), "",
                             QMessageBox::Icon::Warning);
  }

  pub_.publish(params_);
  ros::spinOnce();
  Q_EMIT enable(true);
}

void FillTrajectory::load(const rviz::Config& config)
{
  rviz::Panel::load(config);

  int tmp_int(0);
  float tmp_float(0.01);

  if (config.mapGetInt("movement_type", &tmp_int))
    movement_type_->setCurrentIndex(tmp_int);

  if (config.mapGetInt("approach_type", &tmp_int))
    approach_type_->setCurrentIndex(tmp_int);

  if (config.mapGetFloat("blend_radius", &tmp_float))
    blend_radius_->setValue(tmp_float);

  if (config.mapGetFloat("speed", &tmp_float))
    speed_->setValue(tmp_float);

  if (config.mapGetFloat("laser_power", &tmp_float))
    laser_power_->setValue(tmp_float);

  if (config.mapGetFloat("feed_rate", &tmp_float))
    feed_rate_->setValue(tmp_float);


  Q_EMIT approachTypeChanged();
  QtConcurrent::run(this, &FillTrajectory::sendLoadedInformation);
}

void FillTrajectory::sendLoadedInformation()
{
  updateInternalParameters();

  // Try to send parameters
  unsigned count(0);
  while (1)
  {
    if (pub_.getNumSubscribers() != 0)
    {
      Q_EMIT enable(false);
      pub_.publish(params_);
      ros::spinOnce();
      Q_EMIT enable(true);
      break;
    }
    ros::Duration(0.5).sleep();

    if (++count > 5)
      break;
  }
}

void FillTrajectory::save(rviz::Config config) const
                          {
  config.mapSetValue("movement_type", movement_type_->currentIndex());
  config.mapSetValue("approach_type", approach_type_->currentIndex());
  config.mapSetValue("blend_radius", blend_radius_->value());
  config.mapSetValue("speed", speed_->value());
  config.mapSetValue("laser_power", laser_power_->value());
  config.mapSetValue("feed_rate", feed_rate_->value());
  rviz::Panel::save(config);
}

void FillTrajectory::displayMessageBoxHandler(const QString title,
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

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ram_qt_guis::FillTrajectory, rviz::Panel)
