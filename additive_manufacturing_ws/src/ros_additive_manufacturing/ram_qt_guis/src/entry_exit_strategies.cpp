#include <ram_qt_guis/entry_exit_strategies.hpp>

namespace ram_qt_guis
{
EntryExitStrategies::EntryExitStrategies(QWidget* parent) :
        rviz::Panel(parent)
{
  connect(this, SIGNAL(enable(const bool)), this, SLOT(setEnabled(const bool)));
  setObjectName("Entry, exit strategies");
  setName(objectName());

  //  Entry
  entry_number_of_poses_ = new QSpinBox;
  entry_number_of_poses_->setRange(1, 100);

  entry_angle_ = new QDoubleSpinBox;
  entry_angle_->setRange(-180, 180);
  entry_angle_->setSingleStep(10);
  entry_angle_->setSuffix(" °");

  entry_distance_ = new QDoubleSpinBox;
  entry_distance_->setRange(0, 1000);
  entry_distance_->setSingleStep(1);
  entry_distance_->setSuffix(" mm");

  QGridLayout* entry_layout = new QGridLayout;
  entry_layout->addWidget(new QLabel("Number of poses:"), 0, 0);
  entry_layout->addWidget(entry_number_of_poses_, 0, 1);
  entry_layout->addWidget(new QLabel("Angle:"));
  entry_layout->addWidget(entry_angle_);
  entry_layout->addWidget(new QLabel("Distance:"));
  entry_layout->addWidget(entry_distance_);

  // Exit
  exit_number_of_poses_ = new QSpinBox;
  exit_number_of_poses_->setRange(1, 100);

  exit_angle_ = new QDoubleSpinBox;
  exit_angle_->setRange(-180, 180);
  exit_angle_->setSingleStep(10);
  exit_angle_->setSuffix(" °");

  exit_distance_ = new QDoubleSpinBox;
  exit_distance_->setRange(0.0, 1000);
  exit_distance_->setSingleStep(1);
  exit_distance_->setSuffix(" mm");

  entry_exit_button_ = new QPushButton("Entry / exit strategies");

  QGridLayout* exit_layout = new QGridLayout;
  exit_layout->addWidget(new QLabel("Number of poses:"), 0, 0);
  exit_layout->addWidget(exit_number_of_poses_, 0, 1);
  exit_layout->addWidget(new QLabel("Angle:"));
  exit_layout->addWidget(exit_angle_);
  exit_layout->addWidget(new QLabel("Distance:"));
  exit_layout->addWidget(exit_distance_);

  // Scroll area
  QVBoxLayout *scroll_widget_layout = new QVBoxLayout();
  QWidget *scroll_widget = new QWidget;
  scroll_widget->setLayout(scroll_widget_layout);
  QScrollArea *scroll_area = new QScrollArea;
  scroll_area->setWidget(scroll_widget);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->addWidget(scroll_area);
  scroll_widget_layout->addWidget(new QLabel("<b>Entry strategy</b>"));
  scroll_widget_layout->addLayout(entry_layout);
  scroll_widget_layout->addStretch(1);
  scroll_widget_layout->addWidget(new QLabel("<b>Exit strategy</b>"));
  scroll_widget_layout->addLayout(exit_layout);
  scroll_widget_layout->addStretch(1);
  scroll_widget_layout->addWidget(entry_exit_button_);
  scroll_widget_layout->addStretch(1);

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, SIGNAL(displayMessageBox(const QString, const QString, const QString, const QMessageBox::Icon)),
          this, SLOT(displayMessageBoxHandler(const QString, const QString, const QString, const QMessageBox::Icon)));

  connect(entry_exit_button_, SIGNAL(clicked()), this, SLOT(sendEntryExitParameters()));

  // connect with configChanged signal
  connect(entry_number_of_poses_, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
  connect(entry_angle_, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
  connect(entry_distance_, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

  connect(exit_number_of_poses_, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
  connect(exit_angle_, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
  connect(exit_distance_, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

  // Setup service clients
  entry_parameters_client_ = nh_.serviceClient<ram_utils::EntryExitParameters>("ram/information/entry_parameters");
  exit_parameters_client_ = nh_.serviceClient<ram_utils::EntryExitParameters>("ram/information/exit_parameters");

  // Check connection of clients
  connectToServices();
}

EntryExitStrategies::~EntryExitStrategies()
{
}

void EntryExitStrategies::updateInternalParameters()
{
  Q_EMIT configChanged();

  entry_parameters_.request.angle = entry_angle_->value() * M_PI / 180.0; // Convert in radians
  entry_parameters_.request.distance = entry_distance_->value() / 1000.0; // Convert meters in millimeters
  entry_parameters_.request.number_of_poses = entry_number_of_poses_->value();

  exit_parameters_.request.angle = exit_angle_->value() * M_PI / 180.0;
  exit_parameters_.request.distance = exit_distance_->value() / 1000.0;
  exit_parameters_.request.number_of_poses = exit_number_of_poses_->value();
}

void EntryExitStrategies::connectToService(ros::ServiceClient &client)
{
  Q_EMIT enable(false);

  while (nh_.ok())
  {
    if (client.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM(
                      "RViz panel " << getName().toStdString() << " connected to the service " << client.getService());
      break;
    }
    else
    {
      ROS_ERROR_STREAM(
          "RViz panel " << getName().toStdString() << " could not connect to ROS service: " << client.getService());
      ros::Duration(1).sleep();
    }
  }
}

void EntryExitStrategies::connectToServices()
{
  Q_EMIT enable(false);

  connectToService(entry_parameters_client_);
  connectToService(exit_parameters_client_);

  ROS_INFO_STREAM("RViz panel " << getName().toStdString() << " services connections have been made");
  Q_EMIT enable(true);
}

void EntryExitStrategies::displayMessageBoxHandler(const QString title,
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

void EntryExitStrategies::sendEntryExitParameters()
{
  Q_EMIT enable(false);

  // Call service
  updateInternalParameters();
  bool success(entry_parameters_client_.call(entry_parameters_));

  if (!success)
  {
    Q_EMIT displayMessageBox("Service call failed", QString::fromStdString(entry_parameters_client_.getService()),
                                  "Check the logs!", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  // Call service
  updateInternalParameters();
  success = exit_parameters_client_.call(exit_parameters_);

  if (!success)
  {
    Q_EMIT displayMessageBox("Service call failed",
                                  QString::fromStdString(exit_parameters_client_.getService()),
                                  "Check the logs!", QMessageBox::Icon::Critical);
    Q_EMIT enable(true);
    return;
  }

  Q_EMIT enable(true);
}

void EntryExitStrategies::load(const rviz::Config& config)
{
  int tmp_int(0);
  float tmp_float(0.01);
  Q_EMIT configChanged();

  if (config.mapGetInt("entry_number_of_poses", &tmp_int))
    entry_number_of_poses_->setValue(tmp_int);
  else
    entry_number_of_poses_->setValue(1);

  if (config.mapGetFloat("entry_angle", &tmp_float))
    entry_angle_->setValue(tmp_float);
  else
    entry_angle_->setValue(90);

  if (config.mapGetFloat("entry_distance", &tmp_float))
    entry_distance_->setValue(tmp_float);
  else
    entry_distance_->setValue(10);

  if (config.mapGetInt("exit_number_of_poses", &tmp_int))
    exit_number_of_poses_->setValue(tmp_int);
  else
    exit_number_of_poses_->setValue(1);

  if (config.mapGetFloat("exit_angle", &tmp_float))
    exit_angle_->setValue(tmp_float);
  else
    exit_angle_->setValue(90);

  if (config.mapGetFloat("exit_distance", &tmp_float))
    exit_distance_->setValue(tmp_float);
  else
    exit_distance_->setValue(10);

  Q_EMIT sendEntryExitParameters();
  rviz::Panel::load(config);
}

void EntryExitStrategies::save(rviz::Config config) const
                               {
  config.mapSetValue("entry_number_of_poses", entry_number_of_poses_->value());
  config.mapSetValue("entry_angle", entry_angle_->value());
  config.mapSetValue("entry_distance", entry_distance_->value());

  config.mapSetValue("exit_number_of_poses", exit_number_of_poses_->value());
  config.mapSetValue("exit_angle", exit_angle_->value());
  config.mapSetValue("exit_distance", exit_distance_->value());

  rviz::Panel::save(config);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ram_qt_guis::EntryExitStrategies, rviz::Panel)
