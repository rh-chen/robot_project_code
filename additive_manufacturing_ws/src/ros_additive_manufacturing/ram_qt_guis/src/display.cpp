#include <ram_qt_guis/display.hpp>

namespace ram_qt_guis
{
Display::Display(QWidget* parent) :
        rviz::Panel(parent)
{
  connect(this, SIGNAL(enable(const bool)), this, SLOT(setEnabled(const bool)));
  setObjectName("Display");
  setName(objectName());

  color_mode_ = new QComboBox;
  color_mode_->addItem("Layer level");
  color_mode_->addItem("Layer index");
  color_mode_->addItem("Speed");
  color_mode_->addItem("Pose type");
  color_mode_->addItem("Laser power");
  QHBoxLayout *color_mode_layout = new QHBoxLayout;
  color_mode_layout->addWidget(new QLabel("Color by:"));
  color_mode_layout->addWidget(color_mode_);

  display_mode_ = new QComboBox;
  display_mode_->addItem("Wire-frame mode");
  display_mode_->addItem("Cylinders mode");
  display_mode_->addItem("Wire-frame + axis mode");
  display_mode_->addItem("Cylinders + axis mode");
  QHBoxLayout *display_mode_layout = new QHBoxLayout;
  display_mode_layout->addWidget(new QLabel("Display mode:"));
  display_mode_layout->addWidget(display_mode_);
  connect(display_mode_, SIGNAL(currentIndexChanged(int)), this, SLOT(displayModeChanged()));

  cylinder_size_ = new QDoubleSpinBox;
  cylinder_size_->setRange(0.01, 1000.0);
  cylinder_size_->setSingleStep(0.1);
  cylinder_size_->setValue(1);
  cylinder_size_->setSuffix(" mm");
  QHBoxLayout *cylinder_size_layout = new QHBoxLayout;
  cylinder_size_layout->addWidget(new QLabel("Cylinder size:"));
  cylinder_size_layout->addWidget(cylinder_size_);
  cylinder_size_widget_ = new QWidget;
  cylinder_size_widget_->setLayout(cylinder_size_layout);

  wire_size_ = new QDoubleSpinBox;
  wire_size_->setRange(0.01, 1000.0);
  wire_size_->setSingleStep(0.1);
  wire_size_->setValue(1);
  wire_size_->setSuffix(" mm");
  QHBoxLayout *wire_size_layout = new QHBoxLayout;
  wire_size_layout->addWidget(new QLabel("Wire size:"));
  wire_size_layout->addWidget(wire_size_);
  wire_size_widget_ = new QWidget;
  wire_size_widget_->setLayout(wire_size_layout);

  axis_size_ = new QDoubleSpinBox;
  axis_size_->setRange(0.01, 5000.0);
  axis_size_->setSingleStep(1);
  axis_size_->setValue(2);
  axis_size_->setSuffix(" mm");
  QHBoxLayout *axis_size_layout = new QHBoxLayout;
  axis_size_layout->addWidget(new QLabel("Axis size:"));
  axis_size_layout->addWidget(axis_size_);
  axis_size_widget_ = new QWidget;
  axis_size_widget_->setLayout(axis_size_layout);

  display_labels_ = new QCheckBox;
  display_labels_->setText("Display labels");
  display_labels_->setChecked(true);
  connect(display_labels_, SIGNAL(clicked()), this, SLOT(enableLabelsBox()));

  label_type_ = new QComboBox;
  label_type_->addItem("Pose ID");
  label_type_->addItem("Layer level");
  label_type_->addItem("Layer index");
  label_type_->addItem("Pose ID within layer");

  label_text_size_ = new QDoubleSpinBox;
  label_text_size_->setRange(0.01, 1000.0);
  label_text_size_->setSingleStep(0.1);
  label_text_size_->setValue(1);
  label_text_size_->setSuffix(" mm");

  labels_container_ = new QWidget;
  labels_container_->setEnabled(false);
  QGridLayout* labels_container_layout = new QGridLayout(labels_container_);
  labels_container_layout->addWidget(new QLabel("Labels type:"), 0, 0);
  labels_container_layout->addWidget(label_type_, 0, 1);
  labels_container_layout->addWidget(new QLabel("Labels size:"));
  labels_container_layout->addWidget(label_text_size_);

  range_of_layers_ = new QCheckBox("Display a range of layers (levels)");
  range_of_layers_->setChecked(true);

  first_layer_ = new QSpinBox;
  first_layer_->setRange(0, 50000);
  first_layer_->setValue(0);

  last_layer_ = new QSpinBox;
  last_layer_->setRange(0, 50000);
  last_layer_->setValue(0);

  range_of_layers_container_ = new QWidget;
  range_of_layers_container_->setEnabled(false);
  QGridLayout* range_of_layers_layout = new QGridLayout(range_of_layers_container_);
  range_of_layers_layout->addWidget(new QLabel("First layer:"), 0, 0);
  range_of_layers_layout->addWidget(first_layer_, 0, 1);
  range_of_layers_layout->addWidget(new QLabel("Last layer:"), 1, 0);
  range_of_layers_layout->addWidget(last_layer_, 1, 1);
  connect(first_layer_, SIGNAL(valueChanged(int)), this, SLOT(tweakFirstLayer()));
  connect(last_layer_, SIGNAL(valueChanged(int)), this, SLOT(tweakLastLayer()));
  connect(range_of_layers_, SIGNAL(clicked()), this, SLOT(enableRangeLimitsBox()));

  mesh_color_ = new QPushButton("Pick mesh color");

  display_button_ = new QPushButton("Display markers");
  delete_button_ = new QPushButton("Clear markers");
  QHBoxLayout *buttons = new QHBoxLayout;
  buttons->addWidget(display_button_);
  buttons->addWidget(delete_button_);

  QVBoxLayout *scroll_widget_layout = new QVBoxLayout();
  QWidget *scroll_widget = new QWidget;
  scroll_widget->setLayout(scroll_widget_layout);
  QScrollArea *scroll_area = new QScrollArea;
  scroll_area->setWidget(scroll_widget);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->addWidget(scroll_area);

  scroll_widget_layout->addLayout(color_mode_layout);
  scroll_widget_layout->addLayout(display_mode_layout);
  scroll_widget_layout->addWidget(cylinder_size_widget_);
  scroll_widget_layout->addWidget(wire_size_widget_);
  scroll_widget_layout->addWidget(axis_size_widget_);
  scroll_widget_layout->addWidget(display_labels_);
  scroll_widget_layout->addWidget(labels_container_);
  scroll_widget_layout->addWidget(range_of_layers_);
  scroll_widget_layout->addWidget(range_of_layers_container_);
  scroll_widget_layout->addWidget(mesh_color_);
  scroll_widget_layout->addStretch(1);

  main_layout->addLayout(buttons);

  connect(mesh_color_, SIGNAL(clicked()), this, SLOT(pickColor()));
  connect(display_button_, SIGNAL(clicked()), this, SLOT(sendDisplayInformationButtonHandler()));
  connect(delete_button_, SIGNAL(clicked()), this, SLOT(sendDeleteInformationButtonHandler()));

  qRegisterMetaType<QMessageBox::Icon>();
  connect(this, SIGNAL(displayMessageBox(const QString, const QString, const QString, const QMessageBox::Icon)),
          this, SLOT(displayMessageBoxHandler(const QString, const QString, const QString, const QMessageBox::Icon)));

  // connect with configChanged signal
  connect(color_mode_, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged())); // Combo box
  connect(display_mode_, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
  connect(cylinder_size_, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged())); // double spin box
  connect(wire_size_, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged())); // double spin box
  connect(axis_size_, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));  // double spin box
  connect(display_labels_, SIGNAL(clicked()), this, SIGNAL(configChanged())); // QCheckBox
  connect(label_type_, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged())); // Combo box
  connect(label_text_size_, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged())); // double spin box
  connect(range_of_layers_, SIGNAL(clicked()), this, SIGNAL(configChanged()));
  connect(first_layer_, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
  connect(last_layer_, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));

  // Setup service clients
  display_client_ = nh_.serviceClient<ram_display::DisplayTrajectory>("ram/display/add_trajectory");
  delete_client_ = nh_.serviceClient<ram_display::DeleteTrajectory>("ram/display/delete_trajectory");
  update_mesh_color_client_ = nh_.serviceClient<ram_display::UpdateMeshColor>("ram/display/update_mesh_color");

  // Check connection of client
  QtConcurrent::run(this, &Display::connectToServices);
}

Display::~Display()
{
}

void Display::connectToService(ros::ServiceClient &client)
{
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

void Display::connectToServices()
{
  Q_EMIT enable(false);
  connectToService(display_client_);
  connectToService(delete_client_);
  connectToService(update_mesh_color_client_);
  ROS_INFO_STREAM("RViz panel " << getName().toStdString() << " services connections have been made");
  Q_EMIT enable(true);
}

void Display::updateInternalParameters()
{
  params_.request.axis_size = axis_size_->value() / 1000.0;
  params_.request.cylinder_size = cylinder_size_->value() / 1000.0;
  params_.request.label_type = label_type_->currentIndex();
  params_.request.display_mode = display_mode_->currentIndex();
  params_.request.labels_size = label_text_size_->value() / 1000.0;
  params_.request.color_mode = color_mode_->currentIndex();
  params_.request.wire_size = wire_size_->value() / 1000.0;
  params_.request.display_labels = display_labels_->isChecked();
  params_.request.display_range_of_layers = range_of_layers_->isChecked();
  params_.request.first_layer = first_layer_->value();
  params_.request.last_layer = last_layer_->value();
  // Color is being modified in pickColor();
}

void Display::displayModeChanged()
{
  switch (display_mode_->currentIndex())
  {
    case 0:
      // Wire-frame mode
      cylinder_size_widget_->setEnabled(false);
      wire_size_widget_->setEnabled(true);
      axis_size_widget_->setEnabled(false);
      break;
    case 1:
      // Cylinders mode
      cylinder_size_widget_->setEnabled(true);
      wire_size_widget_->setEnabled(false);
      axis_size_widget_->setEnabled(false);
      break;
    case 2:
      // Wire-frame + axis mode
      cylinder_size_widget_->setEnabled(false);
      wire_size_widget_->setEnabled(true);
      axis_size_widget_->setEnabled(true);
      break;
    case 3:
      // Cylinders + axis mode
      cylinder_size_widget_->setEnabled(true);
      wire_size_widget_->setEnabled(false);
      axis_size_widget_->setEnabled(true);
      break;
    default:
      cylinder_size_widget_->setEnabled(true);
      wire_size_widget_->setEnabled(true);
      axis_size_widget_->setEnabled(true);
      break;
  }
}

void Display::pickColor()
{
  Q_EMIT enable(false);

  QColorDialog color;
  color.setModal(true);

  QColor old_color((int)(params_.request.mesh_color.r * 255),
                   (int)(params_.request.mesh_color.g * 255),
                   (int)(params_.request.mesh_color.b * 255),
                   (int)(params_.request.mesh_color.a * 255));
  QColor rgba = color.getColor(old_color, this, "Mesh color", QColorDialog::ShowAlphaChannel);

  if (!rgba.isValid())
  {
    Q_EMIT enable(true);
    return;
  }

  params_.request.mesh_color.r = rgba.red() / 255.0;
  params_.request.mesh_color.g = rgba.green() / 255.0;
  params_.request.mesh_color.b = rgba.blue() / 255.0;
  params_.request.mesh_color.a = rgba.alpha() / 255.0;

  ram_display::UpdateMeshColor srv;
  srv.request.color = params_.request.mesh_color;
  if (!update_mesh_color_client_.call(srv))
    Q_EMIT displayMessageBox("Update mesh color", "Updating the mesh color failed",
                                  QString::fromStdString(update_mesh_color_client_.getService()),
                                  QMessageBox::Icon::Critical);

  Q_EMIT enable(true);
}

void Display::sendDisplayInformationButtonHandler()
{
  Q_EMIT configChanged();
  Q_EMIT enable(false);

  updateInternalParameters();

  // Run in a separate thread
  QtConcurrent::run(this, &Display::sendDisplayInformation);
}

void Display::sendDisplayInformation()
{
  // Call service
  bool success(display_client_.call(params_));

  if (!success)
  {
    Q_EMIT displayMessageBox("Service call failed", QString::fromStdString(display_client_.getService()),
                                  "Check the logs!", QMessageBox::Icon::Critical);
  }
  else
  {
    if (!params_.response.error.empty())
    {
      Q_EMIT displayMessageBox(QString::fromStdString(display_client_.getService()),
                                    QString::fromStdString(params_.response.error),
                                    "", QMessageBox::Icon::Critical);
    }
  }
  Q_EMIT enable(true);
}

void Display::sendDeleteInformationButtonHandler()
{
  Q_EMIT enable(false);
  // Run in a separate thread
  QtConcurrent::run(this, &Display::sendDeleteInformation);
}

void Display::sendDeleteInformation()
{
  // Call service
  ram_display::DeleteTrajectory delete_traj;
  bool success(delete_client_.call(delete_traj));

  if (!success)
  {
    Q_EMIT displayMessageBox("Service call failed", QString::fromStdString(delete_client_.getService()),
                                  "Check the logs!", QMessageBox::Icon::Critical);
  }
  Q_EMIT enable(true);
}

void Display::enableRangeLimitsBox()
{
  range_of_layers_container_->setEnabled(range_of_layers_->isChecked());
}

void Display::enableLabelsBox()
{
  labels_container_->setEnabled(display_labels_->isChecked());
}

void Display::tweakFirstLayer()
{
  if (first_layer_->value() > last_layer_->value())
    last_layer_->setValue(first_layer_->value());
}

void Display::tweakLastLayer()
{
  if (first_layer_->value() > last_layer_->value())
    first_layer_->setValue(last_layer_->value());
}

void Display::load(const rviz::Config& config)
{
  bool tmp_bool(false);
  int tmp_int(0);
  float tmp_float(0.01);

  if (config.mapGetInt("color_mode", &tmp_int))
    color_mode_->setCurrentIndex(tmp_int);

  if (config.mapGetInt("display_mode", &tmp_int))
    display_mode_->setCurrentIndex(tmp_int);

  if (config.mapGetFloat("cylinder_size", &tmp_float))
    cylinder_size_->setValue(tmp_float);

  if (config.mapGetFloat("wire_size", &tmp_float))
    wire_size_->setValue(tmp_float);

  if (config.mapGetFloat("label_text_size", &tmp_float))
    label_text_size_->setValue (tmp_float);

  tmp_float = 0.05;
  if (config.mapGetFloat("axis_size", &tmp_float))
    axis_size_->setValue(tmp_float);

  if (config.mapGetInt("display_pose_layer", &tmp_int))
    label_type_->setCurrentIndex(tmp_int);

  if (config.mapGetBool("display_labels", &tmp_bool))
    display_labels_->setChecked(tmp_bool);
  else
    display_labels_->setChecked(true);

  enableLabelsBox();

  if (config.mapGetBool("range_of_layers", &tmp_bool))
    range_of_layers_->setChecked(tmp_bool);
  else
    range_of_layers_->setChecked(false);

  enableRangeLimitsBox();

  if (config.mapGetInt("first_layer", &tmp_int))
    first_layer_->setValue(tmp_int);

  if (config.mapGetInt("last_layer", &tmp_int))
    last_layer_->setValue(tmp_int);

  if (config.mapGetFloat("mesh_red", &tmp_float))
    params_.request.mesh_color.r = tmp_float;
  else
    params_.request.mesh_color.r = 0.7;

  if (config.mapGetFloat("mesh_green", &tmp_float))
    params_.request.mesh_color.g = tmp_float;
  else
    params_.request.mesh_color.g = 0.7;

  if (config.mapGetFloat("mesh_blue", &tmp_float))
    params_.request.mesh_color.b = tmp_float;
  else
    params_.request.mesh_color.b = 0.7;

  if (config.mapGetFloat("mesh_alpha", &tmp_float))
    params_.request.mesh_color.a = tmp_float;
  else
    params_.request.mesh_color.a = 0.75;

  rviz::Panel::load(config);
  QtConcurrent::run(this, &Display::sendLoadedInformation);
}

void Display::sendLoadedInformation()
{
  updateInternalParameters();

  // Try to send parameters
  unsigned count(0);
  while (1)
  {
    if (display_client_.exists())
    {
      Q_EMIT enable(false);
      display_client_.call(params_);
      ros::spinOnce();
      Q_EMIT enable(true);
      break;
    }
    ros::Duration(0.5).sleep();

    if (++count > 5)
      break;
  }
}

void Display::save(rviz::Config config) const
                   {
  config.mapSetValue("color_mode", color_mode_->currentIndex());
  config.mapSetValue("display_mode", display_mode_->currentIndex());
  config.mapSetValue("cylinder_size", cylinder_size_->value());
  config.mapSetValue("wire_size", wire_size_->value());
  config.mapSetValue("axis_size", axis_size_->value());
  config.mapSetValue("display_pose_layer", label_type_->currentIndex());
  config.mapSetValue("label_text_size", label_text_size_->value());
  config.mapSetValue("display_labels", display_labels_->isChecked());
  config.mapSetValue("range_of_layers", range_of_layers_->isChecked());
  config.mapSetValue("first_layer", first_layer_->value());
  config.mapSetValue("last_layer", last_layer_->value());
  config.mapSetValue("mesh_red", params_.request.mesh_color.r);
  config.mapSetValue("mesh_green", params_.request.mesh_color.g);
  config.mapSetValue("mesh_blue", params_.request.mesh_color.b);
  config.mapSetValue("mesh_alpha", params_.request.mesh_color.a);
  rviz::Panel::save(config);
}

void Display::displayMessageBoxHandler(const QString title,
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
PLUGINLIB_EXPORT_CLASS(ram_qt_guis::Display, rviz::Panel)
