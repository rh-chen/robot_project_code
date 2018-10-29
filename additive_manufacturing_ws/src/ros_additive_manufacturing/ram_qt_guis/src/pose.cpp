#include <ram_qt_guis/pose.hpp>

namespace ram_qt_guis
{

Pose::Pose(const QString name,
           const QString description,
           const Pose::Mode mode,
           const Pose::PoseType pose_type,
           const Eigen::Isometry3d &pose,
           QWidget *parent):
  QWidget(parent),
  mode_(mode),
  pose_(pose)
{
  setObjectName(name);

  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  QHBoxLayout* label_combo(new QHBoxLayout);
  label_combo->addWidget(new QLabel(description));
  label_combo->addStretch(1);
  layout->addLayout(label_combo);

  mode_select_ = new QComboBox;
  mode_select_->addItem("Matrix");
  mode_select_->addItem("XYZWPR");
  mode_select_->setToolTip("Fanuc and Yaskawa Motoman");
  label_combo->addWidget(mode_select_);
  QStackedWidget *stack = new QStackedWidget;
  connect(
    mode_select_, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
    [ = ](const int &id)
  {
    stack->setCurrentIndex(id);
    Q_EMIT valueChanged(pose_);
  }
  );

  layout->addWidget(stack);
  std::vector<QWidget *> widgets;

  {
    QWidget *widget(new QWidget);
    QGridLayout *layout(new QGridLayout);
    widget->setLayout(layout);
    QDoubleSpinBox *rot_0_0(new QDoubleSpinBox);
    rot_0_0->setValue(1);
    QDoubleSpinBox *rot_0_1(new QDoubleSpinBox);
    QDoubleSpinBox *rot_0_2(new QDoubleSpinBox);
    QDoubleSpinBox *rot_1_0(new QDoubleSpinBox);
    QDoubleSpinBox *rot_1_1(new QDoubleSpinBox);
    rot_1_1->setValue(1);
    QDoubleSpinBox *rot_1_2(new QDoubleSpinBox);
    QDoubleSpinBox *rot_2_0(new QDoubleSpinBox);
    QDoubleSpinBox *rot_2_1(new QDoubleSpinBox);
    QDoubleSpinBox *rot_2_2(new QDoubleSpinBox);
    rot_2_2->setValue(1);
    QDoubleSpinBox *trans_0(new QDoubleSpinBox);
    QDoubleSpinBox *trans_1(new QDoubleSpinBox);
    QDoubleSpinBox *trans_2(new QDoubleSpinBox);
    QDoubleSpinBox *fixed_0(new QDoubleSpinBox);
    QDoubleSpinBox *fixed_1(new QDoubleSpinBox);
    QDoubleSpinBox *fixed_2(new QDoubleSpinBox);
    QDoubleSpinBox *fixed_3(new QDoubleSpinBox);
    fixed_0->setEnabled(false);
    fixed_1->setEnabled(false);
    fixed_2->setEnabled(false);
    fixed_3->setEnabled(false);
    fixed_3->setValue(1);

    const int min_width(20);
    rot_0_0->setMinimumWidth(min_width);
    rot_0_1->setMinimumWidth(min_width);
    rot_0_2->setMinimumWidth(min_width);
    rot_1_0->setMinimumWidth(min_width);
    rot_1_1->setMinimumWidth(min_width);
    rot_1_2->setMinimumWidth(min_width);
    rot_2_0->setMinimumWidth(min_width);
    rot_2_1->setMinimumWidth(min_width);
    rot_2_2->setMinimumWidth(min_width);
    trans_0->setMinimumWidth(min_width);
    trans_1->setMinimumWidth(min_width);
    trans_2->setMinimumWidth(min_width);
    fixed_0->setMinimumWidth(min_width);
    fixed_1->setMinimumWidth(min_width);
    fixed_2->setMinimumWidth(min_width);
    fixed_3->setMinimumWidth(min_width);

    const int max_height(25);
    rot_0_0->setMaximumHeight(max_height);
    rot_0_1->setMaximumHeight(max_height);
    rot_0_2->setMaximumHeight(max_height);
    rot_1_0->setMaximumHeight(max_height);
    rot_1_1->setMaximumHeight(max_height);
    rot_1_2->setMaximumHeight(max_height);
    rot_2_0->setMaximumHeight(max_height);
    rot_2_1->setMaximumHeight(max_height);
    rot_2_2->setMaximumHeight(max_height);
    trans_0->setMaximumHeight(max_height);
    trans_1->setMaximumHeight(max_height);
    trans_2->setMaximumHeight(max_height);
    fixed_0->setMaximumHeight(max_height);
    fixed_1->setMaximumHeight(max_height);
    fixed_2->setMaximumHeight(max_height);
    fixed_3->setMaximumHeight(max_height);

    rot_0_0->setRange(-100, 100);
    rot_0_1->setRange(-100, 100);
    rot_0_2->setRange(-100, 100);
    rot_1_0->setRange(-100, 100);
    rot_1_1->setRange(-100, 100);
    rot_1_2->setRange(-100, 100);
    rot_2_0->setRange(-100, 100);
    rot_2_1->setRange(-100, 100);
    rot_2_2->setRange(-100, 100);
    rot_0_0->setDecimals(5);
    rot_0_1->setDecimals(5);
    rot_0_2->setDecimals(5);
    rot_1_0->setDecimals(5);
    rot_1_1->setDecimals(5);
    rot_1_2->setDecimals(5);
    rot_2_0->setDecimals(5);
    rot_2_1->setDecimals(5);
    rot_2_2->setDecimals(5);
    rot_0_0->setSingleStep(0.1);
    rot_0_1->setSingleStep(0.1);
    rot_0_2->setSingleStep(0.1);
    rot_1_0->setSingleStep(0.1);
    rot_1_1->setSingleStep(0.1);
    rot_1_2->setSingleStep(0.1);
    rot_2_0->setSingleStep(0.1);
    rot_2_1->setSingleStep(0.1);
    rot_2_2->setSingleStep(0.1);

    trans_0->setRange(-100, 100);
    trans_1->setRange(-100, 100);
    trans_2->setRange(-100, 100);
    trans_0->setSingleStep(0.01);
    trans_1->setSingleStep(0.01);
    trans_2->setSingleStep(0.01);
    trans_0->setDecimals(5);
    trans_1->setDecimals(5);
    trans_2->setDecimals(5);

    layout->addWidget(rot_0_0, 0, 0);
    layout->addWidget(rot_0_1, 0, 1);
    layout->addWidget(rot_0_2, 0, 2);
    layout->addWidget(rot_1_0, 1, 0);
    layout->addWidget(rot_1_1, 1, 1);
    layout->addWidget(rot_1_2, 1, 2);
    layout->addWidget(rot_2_0, 2, 0);
    layout->addWidget(rot_2_1, 2, 1);
    layout->addWidget(rot_2_2, 2, 2);
    layout->addWidget(trans_0, 0, 3);
    layout->addWidget(trans_1, 1, 3);
    layout->addWidget(trans_2, 2, 3);
    layout->addWidget(fixed_0, 3, 0);
    layout->addWidget(fixed_1, 3, 1);
    layout->addWidget(fixed_2, 3, 2);
    layout->addWidget(fixed_3, 3, 3);

    if (mode == READ_ONLY)
    {
      rot_0_0->setEnabled(false);
      rot_0_1->setEnabled(false);
      rot_0_2->setEnabled(false);
      rot_1_0->setEnabled(false);
      rot_1_1->setEnabled(false);
      rot_1_2->setEnabled(false);
      rot_2_0->setEnabled(false);
      rot_2_1->setEnabled(false);
      rot_2_2->setEnabled(false);
      trans_0->setEnabled(false);
      trans_1->setEnabled(false);
      trans_2->setEnabled(false);
    }
    else if (mode == READ_WRITE_ORIENTATION)
    {
      trans_0->setEnabled(false);
      trans_1->setEnabled(false);
      trans_2->setEnabled(false);
    }

    matrix_.emplace_back(rot_0_0);
    matrix_.emplace_back(rot_0_1);
    matrix_.emplace_back(rot_0_2);
    matrix_.emplace_back(rot_1_0);
    matrix_.emplace_back(rot_1_1);
    matrix_.emplace_back(rot_1_2);
    matrix_.emplace_back(rot_2_0);
    matrix_.emplace_back(rot_2_1);
    matrix_.emplace_back(rot_2_2);
    matrix_.emplace_back(trans_0);
    matrix_.emplace_back(trans_1);
    matrix_.emplace_back(trans_2);

    for (auto &widget : matrix_)
      connect(widget, SIGNAL(valueChanged(double)), this, SLOT(matrixChanged()));

    widgets.emplace_back(widget);
  }
  {
    QWidget *widget(new QWidget);
    QHBoxLayout *layout(new QHBoxLayout);
    QGridLayout *left_column(new QGridLayout);
    QGridLayout *right_column(new QGridLayout);
    layout->addLayout(left_column);
    layout->addStretch(1);
    layout->addLayout(right_column);
    widget->setLayout(layout);

    QDoubleSpinBox *x_mm(new QDoubleSpinBox);
    x_mm->setSuffix(" mm");
    QDoubleSpinBox *y_mm(new QDoubleSpinBox);
    y_mm->setSuffix(" mm");
    QDoubleSpinBox *z_mm(new QDoubleSpinBox);
    z_mm->setSuffix(" mm");
    QDoubleSpinBox *w_deg(new QDoubleSpinBox);
    w_deg->setSuffix(" °");
    QDoubleSpinBox *p_deg(new QDoubleSpinBox);
    p_deg->setSuffix(" °");
    QDoubleSpinBox *r_deg(new QDoubleSpinBox);
    r_deg->setSuffix(" °");

    x_mm->setRange(-100000, 100000);
    y_mm->setRange(-100000, 100000);
    z_mm->setRange(-100000, 100000);
    w_deg->setRange(-180, 180);
    p_deg->setRange(-180, 180);
    r_deg->setRange(-180, 180);

    left_column->addWidget(new QLabel("X"), 0, 0);
    left_column->addWidget(new QLabel("Y"), 1, 0);
    left_column->addWidget(new QLabel("Z"), 2, 0);
    left_column->addWidget(x_mm, 0, 1);
    left_column->addWidget(y_mm, 1, 1);
    left_column->addWidget(z_mm, 2, 1);
    right_column->addWidget(new QLabel("W"), 0, 0);
    right_column->addWidget(new QLabel("P"), 1, 0);
    right_column->addWidget(new QLabel("R"), 2, 0);
    right_column->addWidget(w_deg, 0, 1);
    right_column->addWidget(p_deg, 1, 1);
    right_column->addWidget(r_deg, 2, 1);

    if (mode == READ_ONLY)
    {
      x_mm->setEnabled(false);
      y_mm->setEnabled(false);
      z_mm->setEnabled(false);
      w_deg->setEnabled(false);
      p_deg->setEnabled(false);
      r_deg->setEnabled(false);
    }
    else if (mode == READ_WRITE_ORIENTATION)
    {
      x_mm->setEnabled(false);
      y_mm->setEnabled(false);
      z_mm->setEnabled(false);
    }

    xyzwpr_.emplace_back(x_mm);
    xyzwpr_.emplace_back(y_mm);
    xyzwpr_.emplace_back(z_mm);
    xyzwpr_.emplace_back(w_deg);
    xyzwpr_.emplace_back(p_deg);
    xyzwpr_.emplace_back(r_deg);

    widgets.emplace_back(widget);
  }
  {
    QWidget *widget(new QWidget);
    QGridLayout *layout(new QGridLayout);
    widget->setLayout(layout);
    layout->addWidget(new QLabel("Yaskawa"));
    widgets.emplace_back(widget);
  }

  for (auto &widget : widgets)
    stack->addWidget(widget);
  mode_select_->setCurrentIndex(pose_type);

  updatePose(pose_);
  connect(this, &Pose::setPose, this, &Pose::updatePose);
  connect(mode_select_, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          this, &Pose::modeChanged);
}

Eigen::Isometry3d Pose::pose()
{
  return pose_;
}

void Pose::save(rviz::Config config)
{
  config.mapSetValue(objectName() + "_mode", mode_select_->currentIndex());
  config.mapSetValue(objectName() + "_x", pose_.translation().x());
  config.mapSetValue(objectName() + "_y", pose_.translation().y());
  config.mapSetValue(objectName() + "_z", pose_.translation().z());
  Eigen::Quaterniond q(pose_.linear());
  config.mapSetValue(objectName() + "_qx", q.x());
  config.mapSetValue(objectName() + "_qy", q.y());
  config.mapSetValue(objectName() + "_qz", q.z());
  config.mapSetValue(objectName() + "_qw", q.w());
}

void Pose::load(const rviz::Config &config)
{
  int mode;
  if (config.mapGetInt(objectName() + "_mode", &mode))
    mode_select_->setCurrentIndex(mode);

  float x, y, z, qx, qy, qz, qw;
  if (!config.mapGetFloat(objectName() + "_x", &x))
    return;
  if (!config.mapGetFloat(objectName() + "_y", &y))
    return;
  if (!config.mapGetFloat(objectName() + "_z", &z))
    return;
  if (!config.mapGetFloat(objectName() + "_qx", &qx))
    return;
  if (!config.mapGetFloat(objectName() + "_qy", &qy))
    return;
  if (!config.mapGetFloat(objectName() + "_qz", &qz))
    return;
  if (!config.mapGetFloat(objectName() + "_qw", &qw))
    return;

  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).matrix(); // Mind the initialization order!
  pose.translation().x() = x;
  pose.translation().y() = y;
  pose.translation().z() = z;
  updatePose(pose);
}

void Pose::modeChanged(const unsigned index)
{
  for (auto &widget : matrix_)
    widget->disconnect();
  for (auto &widget : xyzwpr_)
    widget->disconnect();

  if (index == 0)
  {
    updateMatrixGUI();
    for (auto &widget : matrix_)
      connect(widget, SIGNAL(valueChanged(double)), this, SLOT(matrixChanged()));
  }
  else if (index == 1)
  {
    updateXYZWPRGUI();
    for (auto &widget : xyzwpr_)
      connect(widget, SIGNAL(valueChanged(double)), this, SLOT(XYZWPRChanged()));
  }
}

void Pose::updatePose(const Eigen::Isometry3d &m)
{
  pose_ = m;
  Q_EMIT modeChanged(mode_select_->currentIndex());
}

void Pose::matrixChanged()
{
  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  pose.matrix().row(0) = Eigen::Vector4d(
                           matrix_.at(0)->value(),
                           matrix_.at(1)->value(),
                           matrix_.at(2)->value(),
                           matrix_.at(9)->value());
  pose.matrix().row(1) = Eigen::Vector4d(
                           matrix_.at(3)->value(),
                           matrix_.at(4)->value(),
                           matrix_.at(5)->value(),
                           matrix_.at(10)->value());
  pose.matrix().row(2) = Eigen::Vector4d(
                           matrix_.at(6)->value(),
                           matrix_.at(7)->value(),
                           matrix_.at(8)->value(),
                           matrix_.at(11)->value());

  pose_ = pose;
  Q_EMIT valueChanged(pose_);
}

void Pose::XYZWPRChanged()
{

  using Eigen::Affine3d;
  using Eigen::Matrix3d;
  using Eigen::Vector3d;
  using Eigen::AngleAxisd;

  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  pose.translation()[0] = xyzwpr_.at(0)->value() / 1000.0;
  pose.translation()[1] = xyzwpr_.at(1)->value() / 1000.0;
  pose.translation()[2] = xyzwpr_.at(2)->value() / 1000.0;
  const double deg_to_rad(M_PI / 180.0);
  Matrix3d rot;
  rot = AngleAxisd(xyzwpr_.at(3)->value() * deg_to_rad, Vector3d::UnitX())
      * AngleAxisd(xyzwpr_.at(4)->value() * deg_to_rad, Vector3d::UnitY())
      * AngleAxisd(xyzwpr_.at(5)->value() * deg_to_rad, Vector3d::UnitZ());
  pose.linear() = rot;
  pose_ = pose;
  Q_EMIT valueChanged(pose_);
}

void Pose::updateMatrixGUI()
{
  matrix_.at(0)->setValue(pose_.matrix().row(0)[0]); // rot_0_0
  matrix_.at(1)->setValue(pose_.matrix().row(0)[1]); // rot_0_1
  matrix_.at(2)->setValue(pose_.matrix().row(0)[2]); // rot_0_2
  matrix_.at(3)->setValue(pose_.matrix().row(1)[0]); // rot_1_0
  matrix_.at(4)->setValue(pose_.matrix().row(1)[1]); // rot_1_1
  matrix_.at(5)->setValue(pose_.matrix().row(1)[2]); // rot_1_2
  matrix_.at(6)->setValue(pose_.matrix().row(2)[0]); // rot_2_0
  matrix_.at(7)->setValue(pose_.matrix().row(2)[1]); // rot_2_1
  matrix_.at(8)->setValue(pose_.matrix().row(2)[2]); // rot_2_2
  matrix_.at(9)->setValue(pose_.matrix().col(3)[0]); // trans_0
  matrix_.at(10)->setValue(pose_.matrix().col(3)[1]); // trans_1
  matrix_.at(11)->setValue(pose_.matrix().col(3)[2]); // trans_2
}

void Pose::updateXYZWPRGUI()
{
  xyzwpr_.at(0)->setValue(pose_.translation()[0] * 1000.0);
  xyzwpr_.at(1)->setValue(pose_.translation()[1] * 1000.0);
  xyzwpr_.at(2)->setValue(pose_.translation()[2] * 1000.0);

  const Eigen::Vector3d euler_angles(pose_.linear().eulerAngles(0, 1, 2));
  const Eigen::Vector3d euler_angles_deg(euler_angles * 180.0 / M_PI);
  xyzwpr_.at(3)->setValue(euler_angles_deg[0]); // yaW
  xyzwpr_.at(4)->setValue(euler_angles_deg[1]); // Pitch
  xyzwpr_.at(5)->setValue(euler_angles_deg[2]); // Roll
}

}
