#ifndef RAM_QT_GUIS_FILL_TRAJECTORY_HPP
#define RAM_QT_GUIS_FILL_TRAJECTORY_HPP

#ifndef Q_MOC_RUN
#include <eigen_conversions/eigen_msg.h>
#include <ram_msgs/AdditiveManufacturingParams.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#endif

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QtConcurrent/QtConcurrentRun>

namespace ram_qt_guis
{
class FillTrajectory : public rviz::Panel
{
Q_OBJECT
  public:
  FillTrajectory(QWidget* parent = NULL);
  virtual ~FillTrajectory();

Q_SIGNALS:
  void enable(const bool);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

private:
  void updateInternalParameters();

protected Q_SLOTS:
  void movementTypeChanged();
  void approachTypeChanged();
  void sendInformationButtonHandler();
  void sendInformation();

  void load(const rviz::Config& config);
  void sendLoadedInformation();
  void save(rviz::Config config) const;

  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);

protected:
  QComboBox *movement_type_;
  QComboBox *approach_type_;
  QSpinBox *laser_power_;
  QDoubleSpinBox *feed_rate_;
  QSpinBox *blend_radius_;
  QDoubleSpinBox *speed_;
  QPushButton *send_button_;

  double conversion_factor_; //Used by speed_

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ram_msgs::AdditiveManufacturingParams params_;
};

}

#endif
