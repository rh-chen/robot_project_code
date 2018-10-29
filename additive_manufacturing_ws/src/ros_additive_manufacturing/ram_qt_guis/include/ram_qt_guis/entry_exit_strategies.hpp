#ifndef RAM_QT_GUIS_ENTRY_EXIT_STRATEGIES_HPP
#define RAM_QT_GUIS_ENTRY_EXIT_STRATEGIES_HPP

#ifndef Q_MOC_RUN
#include <eigen_conversions/eigen_msg.h>
#include <ram_utils/EntryExitParameters.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#endif

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QtConcurrent/QtConcurrentRun>

namespace ram_qt_guis
{
class EntryExitStrategies : public rviz::Panel
{
Q_OBJECT

public:
  EntryExitStrategies(QWidget* parent = NULL);
  virtual ~EntryExitStrategies();

Q_SIGNALS:
  void enable(const bool);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

private:
  void updateInternalParameters();
  void connectToService(ros::ServiceClient &client);
  void connectToServices();

protected Q_SLOTS:
  void sendEntryExitParameters();
  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);

  void load(const rviz::Config& config);
  void save(rviz::Config config) const;

protected:
  QSpinBox *entry_number_of_poses_;
  QDoubleSpinBox *entry_angle_;
  QDoubleSpinBox *entry_distance_;

  QSpinBox *exit_number_of_poses_;
  QDoubleSpinBox *exit_angle_;
  QDoubleSpinBox *exit_distance_;

  QPushButton *entry_exit_button_;

  ros::NodeHandle nh_;
  ros::ServiceClient entry_parameters_client_;
  ros::ServiceClient exit_parameters_client_;

  ram_utils::EntryExitParameters entry_parameters_;
  ram_utils::EntryExitParameters exit_parameters_;
};

}

#endif
