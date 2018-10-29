#ifndef RAM_QT_GUIS_DISPLAY_HPP
#define RAM_QT_GUIS_DISPLAY_HPP

#ifndef Q_MOC_RUN
#include <eigen_conversions/eigen_msg.h>
#include <ram_display/DeleteTrajectory.h>
#include <ram_display/DisplayTrajectory.h>
#include <ram_display/UpdateMeshColor.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#endif

#include <QCheckBox>
#include <QColorDialog>
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
class Display : public rviz::Panel
{
Q_OBJECT
  public:
  Display(QWidget* parent = NULL);
  virtual ~Display();

Q_SIGNALS:
  void enable(const bool);
  void displayMessageBox(const QString,
                         const QString,
                         const QString,
                         const QMessageBox::Icon);

private:
  void connectToService(ros::ServiceClient &client);
  void connectToServices();
  void updateInternalParameters();

protected Q_SLOTS:
  void displayModeChanged();
  void pickColor();
  void sendDisplayInformationButtonHandler();
  void sendDisplayInformation();
  void sendDeleteInformationButtonHandler();
  void sendDeleteInformation();
  void enableRangeLimitsBox();
  void enableLabelsBox();
  void tweakFirstLayer();
  void tweakLastLayer();

  void load(const rviz::Config& config);
  void sendLoadedInformation();
  void save(rviz::Config config) const;

  void displayMessageBoxHandler(const QString title,
                                const QString text,
                                const QString info = "",
                                const QMessageBox::Icon icon = QMessageBox::Icon::Information);
protected:
  QComboBox *color_mode_;
  QComboBox *display_mode_;
  QWidget *cylinder_size_widget_;
  QDoubleSpinBox *cylinder_size_;
  QWidget *wire_size_widget_;
  QDoubleSpinBox *wire_size_;
  QWidget *axis_size_widget_;
  QDoubleSpinBox *axis_size_;
  QWidget *labels_container_;
  QComboBox *label_type_;
  QDoubleSpinBox *label_text_size_;
  QCheckBox *display_labels_;
  QCheckBox *range_of_layers_;
  QWidget *range_of_layers_container_;
  QSpinBox *first_layer_;
  QSpinBox *last_layer_;
  QPushButton *display_button_;
  QPushButton *delete_button_;
  QPushButton *mesh_color_;

  ros::NodeHandle nh_;
  ros::ServiceClient display_client_;
  ros::ServiceClient delete_client_;
  ros::ServiceClient update_mesh_color_client_;
  ram_display::DisplayTrajectory params_;
};

}

#endif
