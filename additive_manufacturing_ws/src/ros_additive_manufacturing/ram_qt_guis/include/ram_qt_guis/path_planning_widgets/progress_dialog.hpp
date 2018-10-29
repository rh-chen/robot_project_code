#ifndef RAM_QT_GUIS_ALGORITHMS_WIDGETS_PROGRESS_DIALOG_HPP
#define RAM_QT_GUIS_ALGORITHMS_WIDGETS_PROGRESS_DIALOG_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <QDialog>
#include <QProgressBar>
#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>

namespace ram_qt_guis
{

class ProgressDialog : public QDialog
{
Q_OBJECT

public:
  ProgressDialog();

Q_SIGNALS:
  void drawProgress(const int value,
                    const QString msg);

  void drawProgress(const int value);

  void drawProgress(const QString msg);

private Q_SLOTS:
  void progressValueChanged();

  void progressChanged(const int value,
                       const QString msg);

  void progressChanged(const int value);

  void progressChanged(const QString msg);

private:
  QLabel *progress_message_;
  QProgressBar *progress_value_;
  QPushButton *cancel_button_;
};

}

#endif
