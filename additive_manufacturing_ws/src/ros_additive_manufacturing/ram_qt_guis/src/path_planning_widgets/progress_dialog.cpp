#include <ram_qt_guis/path_planning_widgets/progress_dialog.hpp>

namespace ram_qt_guis
{

ProgressDialog::ProgressDialog()
{
  setWindowTitle("Path planning progress");
  setModal(false);
  resize(350, 100);

  progress_value_ = new QProgressBar;
  progress_value_->setRange(0, 100);
  progress_value_->setValue(0);

  progress_message_ = new QLabel;

  cancel_button_ = new QPushButton("Cancel");

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->addWidget(progress_value_);
  main_layout->addWidget(progress_message_);
  main_layout->addWidget(cancel_button_);

  connect(cancel_button_, SIGNAL(clicked()), this, SLOT(reject()));
  // connect(progress_value_, SIGNAL(valueChanged(int)), this, SLOT(progressValueChanged()));

  connect(this, SIGNAL(drawProgress(int, QString)), this, SLOT(progressChanged(int, QString)));
  connect(this, SIGNAL(drawProgress(int)), this, SLOT(progressChanged(int)));
  connect(this, SIGNAL(drawProgress(QString)), this, SLOT(progressChanged(QString)));

}

void
ProgressDialog::progressValueChanged()
{
  if (progress_value_->value() == 100)
    Q_EMIT accept();
}

void
ProgressDialog::progressChanged(const int value,
                                const QString msg)
{
  if (value)
    progress_value_->setValue(value);
  if (!msg.isEmpty())
    progress_message_->setText(msg);
}

void
ProgressDialog::progressChanged(const int value)
{
  progress_value_->setValue(value);
}

void
ProgressDialog::progressChanged(const QString msg)
{
  progress_message_->setText(msg);
}

}
