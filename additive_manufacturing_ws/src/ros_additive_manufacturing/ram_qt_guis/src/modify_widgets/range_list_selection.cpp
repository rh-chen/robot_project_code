#include <ram_qt_guis/modify_widgets/range_list_selection.hpp>

namespace ram_qt_guis
{
ModifyRangeListSelection::ModifyRangeListSelection(QVBoxLayout* layout,
                                                   const QString help_string,
                                                   const unsigned min_val,
                                                   const unsigned max_val,
                                                   std::vector<unsigned> locked) :
        layout_(layout),
        help_string_(help_string),
        min_val_(min_val),
        max_val_(max_val)
{
  setObjectName("ModifyRangeListSelection");
  layout_->addWidget(new QLabel(help_string_));

  QTabWidget *tab_widget = new QTabWidget;
  layout_->addWidget(tab_widget);

  QHBoxLayout *all_none_buttons = new QHBoxLayout;
  all_button_ = new QPushButton("All");
  none_button_ = new QPushButton("None");
  all_none_buttons->addWidget(all_button_);
  all_none_buttons->addWidget(none_button_);
  layout_->addLayout(all_none_buttons);

  QWidget *range_select = new QWidget;
  QVBoxLayout *range_select_layout = new QVBoxLayout(range_select);
  range_select->setLayout(range_select_layout);

  range_select_layout->addWidget(new QLabel("Select first and last:"));
  min_box_ = new QSpinBox;
  min_box_->setRange(min_val_, max_val_);
  max_box_ = new QSpinBox;
  max_box_->setRange(min_val_, max_val_);
  connect(min_box_, SIGNAL(valueChanged(int)), this, SLOT(tweakRangeMin()));
  connect(max_box_, SIGNAL(valueChanged(int)), this, SLOT(tweakRangeMax()));

  QHBoxLayout *min_box_layout = new QHBoxLayout;
  min_box_layout->addWidget(new QLabel("First"));
  min_box_layout->addWidget(min_box_);
  QHBoxLayout *max_box_layout = new QHBoxLayout;
  max_box_layout->addWidget(new QLabel("Last"));
  max_box_layout->addWidget(max_box_);

  range_select_layout->addLayout(min_box_layout);
  range_select_layout->addLayout(max_box_layout);

  QHBoxLayout *add_remove_invert_buttons = new QHBoxLayout;
  add_button_ = new QPushButton("Add");
  remove_button_ = new QPushButton("Remove");
  invert_button_ = new QPushButton("Invert");
  add_remove_invert_buttons->addWidget(add_button_);
  add_remove_invert_buttons->addWidget(remove_button_);
  add_remove_invert_buttons->addWidget(invert_button_);

  range_select_layout->addLayout(add_remove_invert_buttons);
  range_select_layout->addStretch(1);

  QWidget *scroll_widget = new QWidget;
  QVBoxLayout *checkboxes_layout = new QVBoxLayout;
  scroll_widget->setLayout(checkboxes_layout);
  QScrollArea *scroll_area = new QScrollArea;
  scroll_area->setWidget(scroll_widget);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);

  tab_widget->addTab(scroll_area, "Check-boxes");
  tab_widget->addTab(range_select, "Range select");

  for (unsigned i(min_val_); i <= max_val_; ++i)
  {
    QCheckBox *checkbox = new QCheckBox;
    checkbox->setText(QString::fromStdString(std::to_string(i)));
    checkboxes.push_back(checkbox);
  }

  for (auto checkbox : checkboxes)
  {
    checkboxes_layout->addWidget(checkbox);
    connect(checkbox, SIGNAL(stateChanged(int)), this, SLOT(updateSelectionFromTicks()));
  }

  selection_.resize(checkboxes.size());

  if (!locked.empty())
  {
    std::sort(locked.begin(), locked.end());
    if (locked.back() < checkboxes.size())
    {
      for (unsigned i(0); i < locked.size(); ++i)
      {
        checkboxes[locked[i]]->setEnabled(false);
        checkboxes[locked[i]]->setChecked(true);
      }
    }
    updateSelectionFromTicks();
  }

  button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok
      | QDialogButtonBox::Cancel);
  layout_->addStretch(1);
  layout_->addWidget(button_box_);

  connect(add_button_, SIGNAL(clicked()), this, SLOT(addButton()));
  connect(remove_button_, SIGNAL(clicked()), this, SLOT(removeButton()));
  connect(invert_button_, SIGNAL(clicked()), this, SLOT(invertButton()));
  connect(all_button_, SIGNAL(clicked()), this, SLOT(selectAll()));
  connect(none_button_, SIGNAL(clicked()), this, SLOT(selectNone()));
}

ModifyRangeListSelection::~ModifyRangeListSelection()
{
}

std::vector<unsigned>
ModifyRangeListSelection::getSelection()
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);
  std::vector<unsigned> selection;

  for (unsigned i(0); i < selection_.size(); ++i)
    if (selection_[i] == true)
      selection.push_back(i);

  return selection;
}

void
ModifyRangeListSelection::updateTicksFromSelection()
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  Q_EMIT layout_->parentWidget()->setEnabled(false);

  // Block signals from check boxes to prevent nested infinite calls
  std::vector<QSignalBlocker> blockers;
  for (auto checkbox : checkboxes)
    blockers.push_back(QSignalBlocker(checkbox));

  for (unsigned i(0); i < checkboxes.size(); ++i)
    checkboxes[i]->setChecked(selection_[i]);

  Q_EMIT layout_->parentWidget()->setEnabled(true);
  Q_EMIT selectionChanged(getSelection());
}

void ModifyRangeListSelection::tweakRangeMin()
{
  if (min_box_->value() > max_box_->value())
    max_box_->setValue(min_box_->value());
}

void ModifyRangeListSelection::tweakRangeMax()
{
  if (min_box_->value() > max_box_->value())
    min_box_->setValue(max_box_->value());
}

void
ModifyRangeListSelection::updateSelectionFromTicks()
{
  std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

  Q_EMIT layout_->parentWidget()->setEnabled(false);

  for (unsigned i(0); i < selection_.size(); ++i)
    selection_[i] = checkboxes[i]->isChecked();

  Q_EMIT layout_->parentWidget()->setEnabled(true);
  Q_EMIT selectionChanged(getSelection());
}

void
ModifyRangeListSelection::addButton()
{
  {
    std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

    if (!checkRange())
      return;

    for (unsigned i(min_box_->value()); i <= (unsigned)max_box_->value(); ++i)
      selection_[i] = true;
  }

  updateTicksFromSelection();
}

void
ModifyRangeListSelection::removeButton()
{
  {
    std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

    if (!checkRange())
      return;

    for (unsigned i(min_box_->value()); i <= (unsigned)max_box_->value(); ++i)
      if (checkboxes[i]->isEnabled())
        selection_[i] = false;
  }

  updateTicksFromSelection();
}

void
ModifyRangeListSelection::invertButton()
{
  {
    std::lock_guard<std::recursive_mutex> lock(selection_mutex_);

    if (!checkRange())
      return;

    for (unsigned i(min_box_->value()); i <= (unsigned)max_box_->value(); ++i)
      if (checkboxes[i]->isEnabled())
        selection_[i] = !selection_[i];
  }

  updateTicksFromSelection();
}

void
ModifyRangeListSelection::selectAll()
{
  {
    std::lock_guard<std::recursive_mutex> lock(selection_mutex_);
    std::fill(selection_.begin(), selection_.end(), true);
  }
  updateTicksFromSelection();
}

void
ModifyRangeListSelection::selectNone()
{
  {
    std::lock_guard<std::recursive_mutex> lock(selection_mutex_);
    std::fill(selection_.begin(), selection_.end(), false);
  }
  updateTicksFromSelection();
}

bool
ModifyRangeListSelection::checkRange()
{
  if (min_box_->value() > max_box_->value())
  {
    QMessageBox::warning(this, ("Wrong range"),
                         ("Wrong selection, the \"last\" value must be superior or equal to the \"first\" value."),
                         QMessageBox::Ok);
    return false;
  }

  return true;
}

}
