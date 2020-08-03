/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/ros_logger_gui/main_window.hpp"
#include <iostream>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_logger_gui {
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qn_(argc, argv), file_name_("/rosbag.bag"),
      file_path_(QDir::homePath() + "/ROS_bags") {
  ui_.setupUi(this); // Calling this incidentally connects all ui's triggers to
                    // on_...() callbacks in this class.

  QObject::connect(&qn_, SIGNAL(rosShutdown()), this, SLOT(close()));
  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui_.tab_manager->setCurrentIndex(
      0); // ensure the first tab is showing - qt-designer should have this
          // already hardwired, but often loses it (settings?).

  bool init_ros_ok = qn_.init();
  if (!init_ros_ok) {
    RecordState = Unstarted;
    init_ros_ok = qn_.init();
  } else {
    ui_.label_master->setText("Connected to master at:" +
                             QString::fromStdString(qn_.showMaster()));
    RecordState = Stopped;
  }

  /*********************
  ** Auto Start
  **********************/

  QDir save_dir(file_path_);

  if (!save_dir.exists()) {
    save_dir.mkpath(file_path_);
  }

  subscription_ = qn_.lsSubscription();

  ui_.button_save->setEnabled(false);
  ui_.save_flag->setText("No Recorded Bag Data");
  ui_.button_select_all->setText("Select All");

  updateUI();
  updateTopics();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::showTopicMsg() { QMessageBox msgBox; }

void MainWindow::updateUI() {
  switch (RecordState) {
  case Unstarted:
    ui_.logging_status_flag->setText(
        "<font color='red'>ROS is not started. Retrying</font>");
    ui_.button_toggle_recording->setEnabled(false);
    ui_.button_subscribe->setEnabled(false);
    ui_.button_unsubscribe->setEnabled(false);
    ui_.button_reconnect->setText("Reconnect");
    break;
  case Stopped:
    ui_.logging_status_flag->setText(
        "<font color='red'>Data logging stopped</font>");
    ui_.button_toggle_recording->setText("Start recording");
    ui_.button_subscribe->setEnabled(true);
    ui_.button_unsubscribe->setEnabled(true);
    ui_.button_reconnect->setText("Refresh Screen");
    break;
  case Running:
    ui_.logging_status_flag->setText(
        "<font color='green'>Data logging running</font>");
    ui_.button_toggle_recording->setText("Stop recording");
    ui_.button_subscribe->setEnabled(true);
    ui_.button_unsubscribe->setEnabled(true);
    ui_.button_reconnect->setText("Refresh Screen");
    break;
  }
}

/*****************************************************************************
** Implementation [Buttons]
*****************************************************************************/

void MainWindow::on_button_reconnect_clicked(bool check) {
  bool init_ros_ok = false;
  if (RecordState == Unstarted) {
    init_ros_ok = qn_.init();
  }
  if (init_ros_ok || RecordState != Unstarted) {
    ui_.label_master->setText("Connected to master at: " +
                             QString::fromStdString(qn_.showMaster()));
    updateUI();
    updateTopics();
  }
}

void MainWindow::on_button_toggle_recording_clicked(bool check) {
  switch (RecordState) {
  case Stopped:
    if (qn_.startRecording())
      RecordState = Running;
    break;
  case Running:
    if (qn_.stopRecording())
      RecordState = Stopped;

    if (qn_.showBagSize().isNull()) {
      ui_.save_flag->setText("No data recorded");
    } else {
      ui_.save_flag->setText("Unsaved recorder data. Size is " +
                            qn_.showBagSize());
      ui_.button_save->setEnabled(true);
    }
  }
  updateUI();
  updateTopics();
}

void MainWindow::on_button_save_clicked(bool check) {
  QString write_name = file_path_ + qn_.formatFilenames(file_name_);
  write_name = QFileDialog::getSaveFileName(this, tr("Save Rosbag"), write_name,
                                            tr("ROS bag (*.bag)"));
  qn_.updateFilenames(write_name);

  if (qn_.doRecord()) {
    ui_.save_flag->setText("ROS bag saved");
    ui_.button_save->setEnabled(false);
  }
  write_name = file_path_ + qn_.formatFilenames(file_name_);
}

void MainWindow::on_checkBox_add_date_stateChanged(int) {
  std::map<std::string, bool> date;
  if (ui_.checkBox_add_date->isChecked()) {
    date["append_date"] = true;
  } else {
    date["append_date"] = false;
  }
  qn_.setOptions(date);
}

void MainWindow::on_checkBox_remember_stateChanged(int) {
  std::map<std::string, bool> remember;
  if (ui_.checkBox_remember->isChecked()) {
    remember["remember_topics"] = true;
  } else {
    remember["remember_topics"] = false;
  }
  qn_.setOptions(remember);
}

void MainWindow::on_list_subscription_itemSelectionChanged() {
  int selected = ui_.list_subscription->selectedItems().count();
  int all = ui_.list_subscription->count();
  if (selected == all) {
    ui_.button_select_all->setText("Unselect All");
    selected_all_ = true;
  } else {
    ui_.button_select_all->setText("Select All");
    selected_all_ = false;
  }
}

void MainWindow::on_list_topics_itemSelectionChanged() {
  int selected = ui_.list_topics->selectedItems().count();
  int all = ui_.list_topics->count();
  if (selected == all) {
    ui_.button_select_all->setText("Unselect All");
    selected_all_ = true;
  } else {
    ui_.button_select_all->setText("Select All");
    selected_all_ = false;
  }
}

void MainWindow::on_button_select_all_clicked(bool check) {
  switch (ui_.tab_navigator->currentIndex()) {
  case 0:
    if (!selected_all_) {
      ui_.list_topics->selectAll();
    } else {
      ui_.list_topics->clear();
      updateTopics();
    }
    break;
  case 1:
    if (!selected_all_) {
      ui_.list_subscription->selectAll();
    } else {
      ui_.list_subscription->clear();
      updateTopics();
    }
    break;
  }
  ui_.topic_counter->setText("All topics selected");
}

void MainWindow::on_line_edit_filter_textChanged(QString) {
  QString filter = ui_.line_edit_filter->text();
  QStringList filtered_sub = subscription_.filter(filter);
  ui_.list_subscription->clear();
  ui_.list_subscription->addItems(filtered_sub);

  QStringList filtered_topics = all_topics_.filter(filter);
  ui_.list_topics->clear();
  ui_.list_topics->addItems(filtered_topics);
}

void MainWindow::updateTopics() {
  ui_.list_subscription->clear();
  subscription_ = qn_.lsSubscription();
  ui_.list_subscription->addItems(subscription_);

  ui_.list_topics->clear();
  all_topics_ = qn_.lsAllTopics();
  ui_.list_topics->addItems(all_topics_);

  for (const auto &sub : subscription_) {
    QList<QListWidgetItem *> disp_topics =
        ui_.list_topics->findItems(sub, Qt::MatchContains);
    for (const auto &it : disp_topics) {
      it->setBackground(Qt::green);
      it->setFlags(it->flags() & ~Qt::ItemIsSelectable);
    }
  }
}

void MainWindow::on_button_subscribe_clicked(bool check) {
  QList<QListWidgetItem *> add_topics = ui_.list_topics->selectedItems();
  ui_.topic_counter->setText(QString::number(add_topics.size()) +
                            QString(" topics selected."));
  std::vector<std::string> topics;

  for (const auto &it : add_topics) {
    ui_.list_topics->removeItemWidget(it);
    topics.push_back(it->text().toStdString());
    subscription_ += it->text();
  }
  qn_.addSubscription(topics);
  updateTopics();
}

void MainWindow::on_button_unsubscribe_clicked(bool check) {
  QList<QListWidgetItem *> rm_topics = ui_.list_subscription->selectedItems();

  ui_.topic_counter->setText(QString::number(rm_topics.size()) +
                            QString(" topics selected."));

  std::vector<std::string> topics;
  for (const auto &it : rm_topics) {
    ui_.list_subscription->removeItemWidget(it);
    topics.push_back(it->text().toStdString());
    subscription_.removeAt(subscription_.indexOf(it->text()));
  }
  qn_.rmSubscription(topics);
  updateTopics();
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "ros_logger_gui");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "ros_logger_gui");
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event) {
  WriteSettings();
  QMainWindow::closeEvent(event);
}

} // namespace ros_logger_gui
