/**
 * @file /include/ros_logger_gui/main_window.hpp
 *
 * @brief Qt based gui for ros_logger_gui.
 *
 * @date November 2010
 **/
#ifndef ros_logger_gui_MAIN_WINDOW_H
#define ros_logger_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QAbstractItemView>
#include <QDir>
#include <QFileDialog>
#include <QListWidget>
#include <QMainWindow>
#include <QMessageBox>
#include <QSettings>

#include "qnode.hpp"
#include "ui_main_window.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ros_logger_gui {
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */

class MainWindow : public QMainWindow {
  Q_OBJECT
  Ui::MainWindowDesign ui_;
  QNode qn_;
  QString file_name_;
  QString file_path_;
  QStringList subscription_;
  QStringList all_topics_;

  bool selected_all_;

public:
  MainWindow(int argc, char **argv, QWidget *parent = 0);
  ~MainWindow();

  void ReadSettings();  // Load up qt program settings at startup
  void WriteSettings(); // Save qt program settings when closing

  void closeEvent(QCloseEvent *event); // Overloaded function
  void showTopicMsg();

  enum state { Unstarted, Stopped, Running } RecordState;

  void updateUI();

public Q_SLOTS:

  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_list_topics_itemSelectionChanged();
  void on_list_subscription_itemSelectionChanged();
  void on_checkBox_remember_stateChanged(int);
  void on_checkBox_add_date_stateChanged(int);
  void on_line_edit_filter_textChanged(QString);
  void on_button_toggle_recording_clicked(bool check);
  void on_button_select_all_clicked(bool check);
  void on_button_save_clicked(bool check);
  void on_button_subscribe_clicked(bool check);
  void on_button_unsubscribe_clicked(bool check);
  void on_button_reconnect_clicked(bool check);
  /******************************************
   ** Manual connections
   *******************************************/

private:
  void updateTopics();
};

} // namespace ros_logger_gui

#endif // ros_logger_gui_MAIN_WINDOW_H
