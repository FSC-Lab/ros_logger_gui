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

#include <QtGui/QMainWindow>
#include <QFileDialog>
#include <QAbstractItemView>
#include "ui_main_window.h"
#include "qnode.hpp"

#define RUNNING 2
#define STOPPED 1
#define UNSTARTED 0

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ros_logger_gui
{

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */

class MainWindow : public QMainWindow
{
        Q_OBJECT

public:
        MainWindow(int argc, char **argv, QWidget *parent = 0);
        ~MainWindow();

        void ReadSettings();  // Load up qt program settings at startup
        void WriteSettings(); // Save qt program settings when closing

        void closeEvent(QCloseEvent *event); // Overloaded function
        void showNoMasterMessage();

        QString savedir_path;
        QString filename = "/test.bag";
        QString savedir_default_path = QDir::homePath() + "/ROS_bags";
        QStringList curr_topics;
        QStringList all_topics;

public Q_SLOTS:
        void auto_shutdown(bool check);
        /******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/

                void on_button_browse_dir_clicked(bool check);
                void on_button_start_logging_clicked(bool check);
                void on_button_stop_logging_clicked(bool check);
                void on_button_save_new_dir_clicked(bool check);
                void on_button_refresh_current_clicked(bool check);
                void on_button_refresh_all_clicked(bool check);
                void on_button_update_topic_clicked(bool check);
                void on_button_reset_topic_clicked(bool check);
                void on_button_refresh_state_clicked(bool check);
        /******************************************
    ** Manual connections
    *******************************************/
        void updateRecordingState();

Q_SIGNALS:

private:
        Ui::MainWindowDesign ui;
        QNode qnode;
};

} // namespace ros_logger_gui

#endif // ros_logger_gui_MAIN_WINDOW_H