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
        void showTopicMsg();

        QString save_path;
        QString filename;
        QString default_save_path;
        QStringList subscription;
        QStringList all_topics;

        enum state
        {
                Unstarted,
                Stopped,
                Running
        } RecordState;

public Q_SLOTS:
        void filter_topics();
        void check_topics();
        void update_recstate();

        /******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
        //void on_button_refresh_topic_clicked(bool check);
        void on_button_browse_dir_clicked(bool check);
        void on_button_toggle_logging_clicked(bool check);
        void on_button_save_new_dir_clicked(bool check);
        void on_button_subscribe_clicked(bool check);
        void on_button_unsubscribe_clicked(bool check);
        void on_button_refresh_state_clicked(bool check);
        /******************************************
    ** Manual connections
    *******************************************/

Q_SIGNALS:
        void startSignal();
        void stopSignal();

private:
        void refresh_topic();
        Ui::MainWindowDesign ui;
        QNode qnode;
};

} // namespace ros_logger_gui

#endif // ros_logger_gui_MAIN_WINDOW_H
