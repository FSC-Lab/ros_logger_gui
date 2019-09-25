/**
 * @file /include/qt_logger/main_window.hpp
 *
 * @brief Qt based gui for qt_logger.
 *
 * @date November 2010
 **/
#ifndef qt_logger_MAIN_WINDOW_H
#define qt_logger_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <QFileDialog>
#include <QAbstractItemView>
#include "ui_main_window.h"
#include "qnode.hpp"

#define RUNNING 1
#define STOPPED 0

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qt_logger {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */


class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
        void showNoMasterMessage();

        QString savedir_path;
        QString filename = "/test.bag";
        QString savedir_default_path = QDir::homePath() + "/ROS_bags";
        QStringList curr_topics;
        QStringList all_topics;

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();

            void on_button_browse_dir_clicked(bool check );
            void on_button_start_logging_clicked(bool check );
            void on_button_stop_logging_clicked(bool check );
            void on_button_save_new_dir_clicked(bool check );
            void on_button_refresh_current_clicked(bool check );
            void on_button_refresh_all_clicked(bool check );
            void on_button_update_topic_clicked(bool check );
            void on_button_reset_topic_clicked(bool check);
    /******************************************
    ** Manual connections
    *******************************************/
    void updateRecordingState();

Q_SIGNALS:


private:
        Ui::MainWindowDesign ui;
        QNode qnode;

};

}  // namespace qt_logger

#endif // qt_logger_MAIN_WINDOW_H
