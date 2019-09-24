/**
 * @file /include/qt_recorder/main_window.hpp
 *
 * @brief Qt based gui for qt_recorder.
 *
 * @date November 2010
 **/
#ifndef qt_recorder_MAIN_WINDOW_H
#define qt_recorder_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <QFileDialog>
#include "ui_main_window.h"
#include "qnode.hpp"

#define RUNNING 1
#define STOPPED 0

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qt_recorder {

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


public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();

            void on_button_browse_dir_clicked(bool check );
            void on_button_start_recording_clicked(bool check );
            void on_button_stop_recording_clicked(bool check );
            void on_button_save_new_dir_clicked(bool check );

    /******************************************
    ** Manual connections
    *******************************************/
    void updateRecordingState();
    void updateRecording();

Q_SIGNALS:


private:

        Ui::MainWindowDesign ui;
        QNode qnode;

};

}  // namespace qt_recorder

#endif // qt_recorder_MAIN_WINDOW_H
