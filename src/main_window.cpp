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

#include <QtGui>
#include <QFileDialog>
#include <QDir>
#include <QMessageBox>
#include <iostream>
#include "../include/qt_recorder/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_recorder
{

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc, argv)
{
    ui.setupUi(this);                                                                    // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    bool init_ros_ok = qnode.init();

    /*********************
    ** Logging
    **********************/
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateRecordingState()));

    /*********************
    ** Auto Start
    **********************/

    QDir savedir_default(savedir_default_path);

    if (!savedir_default.exists())
    {
        savedir_default.mkpath(savedir_default_path);
    }

    filename = savedir_default_path + filename;
    qnode.set_savefile(filename);

    ui.line_edit_directory->setText(filename);
    ui.button_save_new_dir->setEnabled(false);
    ui.save_location_flag->setText("Saving ROS bags to " + savedir_default_path);
    ui.new_location_flag->setText("<font color='green'>Using default save location</font>");
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage()
{
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About ..."), tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

void MainWindow::updateRecordingState()
{
    switch (qnode.state)
    {
    case STOPPED:
        ui.recording_status_flag->setText("<font color='red'>Data recording stopped</font>");
        ui.button_start_recording->setEnabled(true);
        ui.button_stop_recording->setEnabled(false);
        break;
    case RUNNING:
        ui.recording_status_flag->setText("<font color='green'>Data recording running</font>");
        ui.button_start_recording->setEnabled(false);
        ui.button_stop_recording->setEnabled(true);
        break;
    }
}

/*****************************************************************************
** Implementation [Buttons]
*****************************************************************************/

void MainWindow::on_button_browse_dir_clicked(bool check)
{
    savedir_path = QFileDialog::getExistingDirectory(this,
                                                     "Save to",
                                                     QDir::currentPath(),
                                                     QFileDialog::ShowDirsOnly);
    filename = savedir_path + "/test.bag";
    ui.new_location_flag->setText("<font color='red'>Save location changed!</font>");
    ui.line_edit_directory->setText(filename);
    ui.button_save_new_dir->setEnabled(true);
}

void MainWindow::on_button_start_recording_clicked(bool check)
{
    filename = ui.line_edit_directory->text();
    qnode.set_savefile(filename);
    qnode.start_recording();
}

void MainWindow::on_button_stop_recording_clicked(bool check)
{
    qnode.stop_recording();
}

void MainWindow::on_button_save_new_dir_clicked(bool check)
{
    qnode.set_savefile(filename);

    if (QString::compare(savedir_path, savedir_default_path))
    {
        ui.new_location_flag->setText("<font color='green'>Save location confirmed</font>");
    }
    else
    {
        ui.new_location_flag->setText("<font color='green'>Using default save location</font>");
    }

    ui.save_location_flag->setText("Saving ROS bags to " + savedir_path);

    ui.button_save_new_dir->setEnabled(false);
}

void MainWindow::on_button_refresh_topic_clicked(bool check)
{
    topic_list = qnode.query_topics();
    ui.list_topics->addItems( topic_list );
}

void MainWindow::on_button_update_topic_clicked(bool check)
{
    QList<QListWidgetItem*> topic_list = ui.list_topics->selectedItems();
    ui.placeholder->setText(QString::number(topic_list.size())+QString(" topics selected."));
    std::vector<std::string> s_topic_names;
    for (const auto topic : topic_list) {
        QString qs = topic->text();
        s_topic_names.push_back(qs.toStdString() );
    }
    qnode.set_topics(s_topic_names);
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings()
{
    QSettings settings("Qt-Ros Package", "qt_ground_station");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings()
{
    QSettings settings("Qt-Ros Package", "qt_ground_station");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

} // namespace qt_recorder
