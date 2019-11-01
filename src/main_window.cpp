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
#include "../include/ros_logger_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_logger_gui
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

    updateRecordingState();

    /*********************
    ** Logging
    **********************/
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(rosShutDown()), this, SLOT(auto_shutdown()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateRecordingState()));

    /*********************
    ** Auto Start
    **********************/

    QDir savedir_default(savedir_default_path);

    if (!savedir_default.exists())
    {
        savedir_default.mkpath(savedir_default_path);
    }

    curr_topics = qnode.get_curr_topics();

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

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::updateRecordingState()
{
    switch (qnode.state)
    {
    case UNSTARTED:
        ui.logging_status_flag->setText("<font color='red'>ROS is not started</font>");
        ui.button_start_logging->setEnabled(false);
        ui.button_stop_logging->setEnabled(false);
        ui.button_refresh_current->setEnabled(false);
        ui.button_refresh_all->setEnabled(false);
        ui.button_update_topic->setEnabled(false);
        ui.button_reset_topic->setEnabled(false);
        break;
    case STOPPED:
        ui.logging_status_flag->setText("<font color='red'>Data logging stopped</font>");
        ui.button_start_logging->setEnabled(true);
        ui.button_stop_logging->setEnabled(false);
        ui.button_refresh_current->setEnabled(true);
        ui.button_refresh_all->setEnabled(true);
        ui.button_update_topic->setEnabled(true);
        ui.button_reset_topic->setEnabled(true);
        break;
    case RUNNING:
        ui.logging_status_flag->setText("<font color='green'>Data logging running</font>");
        ui.button_start_logging->setEnabled(false);
        ui.button_stop_logging->setEnabled(true);
        ui.button_refresh_current->setEnabled(true);
        ui.button_refresh_all->setEnabled(true);
        ui.button_update_topic->setEnabled(true);
        ui.button_reset_topic->setEnabled(true);
        break;
    }
}

/*****************************************************************************
** Implementation [Buttons]
*****************************************************************************/

void MainWindow::auto_shutdown(bool check)
{
    qnode.stop_logging();
    updateRecordingState();
}

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

void MainWindow::on_button_refresh_state_clicked(bool check)
{
    if (qnode.state == UNSTARTED)
    {
        qnode.init();
    }
    else if (qnode.state != UNSTARTED)
    {

    }
    updateRecordingState();
}

void MainWindow::on_button_start_logging_clicked(bool check)
{
    filename = ui.line_edit_directory->text();
    qnode.set_savefile(filename);
    qnode.start_logging();
}

void MainWindow::on_button_stop_logging_clicked(bool check)
{
    qnode.stop_logging();
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

void MainWindow::on_button_refresh_current_clicked(bool check)
{
    ui.list_topics->clear();
    curr_topics = qnode.get_curr_topics();
    ui.list_topics->addItems(curr_topics);
}

void MainWindow::on_button_refresh_all_clicked(bool check)
{
    ui.list_topics->clear();
    all_topics = qnode.get_all_topics();
    ui.list_topics->addItems(all_topics);
    for (const auto topic : curr_topics)
    {
        QList<QListWidgetItem *> lst = ui.list_topics->findItems(topic, Qt::MatchContains);
        for (const auto item : lst)
        {
            item->setBackground(Qt::green);
        }
    }
}

void MainWindow::on_button_update_topic_clicked(bool check)
{
    QList<QListWidgetItem *> q_topics = ui.list_topics->selectedItems();
    ui.placeholder->setText(QString::number(q_topics.size()) + QString(" topics selected."));
    std::vector<std::string> s_topic_names;

    for (const auto topic : q_topics)
    {
        ui.list_topics->takeItem(ui.list_topics->row(topic));
        QString qs = topic->text();

        if (std::find(curr_topics.begin(), curr_topics.end(), qs) == curr_topics.end())
        {
            s_topic_names.push_back(qs.toStdString());
            curr_topics += qs;
        }
    }
    qnode.set_topics(s_topic_names);
}

void MainWindow::on_button_reset_topic_clicked(bool check)
{
    std::vector<std::string> resetter = {};
    qnode.set_topics(resetter);
    ui.list_topics->clear();
    curr_topics.clear();
    curr_topics += QString("clock");
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

} // namespace ros_logger_gui
