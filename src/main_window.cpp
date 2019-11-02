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
    : QMainWindow(parent), qnode(argc, argv),
    filename("/test.bag"),
    default_save_path(QDir::homePath() + "/ROS_bags")
{
    ui.setupUi(this);                                                                    // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    bool init_ros_ok = qnode.init();

    update_recstate();
    refresh_topic();
    /*********************
    ** Logging
    **********************/
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(rosShutDown()), this, SLOT(auto_shutdown()));
    QObject::connect(&qnode, SIGNAL(logStateChanged()), this, SLOT(update_recstate()));
    QObject::connect(ui.line_edit_filter, SIGNAL(textChanged(QString)), this, SLOT(filter_topics()));

    /*********************
    ** Auto Start
    **********************/

    QDir default_save_dir(default_save_path);

    if (!default_save_dir.exists())
    {
        default_save_dir.mkpath(default_save_path);
    }

    subscription = qnode.get_subscription();

    filename = default_save_path + filename;
    qnode.set_savefile(filename);

    ui.line_edit_directory->setText(filename);
    ui.button_save_new_dir->setEnabled(false);
    ui.save_location_flag->setText("Saving ROS bags to " + default_save_path);
    ui.new_location_flag->setText("<font color='green'>Using default save location</font>");
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::showTopicMsg() {
    QMessageBox msgBox;
}

void MainWindow::update_recstate()
{
    switch (qnode.state)
    {
    case QNode::Unstarted:
        ui.logging_status_flag->setText("<font color='red'>ROS is not started</font>");
        ui.button_start_logging->setEnabled(false);
        ui.button_stop_logging->setEnabled(false);
        ui.button_update_topic->setEnabled(false);
        ui.button_reset_topic->setEnabled(false);
        break;
    case QNode::Stopped:
        ui.logging_status_flag->setText("<font color='red'>Data logging stopped</font>");
        ui.button_start_logging->setEnabled(true);
        ui.button_stop_logging->setEnabled(false);
        ui.button_update_topic->setEnabled(true);
        ui.button_reset_topic->setEnabled(true);
        break;
    case QNode::Running:
        ui.logging_status_flag->setText("<font color='green'>Data logging running</font>");
        ui.button_start_logging->setEnabled(false);
        ui.button_stop_logging->setEnabled(true);
        ui.button_update_topic->setEnabled(true);
        ui.button_reset_topic->setEnabled(true);
        break;
    }
}

/*****************************************************************************
** Implementation [Buttons]
*****************************************************************************/

void MainWindow::auto_shutdown()
{
    qnode.stop_logging();
    update_recstate();
}

void MainWindow::on_button_browse_dir_clicked(bool check)
{
    save_path = QFileDialog::getExistingDirectory(this,
                                                     "Save to",
                                                     QDir::currentPath(),
                                                     QFileDialog::ShowDirsOnly);
    filename = save_path + "/test.bag";
    ui.new_location_flag->setText("<font color='red'>Save location changed!</font>");
    ui.line_edit_directory->setText(filename);
    ui.button_save_new_dir->setEnabled(true);
}

void MainWindow::on_button_refresh_state_clicked(bool check)
{
    bool qnode_state = false;
    if (qnode.state == QNode::Unstarted)
    {
        bool qnode_state = qnode.init();
    }
    update_recstate();
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

    if (QString::compare(save_path, default_save_path))
    {
        ui.new_location_flag->setText("<font color='green'>Save location confirmed</font>");
    }
    else
    {
        ui.new_location_flag->setText("<font color='green'>Using default save location</font>");
    }

    ui.save_location_flag->setText("Saving ROS bags to " + save_path);

    ui.button_save_new_dir->setEnabled(false);
}

void MainWindow::filter_topics()
{
    QString filter = ui.line_edit_filter->text();
    QStringList filtered_sub = subscription.filter(filter);
    ui.list_subscription->clear();
    ui.list_subscription->addItems(filtered_sub);

    QStringList filtered_topics = topics.filter(filter);
    ui.list_topics->clear();
    ui.list_topics->addItems(filtered_topics);
}

void MainWindow::on_button_refresh_topic_clicked(bool check)
{
    refresh_topic();
}

void MainWindow::refresh_topic()
{
    ui.list_subscription->clear();
    subscription = qnode.get_subscription();
    ui.list_subscription->addItems(subscription);

    ui.list_topics->clear();
    topics = qnode.get_topics();
    ui.list_topics->addItems(topics);

    for (const auto &sub : subscription)
    {
        QList<QListWidgetItem *> disp_topics = ui.list_topics->findItems(sub, Qt::MatchContains);
        for (const auto topic : disp_topics)
        {
            topic->setBackground(Qt::green);
        }
    }
}

void MainWindow::on_button_update_topic_clicked(bool check)
{
    QList<QListWidgetItem *> q_topics = ui.list_topics->selectedItems();
    ui.placeholder->setText(QString::number(q_topics.size()) + QString(" topics selected."));
    std::vector<std::string> s_topic_names;

    for (const auto &topic : q_topics)
    {
        ui.list_topics->takeItem(ui.list_topics->row(topic));
        QString qs = topic->text();

        if (std::find(subscription.begin(), subscription.end(), qs) == subscription.end())
        {
            s_topic_names.push_back(qs.toStdString());
            subscription += qs;
        }
    }
    qnode.set_topics(s_topic_names);
    refresh_topic();
}

void MainWindow::on_button_reset_topic_clicked(bool check)
{
    std::vector<std::string> resetter = {};
    qnode.set_topics(resetter);
    ui.list_topics->clear();
    subscription.clear();
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
