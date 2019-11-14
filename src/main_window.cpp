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
#include <QListWidget>
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
    while (!init_ros_ok)
    {
        RecordState = Unstarted;
        init_ros_ok = qnode.init();
        sleep(5);
    }
    RecordState = Stopped;

    /*********************
    ** Logging
    **********************/
    QObject::connect(&qnode, SIGNAL(logStateChanged()), this, SLOT(update_recstate()));
    QObject::connect(&qnode, SIGNAL(rosRaise()), this, SLOT(update_recstate()));
    QObject::connect(this, SIGNAL(startSignal()), &qnode, SLOT(startRecord()));
    QObject::connect(this, SIGNAL(stopSignal()), &qnode, SLOT(stopRecord()));
    QObject::connect(this, SIGNAL(stopSignal()), &qnode, SLOT(doRecord()));
    QObject::connect(ui.checkbox_topics, SIGNAL(stateChanged(int)), this, SLOT(check_topics()));
    QObject::connect(ui.line_edit_filter, SIGNAL(textChanged(QString)), this, SLOT(filter_topics()));
    /*********************
    ** Auto Start
    **********************/

    QDir default_save_dir(default_save_path);

    if (!default_save_dir.exists())
    {
        default_save_dir.mkpath(default_save_path);
    }

    subscription = qnode.lsSubscription();

    filename = default_save_path + filename;

    ui.line_edit_directory->setText(filename);
    ui.button_save_new_dir->setEnabled(false);
    ui.save_location_flag->setText("Saving ROS bags to " + default_save_path);
    ui.new_location_flag->setText("<font color='green'>Using default save location</font>");

    update_recstate();
    refresh_topic();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::showTopicMsg()
{
    QMessageBox msgBox;
}

void MainWindow::update_recstate()
{
    switch (RecordState)
    {
    case Unstarted:
        ui.logging_status_flag->setText("<font color='red'>ROS is not started</font>");
        ui.button_toggle_logging->setEnabled(false);
        ui.button_subscribe->setEnabled(false);
        ui.button_unsubscribe->setEnabled(false);
        break;
    case Stopped:
        ui.logging_status_flag->setText("<font color='red'>Data logging stopped</font>");
        ui.button_toggle_logging->setText("Start recording");
        ui.button_subscribe->setEnabled(true);
        ui.button_unsubscribe->setEnabled(true);
        break;
    case Running:
        ui.logging_status_flag->setText("<font color='green'>Data logging running</font>");
        ui.button_toggle_logging->setText("Stop recording");
        ui.button_subscribe->setEnabled(true);
        ui.button_unsubscribe->setEnabled(true);
        break;
    }
}

/*****************************************************************************
** Implementation [Buttons]
*****************************************************************************/

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
    if (RecordState == Unstarted)
    {
        bool qnode_state = qnode.init();
    }
    update_recstate();
}

void MainWindow::on_button_toggle_logging_clicked(bool check)
{
    switch (RecordState)
    {
    case Stopped:
        filename = ui.line_edit_directory->text();
        qnode.updateFilenames(filename);
        Q_EMIT startSignal();
        RecordState = Running;
        break;
    case Running:
        RecordState = Stopped;
        Q_EMIT stopSignal();
    }
    update_recstate();
}

void MainWindow::on_button_save_new_dir_clicked(bool check)
{
    qnode.updateFilenames(filename);

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

void MainWindow::check_topics()
{
    if (ui.checkbox_topics->isChecked())
    {
        switch (ui.tab_navigator->currentIndex())
        {
        case 0:
            ui.list_topics->selectAll();
            break;
        case 1:
            ui.list_subscription->selectAll();
            break;
        }
        ui.topic_counter->setText("All topics selected");
    }
    else
    {
        ui.list_topics->clear();
        ui.list_subscription->clear();
        ui.list_topics->clear();
        subscription.clear();
    }
    ui.checkbox_topics->setCheckState(Qt::Unchecked);
}

void MainWindow::filter_topics()
{
    QString filter = ui.line_edit_filter->text();
    QStringList filtered_sub = subscription.filter(filter);
    ui.list_subscription->clear();
    ui.list_subscription->addItems(filtered_sub);

    QStringList filtered_topics = all_topics.filter(filter);
    ui.list_topics->clear();
    ui.list_topics->addItems(filtered_topics);
}

void MainWindow::refresh_topic()
{
    ui.list_subscription->clear();
    subscription = qnode.lsSubscription();
    ui.list_subscription->addItems(subscription);

    ui.list_topics->clear();
    all_topics = qnode.lsAllTopics();
    ui.list_topics->addItems(all_topics);

    for (const auto &sub : subscription)
    {
        QList<QListWidgetItem *> disp_topics = ui.list_topics->findItems(sub, Qt::MatchContains);
        for (const auto &it : disp_topics)
        {
            it->setBackground(Qt::green);
            it->setFlags(it->flags() & ~Qt::ItemIsSelectable);
        }
    }
}

void MainWindow::on_button_subscribe_clicked(bool check)
{
    QList<QListWidgetItem *> add_topics = ui.list_topics->selectedItems();
    ui.topic_counter->setText(QString::number(add_topics.size()) + QString(" topics selected."));
    std::vector<std::string> topics;

    for (const auto &it : add_topics)
    {
        ui.list_topics->removeItemWidget(it);
        topics.push_back(it->text().toStdString());
        subscription += it->text();
    }
    qnode.addSubscription(topics);
    refresh_topic();
}

void MainWindow::on_button_unsubscribe_clicked(bool check)
{
    QList<QListWidgetItem *> rm_topics = ui.list_subscription->selectedItems();

    ui.topic_counter->setText(QString::number(rm_topics.size()) + QString(" topics selected."));

    std::vector<std::string> topics;
    for (const auto &it : rm_topics)
    {
        ui.list_subscription->removeItemWidget(it);
        topics.push_back(it->text().toStdString());
        subscription.removeAt(subscription.indexOf(it->text()));
    }
    qnode.rmSubscription(topics);
    refresh_topic();
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
