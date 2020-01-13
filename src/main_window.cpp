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
      file_name("/rosbag.bag"),
      file_path(QDir::homePath() + "/ROS_bags")
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    bool init_ros_ok = qnode.init();
    if (!init_ros_ok)
    {
        RecordState = Unstarted;
        init_ros_ok = qnode.init();
    }
    else
    {
        ui.label_master->setText("Connected to master at:" + QString::fromStdString(qnode.showMaster()));
        RecordState = Stopped;
    }

    /*********************
    ** Auto Start
    **********************/

    QDir save_dir(file_path);

    if (!save_dir.exists())
    {
        save_dir.mkpath(file_path);
    }

    subscription = qnode.lsSubscription();

    ui.button_save->setEnabled(false);
    ui.save_flag->setText("No Recorded Bag Data");
    ui.button_select_all->setText("Select All");

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

void MainWindow::showTopicMsg()
{
    QMessageBox msgBox;
}

void MainWindow::updateUI()
{
    switch (RecordState)
    {
    case Unstarted:
        ui.logging_status_flag->setText("<font color='red'>ROS is not started. Retrying</font>");
        ui.button_toggle_recording->setEnabled(false);
        ui.button_subscribe->setEnabled(false);
        ui.button_unsubscribe->setEnabled(false);
        ui.button_reconnect->setText("Reconnect");
        break;
    case Stopped:
        ui.logging_status_flag->setText("<font color='red'>Data logging stopped</font>");
        ui.button_toggle_recording->setText("Start recording");
        ui.button_subscribe->setEnabled(true);
        ui.button_unsubscribe->setEnabled(true);
        ui.button_reconnect->setText("Refresh Screen");
        break;
    case Running:
        ui.logging_status_flag->setText("<font color='green'>Data logging running</font>");
        ui.button_toggle_recording->setText("Stop recording");
        ui.button_subscribe->setEnabled(true);
        ui.button_unsubscribe->setEnabled(true);
        ui.button_reconnect->setText("Refresh Screen");
        break;
    }
}

/*****************************************************************************
** Implementation [Buttons]
*****************************************************************************/

void MainWindow::on_button_reconnect_clicked(bool check)
{
    bool init_ros_ok = false;
    if (RecordState == Unstarted)
    {
        init_ros_ok = qnode.init();
    }
    if (init_ros_ok || RecordState != Unstarted)
    {
        ui.label_master->setText("Connected to master at: " + QString::fromStdString(qnode.showMaster()));
        updateUI();
        updateTopics();
    }
}

void MainWindow::on_button_toggle_recording_clicked(bool check)
{
    switch (RecordState)
    {
    case Stopped:
        if (qnode.startRecording())
            RecordState = Running;
        break;
    case Running:
        if (qnode.stopRecording())
            RecordState = Stopped;

        if (qnode.showBagSize().isNull())
        {
            ui.save_flag->setText("No data recorded");
        }
        else
        {
            ui.save_flag->setText("Unsaved recorder data. Size is " + qnode.showBagSize());
            ui.button_save->setEnabled(true);
        }
    }
    updateUI();
    updateTopics();
}

void MainWindow::on_button_save_clicked(bool check)
{
    QString write_name = file_path + qnode.formatFilenames(file_name);
    write_name = QFileDialog::getSaveFileName(this,
                                             tr("Save Rosbag"),
                                             write_name,
                                             tr("ROS bag (*.bag)"));
    qnode.updateFilenames(write_name);

    if (qnode.doRecord())
    {
        ui.save_flag->setText("ROS bag saved");
        ui.button_save->setEnabled(false);
    }
    write_name = file_path + qnode.formatFilenames(file_name);
}

void MainWindow::on_checkBox_add_date_stateChanged(int)
{
    std::map<std::string, bool> date;
    if (ui.checkBox_add_date->isChecked())
    {
        date["append_date"] = true;
    }
    else
    {
        date["append_date"] = false;
    }
    qnode.setOptions(date);
}

void MainWindow::on_checkBox_remember_stateChanged(int)
{
    std::map<std::string, bool> remember;
    if (ui.checkBox_remember->isChecked())
    {
        remember["remember_topics"] = true;
    }
    else
    {
        remember["remember_topics"] = false;
    }
    qnode.setOptions(remember);
}

void MainWindow::on_list_subscription_itemSelectionChanged()
{
    int selected = ui.list_subscription->selectedItems().count();
    int all = ui.list_subscription->count();
    if (selected == all)
    {
        ui.button_select_all->setText("Unselect All");
        selected_all_ = true;
    }
    else
    {
        ui.button_select_all->setText("Select All");
        selected_all_ = false;
    }
}

void MainWindow::on_list_topics_itemSelectionChanged()
{
    int selected = ui.list_topics->selectedItems().count();
    int all = ui.list_topics->count();
    if (selected == all)
    {
        ui.button_select_all->setText("Unselect All");
        selected_all_ = true;
    }
    else
    {
        ui.button_select_all->setText("Select All");
        selected_all_ = false;
    }
}

void MainWindow::on_button_select_all_clicked(bool check)
{
    switch (ui.tab_navigator->currentIndex())
    {
    case 0:
        if (!selected_all_)
        {
            ui.list_topics->selectAll();
        }
        else
        {
            ui.list_topics->clear();
            updateTopics();
        }
        break;
    case 1:
        if (!selected_all_)
        {
            ui.list_subscription->selectAll();
        }
        else
        {
            ui.list_subscription->clear();
            updateTopics();
        }
        break;
    }
    ui.topic_counter->setText("All topics selected");
}

void MainWindow::on_line_edit_filter_textChanged(QString)
{
    QString filter = ui.line_edit_filter->text();
    QStringList filtered_sub = subscription.filter(filter);
    ui.list_subscription->clear();
    ui.list_subscription->addItems(filtered_sub);

    QStringList filtered_topics = all_topics.filter(filter);
    ui.list_topics->clear();
    ui.list_topics->addItems(filtered_topics);
}

void MainWindow::updateTopics()
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
    updateTopics();
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
    updateTopics();
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings()
{
    QSettings settings("Qt-Ros Package", "ros_logger_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings()
{
    QSettings settings("Qt-Ros Package", "ros_logger_gui");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

} // namespace ros_logger_gui
