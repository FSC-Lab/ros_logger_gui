/**
 * @file /include/ros_logger_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ros_logger_gui_QNODE_HPP_
#define ros_logger_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/master.h>
#include <ros/network.h>
#include <ros/ros.h>
#include <ros/message_event.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <topic_tools/shape_shifter.h>
#include <boost/foreach.hpp>
#endif

#include <std_msgs/String.h>

#include <QThread>
#include <QStringListModel>

#define foreach BOOST_FOREACH

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_logger_gui
{

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
    Q_OBJECT
public:
    QNode(int argc, char **argv);
    virtual ~QNode();
    bool init();
    void run();

    /*********************
	** Logging
	**********************/

    enum LoggingState
    {
        Unstarted,
        Stopped,
        Running
    } state;

    ros::master::V_TopicInfo topic_infos;

    void start_logging();
    void stop_logging();
    void set_savefile(QString filename);
    void set_topics(std::vector<std::string> new_topics);

    QStringList get_curr_topics();
    QStringList get_all_topics();
    std::vector<std::string> sub_topics;
    void get_configured_topics();

Q_SIGNALS:
    void rosLoopUpdate();
    void rosShutdown();
    void rosLaunch();

private:
    int init_argc;
    char **init_argv;
    bool bag_active_;

    std::vector<ros::Subscriber> sub;
    void set_subscription(std::vector<std::string> new_topics);
    void clear_subscription();
    void write_msg(const ros::MessageEvent<topic_tools::ShapeShifter const> &event);

    rosbag::Bag bag;
    std::string bag_file;
};

} // namespace ros_logger_gui

#endif
