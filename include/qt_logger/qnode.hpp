/**
 * @file /include/qt_logger/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_logger_QNODE_HPP_
#define qt_logger_QNODE_HPP_

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
#include <boost/range/combine.hpp>
#endif

#include <string>
#include <QThread>
#include <QStringListModel>
#include <qt_logger/Mocap.h>
#include <qt_logger/Topic_for_log.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_logger
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
        Stopped,
        Running
    } state;

    ros::master::V_TopicInfo topic_infos;
    
    void start_logging();
    void stop_logging();

    void set_savefile(QString filename);
    void set_topics(std::vector<std::string> new_topics);
    void _set_subscription(std::vector<std::string> new_topics);

    QStringList query_curr_topics();
    QStringList query_all_topics();
    std::vector<std::string> topic_list = {"/mocap/UAV0"};
    

Q_SIGNALS:
    void rosLoopUpdate();
    void rosShutdown();

private:
    int init_argc;
    char **init_argv;

    std::string savefile;

    qt_logger::Mocap mocap[3];
    std::vector<ros::Subscriber> sub;
    ros::Subscriber mocapUAV0;
    ros::Subscriber mocapUAV1;
    ros::Subscriber mocapPayload;

    void _write_msg(const ros::MessageEvent<topic_tools::ShapeShifter const> &event);

    rosbag::Bag bag;
};

} // namespace qt_logger

#endif
