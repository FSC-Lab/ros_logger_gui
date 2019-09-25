/**
 * @file /include/qt_recorder/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_recorder_QNODE_HPP_
#define qt_recorder_QNODE_HPP_

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
#include <qt_recorder/Mocap.h>
#include <qt_recorder/Topic_for_log.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_recorder
{

/*****************************************************************************
** Class
*****************************************************************************/

struct uav_record
{
    qt_recorder::Topic_for_log log;
};

class QNode : public QThread
{
    Q_OBJECT
public:
    QNode(int argc, char **argv);
    virtual ~QNode();
    bool init();
    void run();

    /*********************
	** Recording
	**********************/

    enum RecordingState
    {
        Stopped,
        Running
    } state;

    ros::master::V_TopicInfo topic_infos;
    
    void start_recording();
    void stop_recording();

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

    qt_recorder::Mocap mocap[3];
    std::vector<ros::Subscriber> sub;
    ros::Subscriber mocapUAV0;
    ros::Subscriber mocapUAV1;
    ros::Subscriber mocapPayload;

    void _write_msg(const ros::MessageEvent<topic_tools::ShapeShifter const> &event);

    rosbag::Bag bag;
};

} // namespace qt_recorder

#endif
