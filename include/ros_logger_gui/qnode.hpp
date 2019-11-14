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
#include <topic_tools/shape_shifter.h>
#include <rosbag/query.h>
#include <rosbag/bag.h>
#include <rosbag/recorder.h>
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/message_event.h>
#include <ros/master.h>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#endif

#if BOOST_FILESYSTEM_VERSION < 3
#include <sys/statvfs.h>
#endif

#include <std_msgs/String.h>
#include <QThread>
#include <QStringListModel>

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

    ros::master::V_TopicInfo topic_infos;
    // Subscription management
    QStringList lsSubscription();
    QStringList lsAllTopics();
    void addSubscription(const std::vector<std::string> topics);
    void rmSubscription(const std::vector<std::string> topics);
    void echoRaised(const char *message, ...);
    void echoWarning(const std::string message);

    void updateFilenames(QString filename);
    void startWriting();
    void stopWriting();
    void doQueue(const ros::MessageEvent<topic_tools::ShapeShifter const> &event, std::string const &topic);

    std::map<std::string, boost::shared_ptr<ros::Subscriber>> sub_;

    std::vector<std::string> get_configured_topics();

    boost::shared_ptr<ros::Subscriber> subscribe(std::string const &topic);

    QString echoString;
    bool record_signal_ = true;
    boost::mutex record_mutex_;

public Q_SLOTS:
    void startRecord();
    void stopRecord();
    void doRecord();

Q_SIGNALS:
    void rosRaise();
    void rosLoopUpdate();
    void rosLaunch();
    void logStateChanged();

private:
    int init_argc;
    char **init_argv;

    rosbag::RecorderOptions options_;
    bool bag_active_ = false;

    // unsigned long long min_space_ = 1024 * 1024 * 1024;
    bool checkLogging();

    bool scheduledCheckDisk();
    bool checkDisk();
    void checkNumSplits();
    bool checkSize();
    bool checkDuration(const ros::Time &);
    void doCheckMaster(ros::TimerEvent const &e, ros::NodeHandle &node_handle);

    template <class T>
    static std::string timeToStr(T ros_t);
    rosbag::Bag bag_;
    std::string target_filename_;
    std::string write_filename_;
    std::list<std::string> current_files_;

    int exit_code_;

    // Queue and data control
    boost::condition_variable_any queue_condition_;
    boost::mutex queue_mutex_;
    std::queue<rosbag::OutgoingMessage> *queue_;
    uint64_t queue_size_;
    uint64_t max_queue_size_;

    uint64_t split_count_;

    // Time variables
    ros::Time last_buffer_warn_;
    ros::Time start_time_;
    bool writing_enabled_;
    boost::mutex check_disk_mutex_;
    ros::WallTime check_disk_next_;
    ros::WallTime warn_next_;
};

} // namespace ros_logger_gui

#endif
