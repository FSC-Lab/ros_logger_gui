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
#include <ros/ros.h>
#endif
#include <ros/master.h>
#include <rosbag/bag.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <qt_recorder/Mocap.h>
#include <qt_recorder/Topic_for_log.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Eigen>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_recorder {

/*****************************************************************************
** Class
*****************************************************************************/


struct uav_record {
    qt_recorder::Topic_for_log log;

};

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

        enum RecordingState {
            Stopped,
            Running
        } state;

        qt_recorder::Mocap GetMocap(int ID);


        void start_recording();
        void stop_recording();
        void update_recording();
        void set_savefile( QString filename );
        void get_topic();


Q_SIGNALS:
        void rosLoopUpdate();
        void rosShutdown();

private:
	int init_argc;
	char** init_argv;

        std::string savefile;

        qt_recorder::Mocap mocap[3];
        qt_recorder::Mocap mocap_payload;

        ros::Subscriber mocapUAV0;
        ros::Subscriber mocapUAV1;
        ros::Subscriber mocapPayload;

        void sub_mocapUAV0(const qt_recorder::Mocap::ConstPtr& msg);
        void sub_mocapUAV1(const qt_recorder::Mocap::ConstPtr& msg);
        void sub_mocapPayload(const qt_recorder::Mocap::ConstPtr& msg);

        void write_multi_topic(auto &topic, std::string tag);
        void write_topic(auto _topic, std::string tag);
        void _write_float32_multiarray3(auto raw_msg, std::string tag);
//        std_msgs::Float32               _write_float32(float msg);
//        std_msgs::Int32                 _write_int32(int msg);


        ros::master::V_TopicInfo topic_infos;
        rosbag::Bag bag;

    QStringListModel logging_model;
};

}  // namespace qt_recorder

#endif
