/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sstream>
#include "../include/qt_recorder/qnode.hpp"

#define VARNAME(Var) (#Var)

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_recorder
{

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char **argv) : init_argc(argc),
									  init_argv(argv)
{
}

QNode::~QNode()
{
	if (ros::isStarted())
	{
		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
	}
	wait();
}

bool QNode::init()
{
	ros::init(init_argc, init_argv, "qt_recorder");
	if (!ros::master::check())
	{
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	ros::master::getTopics(topic_infos);

	// Add your ros communications here.
	// Default Topics
	sub.push_back(n.subscribe("/mocap/UAV0", 1000, &QNode::write_msg, this));
	sub.push_back(n.subscribe("/mocap/UAV1", 1000, &QNode::write_msg, this));

	//mocapUAV1 = n.subscribe<qt_recorder::Mocap>("/mocap/UAV1", 1000, &QNode::sub_mocapUAV1, this);
	//mocapPayload = n.subscribe<qt_recorder::Mocap>("/mocap/Payload", 1000, &QNode::sub_mocapPayload, this);

	start();
	state = Stopped;

	return true;
}

void QNode::run()
{
	ros::Rate loop_rate(4);
	while (ros::ok())
	{
		ros::spinOnce();
		/*---------------------emit signals to tigger label update --------------------------*/

		/* signal a ros loop update  */
		Q_EMIT rosLoopUpdate();

		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::write_msg(const ros::MessageEvent<topic_tools::ShapeShifter const> &event)
{
	if ( state == Running )
	{
		ros::M_string &header = event.getConnectionHeader();
		const topic_tools::ShapeShifter::ConstPtr &msg = event.getMessage();
		bag.write(header["topic"], ros::Time::now(), msg);
	}
}

void QNode::add_subscription()
{
	//n.subscribe<qt_recorder::Mocap>("/mocap/UAV0", 1000, &QNode::sub_mocapUAV0, this);
}

void QNode::get_topic()
{
	/*	int i = 1;
	std::vector<std::string> lv_elems;

	char lc_delim[2];
	lc_delim[0] = '/';
	lc_delim[1] = '\0';

	//boost::algorithm::split(lv_elems, topic_infos[0].name, boost::algorithm::is_any_of(lc_delim));

	if (lv_elems[0] == "vicon")*/
}

void QNode::start_recording()
{
	bag.open(savefile, rosbag::bagmode::Write);
	state = Running;
}

void QNode::stop_recording()
{
	bag.close();
	state = Stopped;
}

void QNode::set_savefile(QString filename)
{
	savefile = filename.toStdString();
}



} // namespace qt_recorder
