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

	// Add your ros communications here.
	// Default Topics
	for (const auto &topic : topic_list)
	{
		sub.push_back(n.subscribe(topic, 1000, &QNode::_write_msg, this));
	}
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

QStringList QNode::query_all_topics()
{
	ros::master::getTopics(topic_infos);
	QStringList topic_names;
	for (const auto &topic : topic_infos)
	{
		topic_names += QString::fromStdString(topic.name);
	}
	return topic_names;
}

QStringList QNode::query_curr_topics()
{
	QStringList topic_names;
	for (const auto &topic : topic_list)
	{
		topic_names += QString::fromStdString(topic);
	}
	return topic_names;
}

void QNode::set_topics(std::vector<std::string> new_topics)
{
	topic_list.insert(topic_list.end(), new_topics.begin(), new_topics.end());
	_set_subscription(new_topics);
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

void QNode::_write_msg(const ros::MessageEvent<topic_tools::ShapeShifter const> &event)
{
	if (state == Running)
	{
		ros::M_string &header = event.getConnectionHeader();
		const topic_tools::ShapeShifter::ConstPtr &msg = event.getMessage();
		bag.write(header["topic"], ros::Time::now(), msg);
	}
}

void QNode::_set_subscription(std::vector<std::string> new_topics)
{
	ros::NodeHandle n;

	if (new_topics.empty())
	{
		sub.clear();
		topic_list.clear();
		topic_list.push_back("clock");
		sub.push_back(n.subscribe("clock", 1000, &QNode::_write_msg, this));
	}
	else
	{
		for (const auto topic : new_topics)
		{
			sub.push_back(n.subscribe(topic, 1000, &QNode::_write_msg, this));
		}
	}
}

} // namespace qt_recorder
