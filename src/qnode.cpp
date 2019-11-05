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

#include <sstream>
#include "../include/ros_logger_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_logger_gui
{

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char **argv) : init_argc(argc),
									  init_argv(argv),
									  state(Unstarted)
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
	ros::init(init_argc, init_argv, "ros_logger_gui");
	if (!ros::master::check())
	{
		return false;
		state = Unstarted;
	}
	else
	{
		ros::start(); // explicitly needed since our nodehandle is going out of scope.
		state = Stopped;
	}
	ros::NodeHandle n;

	// Add your ros communications here.
	// Add default topics


	for (const auto &it : get_configured_topics())
	{
		sub.insert({it, n.subscribe(it, 1000, &QNode::write_msg, this)});
	}
	start();
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

std::vector<std::string> QNode::get_configured_topics()
{
	std::vector<std::string> topics;
	ros::NodeHandle n;
	n.getParam("/ros_logger_gui/topics",  topics);
	return topics;
}

QStringList QNode::get_all_topics()
{
	ros::master::getTopics(topic_infos);
	QStringList topic_names;
	for (const auto &topic : topic_infos)
	{
		topic_names += QString::fromStdString(topic.name);
	}
	return topic_names;
}

QStringList QNode::get_subscription()
{
	QStringList topic_names;
	for (auto &it : sub)
	{
		topic_names += QString::fromStdString(it.first);
	}
	return topic_names;
}

void QNode::add_subscription(std::vector<std::string> topics)
{
	ros::NodeHandle n;

	for (const auto &it : topics)
	{
		sub.insert({it, n.subscribe(it, 1000, &QNode::write_msg, this)});
	}
}

void QNode::rm_subscription(std::vector<std::string> topics)
{
	for (const auto &it : topics)
	{
		sub[it].shutdown();
		sub.erase(it);
	}
}

void QNode::reset_subscription()
{
	ros::NodeHandle n;

	for (auto &it : sub)
	{
		it.second.shutdown();
	}
	sub.clear();
	for (const auto &it : get_configured_topics())
	{
		sub.insert({it, n.subscribe(it, 1000, &QNode::write_msg, this)});
	}
}


void QNode::start_logging()
{
	if (bag_active_)
	{
		return;
	}
	bag_active_ = true;
	state = Running;

	bag.setCompression(rosbag::compression::Uncompressed);
	try
	{
		bag.open(bag_file, rosbag::bagmode::Write);
		Q_EMIT logStateChanged();
	}
	catch (rosbag::BagException err)
	{
		ROS_ERROR("Error opening bag: %s", err.what());
		bag_active_ = false;
	}
}

void QNode::stop_logging()
{
	if (!bag_active_)
		return;
	bag.close();
	Q_EMIT logStateChanged();
	state = Stopped;
	reset_subscription();
}

void QNode::set_savefile(QString filename)
{
	bag_file = filename.toStdString();
}

void QNode::write_msg(const ros::MessageEvent<topic_tools::ShapeShifter const> &event)
{
	if (state == Running)
	{
		ros::M_string &header = event.getConnectionHeader();
		const topic_tools::ShapeShifter::ConstPtr &msg = event.getMessage();
		bag.write(header["topic"], ros::Time::now(), msg);
	}
}

} // namespace ros_logger_gui
