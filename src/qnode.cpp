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

	get_configured_topics();

	for (const auto &topic : sub_topics)
	{
		sub.push_back(n.subscribe(topic, 1000, &QNode::write_msg, this));
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

void QNode::get_configured_topics()
{
	ros::NodeHandle n;
	n.getParam("/ros_logger_gui/topics", sub_topics);
}

QStringList QNode::get_topics()
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
	for (auto &s_: sub_topics)
	{
		topic_names += QString::fromStdString(s_);
	}
	return topic_names;
}

void QNode::set_topics(std::vector<std::string> new_topics)
{
	sub_topics.insert(sub_topics.end(), new_topics.begin(), new_topics.end());
	set_subscription(new_topics);
}

void QNode::start_logging()
{
	if(bag_active_)
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
	clear_subscription();

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

void QNode::clear_subscription()
{
	for (auto &s_: sub)
	{
		s_.shutdown();
	}
	sub.clear();
}

void QNode::set_subscription(std::vector<std::string> new_topics)
{
	ros::NodeHandle n;
	if (new_topics.empty())
	{
		clear_subscription();
		get_configured_topics();
		for (const auto &s_ : sub_topics)
			sub.push_back(n.subscribe(s_, 1000, &QNode::write_msg, this));
			
		//sub_topics.push_back("clock");
		//sub.push_back(n.subscribe("clock", 1000, &QNode::write_msg, this));
	}
	else
	{
		for (const auto topic : new_topics)
		{
			sub.push_back(n.subscribe(topic, 1000, &QNode::write_msg, this));
		}
	}
}

} // namespace ros_logger_gui
