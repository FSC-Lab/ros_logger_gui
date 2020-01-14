/*****************************************************************************
** Includes
*****************************************************************************/
#include <queue>
#include <set>
#include <sstream>
#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#include "../include/ros_logger_gui/qnode.hpp"

typedef ros::SubscriptionCallbackHelperT<
	const ros::MessageEvent<topic_tools::ShapeShifter const> &>
	ros_sub_helper;
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_logger_gui
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
	ros::init(init_argc, init_argv, "ros_logger_gui");
	if (!ros::master::check())
	{
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.

	ros::NodeHandle n;

	options_.max_duration = ros::Duration(1200);
	options_.split = false;
	options_.append_date = false;

	// Add your ros communications here.
	// Add default topics

	ros::Time::waitForValid();

	addSubscription(getConfiguredTopics());
	start();
	start_time_ = ros::Time::now();
	return true;
}

void QNode::run()
{
	ros::Rate loop_rate(4);
	queue_ = new std::queue<rosbag::OutgoingMessage>;
	while (ros::ok())
	{
		ros::spinOnce();
		/*---------------------emit signals to tigger label update --------------------------*/

		/* signal a ros loop update  */
		Q_EMIT rosLoopUpdate();

		if (!ros::master::check())
		{
			delete queue_;
			Q_EMIT rosShutdown();
			break;
		}
		loop_rate.sleep();
	}
	delete queue_;
	Q_EMIT rosShutdown();
}

std::string QNode::showMaster()
{
	return ros::master::getURI();
}

bool QNode::startRecording()
{
	stop_signal_ = false;
	ros::NodeHandle nh;
	boost::mutex::scoped_lock record_lock_(record_mutex_);
	if (!nh.ok())
		return false;

	if (sub_.empty())
		return false;

	return true;
}

bool QNode::stopRecording()
{
	stop_signal_ = true;
	std::vector<std::string> prev_topics_;
	{
		boost::mutex::scoped_lock record_lock_(record_mutex_);

		for (auto &it : sub_)
		{
			it.second->shutdown();
			if (remember_topics_)
				prev_topics_.push_back(it.first);
		}
	}
	sub_.clear();

	if (remember_topics_)
	{
		addSubscription(prev_topics_);
	}
	else
	{
		addSubscription(getConfiguredTopics());
	}
	return true;
}

bool QNode::setOptions(std::map<std::string, bool> opts)
{
	if (opts.count("remember_topics") != 0)
	{
		remember_topics_ = opts.find("remember_topics")->second;
	}
	if (opts.count("append_date") != 0)
	{
		options_.append_date = opts.find("append_date")->second;
	}
	if (opts.count("split") != 0)
	{
		options_.split = opts.find("split")->second;
	}
}

std::vector<std::string> QNode::getConfiguredTopics()
{
	std::vector<std::string> topics;
	ros::NodeHandle n;
	n.getParam("/ros_logger_gui/topics", topics);
	return topics;
}

QString QNode::showBagSize()
{
	float bag_size_ = 0;
	if (queue_size_ > 1024)
	{
		const uint64_t KB = 1024;
		bag_size_ = ceil(1.0 * queue_size_ / KB);

		return QString::number(bag_size_) + "KB";
	}
	else if (queue_size_ > 1024 * 1024)
	{
		const uint64_t MB = 1024 * 1024;
		bag_size_ = ceil(1.0 * queue_size_ / MB);

		return QString::number(bag_size_) + "MB";
	}
	else
		return QString();
}

QStringList QNode::lsAllTopics()
{
	ros::master::getTopics(topic_infos);
	QStringList topic_names;
	for (const auto &it : topic_infos)
	{
		topic_names += QString::fromStdString(it.name);
	}
	return topic_names;
}

QStringList QNode::lsSubscription()
{
	//boost::mutex::scoped_lock record_lock(record_mutex_);
	QStringList topic_names;
	for (auto &it : sub_)
	{
		topic_names += QString::fromStdString(it.first);
	}
	return topic_names;
}

boost::shared_ptr<ros::Subscriber> QNode::subscribe(std::string const &topic)
{
	ros::NodeHandle nh;
	boost::shared_ptr<int> count(boost::make_shared<int>(options_.limit));
	boost::shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());

	ros::SubscribeOptions ops;
	ops.topic = topic;
	ops.queue_size = 100;
	ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
	ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
	ops.helper = boost::make_shared<ros_sub_helper>(
		boost::bind(&ros_logger_gui::QNode::doQueue, this, _1, topic));
	ops.transport_hints = options_.transport_hints;
	*sub = nh.subscribe(ops);

	return sub;
}

void QNode::addSubscription(const std::vector<std::string> topics)
{
	boost::mutex::scoped_lock record_lock(record_mutex_);

	for (const auto &it : topics)
		sub_.insert({it, subscribe(it)});
}

void QNode::rmSubscription(const std::vector<std::string> topics)
{
	boost::mutex::scoped_lock record_lock(record_mutex_);
	for (const auto &it : topics)
	{
		if (sub_.count(it) > 0)
		{
			sub_[it]->shutdown();
			sub_.erase(it);
		}
	}
}

void QNode::echoRaised(const char *message, ...)
{
	Q_EMIT rosRaise();
	echoString = QString(message);
}

template <class T>
std::string QNode::timeToStr(T ros_t)
{
	(void)ros_t;
	std::stringstream msg;
	const boost::posix_time::ptime now =
		boost::posix_time::second_clock::local_time();
	boost::posix_time::time_facet *const f =
		new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
	msg.imbue(std::locale(msg.getloc(), f));
	msg << now;
	return msg.str();
}

void QNode::doQueue(const ros::MessageEvent<topic_tools::ShapeShifter const> &event, std::string const &topic)
{
	if (!stop_signal_)
	{
		const ros::M_stringPtr &header_ptr = event.getConnectionHeaderPtr();

		const topic_tools::ShapeShifter::ConstPtr &msg = event.getMessage();
		rosbag::OutgoingMessage out(topic, msg, header_ptr, ros::Time::now());
		{
			boost::mutex::scoped_lock lock(queue_mutex_);
			queue_->push(out);
			queue_size_ += out.msg->size();
			while (options_.buffer_size > 0 && queue_size_ > options_.buffer_size)
			{
				rosbag::OutgoingMessage drop = queue_->front();
				queue_->pop();
				queue_size_ -= drop.msg->size();
			}
		}
	}
}

void QNode::startWriting()
{
	if (bag_active_)
	{
		return;
	}
	bag_active_ = true;

	bag_.setCompression(rosbag::compression::Uncompressed);
	bag_.setChunkThreshold(options_.chunk_size);

	try
	{
		bag_.open(write_filename_, rosbag::bagmode::Write);
	}
	catch (rosbag::BagException e)
	{
		echoRaised("Error writing: %s", e.what());
		exit_code_ = 1;
		ros::shutdown();
	}
}

void QNode::stopWriting()
{
	bag_.close();
	rename(write_filename_.c_str(), target_filename_.c_str());
	bag_active_ = false;
}

bool QNode::checkSize()
{
	if (options_.max_size > 0)
	{
		if (bag_.getSize() > options_.max_size)
		{
			if (options_.split)
			{
				stopWriting();
				split_count_++;
				checkNumSplits();
				startWriting();
			}
			else
			{
				ros::shutdown();
				return true;
			}
		}
	}
	return false;
}

bool QNode::checkDuration(const ros::Time &t)
{
	if (options_.max_duration > ros::Duration(0))
	{
		if (t - start_time_ > options_.max_duration)
		{
			if (options_.split)
			{
				while (start_time_ + options_.max_duration < t)
				{
					stopWriting();
					split_count_++;
					checkNumSplits();
					start_time_ += options_.max_duration;
					startWriting();
				}
			}
			else
			{
				ros::shutdown();
				return true;
			}
		}
	}
	return false;
}

bool QNode::doRecord()
{
	// Open bag file for writing
	startWriting();

	// Schedule the disk space check
	warn_next_ = ros::WallTime();

	try
	{
		checkDisk();
	}
	catch (rosbag::BagException &ex)
	{
		ROS_ERROR_STREAM(ex.what());
		exit_code_ = 1;
		stopWriting();
		return false;
	}

	check_disk_next_ = ros::WallTime::now() + ros::WallDuration().fromSec(20.0);

	// Technically the queue_mutex_ should be locked while checking empty.
	// Except it should only get checked if the node is not ok, and thus
	// it shouldn't be in contention.
	ros::NodeHandle nh;
	bool done = false;

	while (nh.ok() || !queue_->empty())
	{
		boost::unique_lock<boost::mutex> lock(queue_mutex_);

		while (queue_->empty())
		{
			if (stop_signal_)
			{
				lock.release()->unlock();
				done = true;
				break;
			}
			boost::xtime xt;
#if BOOST_VERSION >= 105000
			boost::xtime_get(&xt, boost::TIME_UTC_);
#else
			boost::xtime_get(&xt, boost::TIME_UTC);
#endif
			xt.nsec += 250000000;
			queue_condition_.timed_wait(lock, xt);
			if (checkDuration(ros::Time::now()))
			{
				done = true;
				break;
			}
		}
		if (done)
			break;

		rosbag::OutgoingMessage out = queue_->front();
		queue_->pop();
		queue_size_ -= out.msg->size();

		lock.release()->unlock();

		if (checkSize())
			break;

		if (checkDuration(out.time))
			break;

		try
		{
			if (scheduledCheckDisk() && checkLogging())
				bag_.write(out.topic, out.time, *out.msg, out.connection_header);
		}
		catch (rosbag::BagException &ex)
		{
			ROS_ERROR_STREAM(ex.what());
			exit_code_ = 1;
			break;
		}
	}

	stopWriting();
	return done;
}

bool QNode::scheduledCheckDisk()
{
	boost::mutex::scoped_lock lock(check_disk_mutex_);

	if (ros::WallTime::now() < check_disk_next_)
		return true;

	check_disk_next_ += ros::WallDuration().fromSec(20.0);
	return checkDisk();
}

bool QNode::checkDisk()
{
	const uint GB = (1024 * 1024) * 1024;
	std::string bag_name = bag_.getFileName();
#if BOOST_FILESYSTEM_VERSION < 3
	struct statvfs fiData;
	if ((statvfs(bag_name.c_str(), &fiData)) < 0)
	{
		echoRaised("Failed to check filesystem stats.");
		return true;
	}
	unsigned long long free = 1ULL * fiData.f_bsize * fiData.f_bavail;
	const double free_gb = ceil(1.0 * free / GB);
	if (free < options_.min_space)
	{
		echoError("%f GB of free space is available on disk with %s.  Disabling recording.", free_gb, bag_.getFileName().c_str());
		writing_enabled_ = false;
		return false;
	}
	else if (free < 5 * options_.min_space)
	{
		echoRaised("%f GB of free space is available on disk with %s.", free_gb, bag_.getFileName().c_str());
	}
	else
	{
		writing_enabled_ = true;
	}
#else
	boost::filesystem::path p(boost::filesystem::system_complete(bag_name.c_str()));
	boost::filesystem::space_info info;
	unsigned long long free = 1ULL * info.available;
	const double free_gb = ceil(1.0 * free / GB);
	try
	{
		info = boost::filesystem::space(p.parent_path());
	}
	catch (boost::filesystem::filesystem_error &e)
	{
		echoRaised("Failed to check filesystem stats [%s].", e.what());
		writing_enabled_ = false;
		return false;
	}
	if (info.available < options_.min_space)
	{
		writing_enabled_ = false;
		throw rosbag::BagException(std::to_string(free_gb) + "GB of free space is available on disk with " + bag_.getFileName() + ". Disabling recording.");
	}
	else if (info.available < 5 * options_.min_space)
	{
		echoRaised("%f GB of free space is available on disk with %s.", free_gb, bag_.getFileName().c_str());
		writing_enabled_ = true;
	}
	else
	{
		writing_enabled_ = true;
	}
#endif
	return true;
}

void QNode::checkNumSplits()
{
	if (options_.max_splits > 0)
	{
		current_files_.push_back(target_filename_);
		if (current_files_.size() > options_.max_splits)
		{
			int err = unlink(current_files_.front().c_str());
			if (err != 0)
			{
				echoRaised("Unable to remove %s: %s", current_files_.front().c_str(), strerror(errno));
			}
			current_files_.pop_front();
		}
	}
}

bool QNode::checkLogging()
{
	if (writing_enabled_)
		return true;

	ros::WallTime now = ros::WallTime::now();
	if (now >= warn_next_)
	{
		warn_next_ = now + ros::WallDuration().fromSec(5.0);

		echoRaised("Not logging message because logging disabled.  Most likely cause is a full disk.");
	}
	return false;
}

QString QNode::formatFilenames(QString filename)
{
	std::vector<std::string> parts;

	std::string prefix = filename.toStdString();
	size_t ind = prefix.rfind(".bag");

	if (ind != std::string::npos && ind == prefix.size() - 4)
	{
		prefix.erase(ind);
	}

	if (prefix.length() > 0)
		parts.push_back(prefix);
	if (options_.append_date)
		parts.push_back(timeToStr(ros::WallTime::now()));
	if (options_.split)
		parts.push_back(boost::lexical_cast<std::string>(split_count_));

	if (parts.size() == 0)
	{
		echoRaised("Bag filename is empty (neither of these was specified: prefix, append_date, split)");
	}

	target_filename_ = parts[0];
	for (unsigned int i = 1; i < parts.size(); i++)
	{
		target_filename_ += std::string("_") + parts[i];
	}

	target_filename_ += std::string(".bag");
	return QString::fromStdString(target_filename_);
}

void QNode::updateFilenames(QString filename)
{
	target_filename_ = filename.toStdString();
	write_filename_ = filename.toStdString() + std::string(".active");
}

// namespace ros_logger_gui

} // namespace ros_logger_gui
