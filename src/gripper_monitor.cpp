/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <control_msgs/GripperCommandActionFeedback.h>
#include <control_msgs/GripperCommandActionGoal.h>
#include <control_msgs/GripperCommandActionResult.h>
#include <pr2_grasping/GraspingGroup.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include "RobotUtils.hpp"
#include "PkgUtils.hpp"
#include "Config.hpp"


#define TIME_WINDOW			5
#define TIME_WINDOW_STEADY	1
#define POS_STD_THRES		1E-5
#define EFFORT_STD_THRES	1E-2


/***** Accumulators type definition *****/
typedef boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::variance> > AccType;

/***** Global variables *****/
boost::mutex mutex;
std::list<control_msgs::GripperCommandActionFeedback> feedback;
std::list< std::pair<AccType, AccType> > acc;
std::list< std::pair<AccType, AccType> > stdAcc;
std::pair<bool, ros::Time> steady = std::make_pair(false, ros::Time());


/**************************************************/
void feedbackReceived(const control_msgs::GripperCommandActionFeedbackConstPtr &msg_,
					  ros::Publisher &stuckPublisher_)
{
	mutex.lock();

	// Track feedback messages to know the elapsed time
	feedback.push_back(*msg_);
	// Add new accumulator to get the stats of the last elapsed period
	acc.push_back(std::pair<AccType, AccType>());
	stdAcc.push_back(std::pair<AccType, AccType>());

	for (std::list< std::pair<AccType, AccType> >::iterator it = acc.begin(); it != acc.end(); it++)
	{
		it->first(msg_->feedback.position);
		it->second(msg_->feedback.effort);
	}


	double posMean = boost::accumulators::mean(acc.front().first);
	double posStdDev = sqrt(boost::accumulators::variance(acc.front().first));
	double effortMean = boost::accumulators::mean(acc.front().second);
	double effortStdDev = sqrt(boost::accumulators::variance(acc.front().second));


	for (std::list< std::pair<AccType, AccType> >::iterator it = stdAcc.begin(); it != stdAcc.end(); it++)
	{
		it->first(posStdDev);
		it->second(effortStdDev);
	}


	ROS_DEBUG("pos: (%.3f, %.5f) -- eff: (%.3f, %.5f) -- acc: %zu", posMean, posStdDev, effortMean, effortStdDev, acc.size());

	double posStd2 = sqrt(boost::accumulators::variance(stdAcc.front().first));
	double effStd2 = sqrt(boost::accumulators::variance(stdAcc.front().second));
	ROS_DEBUG("posStd2: %.4E -- effStd2: %.4E -- stdAcc: %zu", posStd2, effStd2, stdAcc.size());


	ros::Duration elapsed = feedback.back().header.stamp - feedback.front().header.stamp;
	if (msg_->feedback.effort < 0 &&
			elapsed.toSec() > TIME_WINDOW &&
			posStd2 < POS_STD_THRES &&
			effStd2 < EFFORT_STD_THRES)
	{
		ROS_INFO_ONCE("...steady zone reached");
		if (!steady.first)
		{
			ROS_DEBUG("setting comparison time");
			steady.second = feedback.back().header.stamp;
			steady.first = true;
		}
		else
		{
			ros::Duration steadyElapsed = feedback.back().header.stamp - steady.second;
			ROS_DEBUG("steady elapsed: %.4f", steadyElapsed.toSec());
			if (steadyElapsed.toSec() >= TIME_WINDOW_STEADY)
			{
				ROS_INFO("...gripper stuck!");
				std_msgs::Bool stuckMsg;
				stuckMsg.data = true;
				stuckPublisher_.publish(stuckMsg);

				steady.first = false;
			}
		}
	}

	// Remove old stuff
	while (elapsed.toSec() > TIME_WINDOW)
	{
		feedback.pop_front();
		acc.pop_front();
		stdAcc.pop_front();
		elapsed = feedback.back().header.stamp - feedback.front().header.stamp;
	}

	mutex.unlock();
}


/**************************************************/
void newGoal(const control_msgs::GripperCommandActionGoalConstPtr & msg_)
{
	mutex.lock();
	feedback.clear();
	acc.clear();
	stdAcc.clear();
	steady.first = false;
	mutex.unlock();
}


/**************************************************/
int main(int argn_, char **argv_)
{
	ros::init(argn_, argv_, "gripper_monitor");
	ros::NodeHandle handler;


	/********** Start spinning **********/
	ros::AsyncSpinner spinner(3);
	spinner.start();


	/********** Load the node's configuration **********/
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(PkgUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + PkgUtils::getConfigPath());

	if (Config::get()["gmonitorDebug"].as<bool>())
	{
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
			ros::console::notifyLoggerLevelsChanged();
	}


	/********** Retrieve the group used for grasping **********/
	std::string serviceName = "/pr2_grasping/effector_name";
	while (!ros::service::waitForService(serviceName, ros::Duration(1)))
		ros::Duration(0.5).sleep();

	ROS_INFO("Retrieving grasping group");
	pr2_grasping::GraspingGroup srv;
	srv.response.result = false;
	while (!srv.response.result)
	{
		if (ros::service::call(serviceName, srv))
			ROS_INFO("Grasping group query %s", srv.response.result ? "SUCCESSFUL" : "FAILED, retrying...");
		ros::Duration(0.5).sleep();
	}
	ROS_INFO("Grasping group: %s", srv.response.groupName.c_str());


	/********** Set subscriptions/publishers **********/
	ros::Publisher stuckPublisher = handler.advertise<std_msgs::Bool>("/pr2_grasping/gripper_action_stuck", 1);
	std::string baseTopic = RobotUtils::getGripperTopic(srv.response.groupName);
	ros::Subscriber feedbackSubscriber = handler.subscribe<control_msgs::GripperCommandActionFeedback>(baseTopic + "/feedback", 1, boost::bind(feedbackReceived, _1, stuckPublisher));
	ros::Subscriber goalSubscriber = handler.subscribe<control_msgs::GripperCommandActionGoal>(baseTopic + "/goal", 1, newGoal);


	ros::waitForShutdown();
	return EXIT_SUCCESS;
}
