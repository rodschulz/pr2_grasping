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
#include "GraspingUtils.hpp"
#include "Config.hpp"


/***** Accumulators type definition *****/
typedef boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::variance> > AccType;

/***** Global variables *****/
boost::mutex mutex;
std::list<control_msgs::GripperCommandActionFeedback> feedback;
std::list< std::pair<AccType, AccType> > acc;


/**************************************************/
void feedbackReceived(const control_msgs::GripperCommandActionFeedbackConstPtr &msg_,
					  ros::Publisher &stuckPublisher_)
{
	mutex.lock();

	// Track feedback messages to know the elapsed time
	feedback.push_back(*msg_);
	// Add new accumulator to get the stats of the last elapsed period
	acc.push_back(std::pair<AccType, AccType>());

	for (std::list< std::pair<AccType, AccType> >::iterator it = acc.begin(); it != acc.end(); it++)
	{
		it->first(msg_->feedback.position);
		it->second(msg_->feedback.effort);
	}


	double posMean = boost::accumulators::mean(acc.front().first);
	double posStdDev = sqrt(boost::accumulators::variance(acc.front().first));
	double effortMean = boost::accumulators::mean(acc.front().second);
	double effortStdDev = sqrt(boost::accumulators::variance(acc.front().second));


	ROS_DEBUG("pos: (%.5f, %.5f) -- eff: (%.5f, %.5f)", posMean, posStdDev, effortMean, effortStdDev);


	ros::Duration elapsed = feedback.back().header.stamp - feedback.front().header.stamp;
	if (elapsed.toSec() > 3 && posStdDev < 0.0005 && effortStdDev < 0.2)
	{
		std_msgs::Bool stuckMsg;
		stuckMsg.data = true;
		stuckPublisher_.publish(stuckMsg);
	}

	// Remove old stuff
	while (elapsed.toSec() > 3)
	{
		feedback.pop_front();
		acc.pop_front();
		elapsed = feedback.back().header.stamp - feedback.front().header.stamp;
	}

	mutex.unlock();
}


/**************************************************/
void newGoal(const control_msgs::GripperCommandActionGoalConstPtr &msg_)
{
	mutex.lock();
	feedback.clear();
	acc.clear();
	mutex.unlock();
}


/**************************************************/
int main(int argn_, char **argv_)
{
	ros::init(argn_, argv_, "gripper_monitor");
	ros::NodeHandle handler;

	// Load the node's configuration
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(GraspingUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + GraspingUtils::getConfigPath());

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


	/********** Spin the node **********/
	ros::AsyncSpinner spinner(3);
	spinner.start();
	ros::waitForShutdown();


	return EXIT_SUCCESS;
}
