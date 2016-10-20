/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pr2_controllers_msgs/Pr2GripperCommandActionFeedback.h>
#include <pr2_controllers_msgs/Pr2GripperCommandActionGoal.h>
#include <pr2_grasping/GraspingGroup.h>
#include <boost/algorithm/string.hpp>
#include "RobotUtils.hpp"


/**************************************************/
void feedbackReceived(const pr2_controllers_msgs::Pr2GripperCommandActionFeedbackConstPtr &msg_)
{
}


/**************************************************/
void newGoal(const pr2_controllers_msgs::Pr2GripperCommandActionGoalConstPtr &msg_)
{
}


/**************************************************/
int main(int argn_, char **argv_)
{
	ros::init(argn_, argv_, "pr2_cloud_labeler");
	ros::NodeHandle handler;


	/********** Retrieve the group used for grasping **********/
	ROS_INFO("Retrieving grasping group");
	pr2_grasping::GraspingGroup srv;
	srv.response.result = false;
	while(!srv.response.result)
	{
		if (ros::service::call("/pr2_grasping/effector_name", srv))
			ROS_INFO("Queried grasping group %s", srv.response.result ? "SUCCESSFUL" : "FAILED");
		ros::Duration(1).sleep();
	}
	ROS_INFO("Using grasping group %s", srv.response.groupName.c_str());


	//r_gripper_controller/gripper_action/cancel		actionlib_msgs/GoalID
	//r_gripper_controller/gripper_action/feedback		pr2_controllers_msgs/Pr2GripperCommandActionFeedback
	//r_gripper_controller/gripper_action/goal			pr2_controllers_msgs/Pr2GripperCommandActionResult
	//r_gripper_controller/gripper_action/result		pr2_controllers_msgs/Pr2GripperCommandActionGoal
	//r_gripper_controller/gripper_action/status		actionlib_msgs/GoalStatusArray

	/********** Set subscriptions/publishers **********/
	std::string baseTopic = RobotUtils::getGripperTopic(srv.response.groupName);
	ros::Subscriber feedbackSubscriber = handler.subscribe<pr2_controllers_msgs::Pr2GripperCommandActionFeedback>(baseTopic + "/feedback", 1, feedbackReceived);
	ros::Subscriber goalSubscriber = handler.subscribe<pr2_controllers_msgs::Pr2GripperCommandActionGoal>(baseTopic + "/goal", 1, newGoal);


	/********** Spin the node **********/
	ros::AsyncSpinner spinner(3);
	spinner.start();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
