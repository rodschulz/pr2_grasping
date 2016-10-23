/**
 * Author: rodrigo
 * 2016
 */
#include "RobotUtils.hpp"
#include <ros/ros.h>

std::pair<std::string, std::string> RobotUtils::getEffectorNames(const Effector &arm_)
{
	switch (arm_)
	{
	default:
		ROS_WARN("Wrong effector type, assuming right arm");

	case RIGHT_ARM:
		return std::pair<std::string, std::string>("right_arm", "right_eef");

	case LEFT_ARM:
		return std::pair<std::string, std::string>("left_arm", "left_eef");
	}
}


std::pair<std::string, std::string> RobotUtils::getEffectorNames(const std::string &arm_)
{
	if (boost::iequals(arm_, "right") || boost::iequals(arm_, "right_arm"))
		return getEffectorNames(RIGHT_ARM);

	else if (boost::iequals(arm_, "left") || boost::iequals(arm_, "left_arm"))
		return getEffectorNames(LEFT_ARM);

	else
	{
		ROS_WARN("Wrong effector type, assuming right arm");
		return getEffectorNames(RIGHT_ARM);
	}
}


std::string getEffectorFrame(const std::string &arm_)
{
	if (boost::iequals(arm_, "right") || boost::iequals(arm_, "right_arm"))
		return FRAME_R_GRIPPER;

	else if (boost::iequals(arm_, "left") || boost::iequals(arm_, "left_arm"))
		return FRAME_L_GRIPPER;

	else
	{
		ROS_WARN("Wrong effector type, assuming right arm");
		return FRAME_R_GRIPPER;
	}
}


std::string RobotUtils::getGripperTopic(const std::string &arm_)
{
	if (boost::iequals(arm_, "right") || boost::iequals(arm_, "right_arm"))
		return "r_gripper_controller/gripper_action";

	else if (boost::iequals(arm_, "left") || boost::iequals(arm_, "left_arm"))
		return "l_gripper_controller/gripper_action";

	else
	{
		ROS_WARN("Wrong effector type, assuming right arm");
		return "r_gripper_controller/gripper_action";
	}
}


std::string RobotUtils::getArmTopic(const std::string &arm_)
{
	if (boost::iequals(arm_, "right") || boost::iequals(arm_, "right_arm"))
		return "/r_arm_controller/follow_joint_trajectory";

	else if (boost::iequals(arm_, "left") || boost::iequals(arm_, "left_arm"))
		return "/l_arm_controller/follow_joint_trajectory";

	else
	{
		ROS_WARN("Wrong effector type, assuming right arm");
		return "/r_arm_controller/follow_joint_trajectory";
	}
}


bool RobotUtils::planAndMove(MoveGroupPtr &group_,
							 const int maxRetries_)
{
	int retries = 0;
	moveit::planning_interface::MoveGroup::Plan plan;

	bool planOk = group_->plan(plan);
	while (!planOk)
	{
		if (maxRetries_ != -1 && ++retries > maxRetries_)
		{
			ROS_WARN(".....too many planning retries, aborting");
			break;
		}

		ROS_INFO(".....planning failed, retrying");
		ros::Duration(0.5).sleep();
		planOk = group_->plan(plan);
	}

	// Perform the movement
	if (planOk)
		return group_->move();
	else
		return false;
}


bool RobotUtils::move(MoveGroupPtr &group_,
					  const int maxRetries_)
{
	int retries = 0;
	while (!group_->move())
	{
		if (maxRetries_ != -1 && ++retries > maxRetries_)
		{
			ROS_WARN(".....too many planning retries, aborting");
			return false;
		}

		if (maxRetries_ == -1)
			ROS_INFO(".....planning failed, retrying (no limit of attempts)");
		else
			ROS_INFO(".....planning failed, retrying (%d out of %d attempts)", retries, maxRetries_);
		ros::Duration(0.5).sleep();
	}

	return true;
}

void RobotUtils::moveHead(const float x_,
						  const float y_,
						  const float z_)
{
	// define action client
	HeadClient *headClient = new HeadClient("/head_traj_controller/point_head_action", true);

	// wait for the action server to come up
	while (!headClient->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for the point_head_action server to come up");

	ROS_INFO("Moving head");

	// the target point, expressed in the given frame
	geometry_msgs::PointStamped targetPoint;
	targetPoint.header.frame_id = FRAME_BASE;
	targetPoint.point.x = x_;
	targetPoint.point.y = y_;
	targetPoint.point.z = z_;

	// make the kinect x axis point at the desired position
	control_msgs::PointHeadGoal goal;
	goal.target = targetPoint;
	goal.pointing_frame = "head_mount_kinect_rgb_link";
	goal.pointing_axis.x = 1;
	goal.pointing_axis.y = 0;
	goal.pointing_axis.z = 0;

	ROS_INFO("...sending head goal");
	headClient->sendGoal(goal);
	headClient->waitForResult(ros::Duration(60));
}

float RobotUtils::getPR2GripperJointOpening(const float gap_)
{
	float oppening;

	if (gap_ > GRIPPER_OPEN)
		oppening = GRIPPER_OPEN / GAP_CONVERSION_RATIO;

	else if (gap_ < GRIPPER_CLOSED)
		oppening = GRIPPER_CLOSED;

	else
		oppening = gap_ / GAP_CONVERSION_RATIO;

	return oppening;
}

actionlib::SimpleClientGoalState RobotUtils::moveGripper(const std::string arm_,
		const float position_,
		const float maxEffort_)
{
	GripperClient *client = new GripperClient(RobotUtils::getGripperTopic(arm_), true);
	while (!client->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for gripper action server");

	control_msgs::GripperCommandGoal cmd;
	cmd.command.position =  position_;
	cmd.command.max_effort = maxEffort_;

	client->cancelAllGoals();
	ros::Duration(1.0).sleep();

	ROS_INFO("Sending gripper goal");
	return client->sendGoalAndWait(cmd);
}
