/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <utility>
#include <string>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/SingleJointPositionAction.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/GripperCommandAction.h>

enum Effector
{
	RIGHT_ARM,
	LEFT_ARM
};


#define FRAME_KINNECT		"head_mount_kinect_ir_optical_frame"
#define FRAME_BASE			"base_footprint"


typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;
/***** Client types definitions *****/
typedef actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> HeadClient;
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;


// Class implementing utilities for robot interaction
class RobotUtils
{
public:
	/**************************************************/
	static std::pair<std::string, std::string> getEffectorNames(const Effector &arm_)
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


	/**************************************************/
	static std::pair<std::string, std::string> getEffectorNames(const std::string &arm_)
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


	/**************************************************/
	static std::string getGripperTopic(const std::string &arm_)
	{
		if (boost::iequals(arm_, "right") || boost::iequals(arm_, "right_arm"))
			return "r_gripper_controller/gripper_action";

		else if (boost::iequals(arm_, "left") || boost::iequals(arm_, "left_arm"))
			return "l_gripper_controller/gripper_action";

		else
		{
			ROS_WARN("Wrong effector type, assuming right arm");
			return "";
		}
	}


	/**************************************************/
	static bool planAndMove(MoveGroupPtr &group_,
							const int maxRetries_ = -1)
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


	/**************************************************/
	static bool move(MoveGroupPtr &group_,
					 const geometry_msgs::PoseStamped &pose_,
					 const int maxRetries_ = -1)
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


	/**************************************************/
	static void moveHead(const float x_, const float y_, const float z_)
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

		// displacement limits (at least 1 sec and no faster than 1 rad/s)
		// goal.min_duration = ros::Duration(1);
		// goal.max_velocity = 1.0;

		ROS_INFO("...sending head goal");
		headClient->sendGoal(goal);
		headClient->waitForResult(ros::Duration(60));

		// ROS_INFO("...head moved");
	}

private:
	// Constructor
	RobotUtils();
	// Destructor
	~RobotUtils();
};
