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
	static inline std::pair<std::string, std::string> getEffectorNames(const Effector &arm_)
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
	static inline std::pair<std::string, std::string> getEffectorNames(const std::string &arm_)
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
	static inline std::string getGripperTopic(const std::string &arm_)
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
	static inline bool planAndMove(MoveGroupPtr &group_,
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
	static inline bool move(MoveGroupPtr &group_,
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

			ROS_INFO(".....planning failed, retrying");
			ros::Duration(0.5).sleep();
		}

		return true;
	}


private:
	// Constructor
	RobotUtils();
	// Destructor
	~RobotUtils();
};
