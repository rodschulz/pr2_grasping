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


enum Effector
{
	RIGHT_ARM,
	LEFT_ARM
};


#define FRAME_KINNECT		"head_mount_kinect_ir_optical_frame"
#define FRAME_BASE			"base_footprint"


typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;


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
	static inline bool moveGroup(MoveGroupPtr &group_, const geometry_msgs::PoseStamped &pose_, const int maxRetries_ = -1)
	{
		int retries = 0;

		// Plan the movement
		moveit::planning_interface::MoveGroup::Plan plan;
		bool planOk = group_->plan(plan);
		while (!planOk)
		{
			ROS_INFO(".....planning failed, retrying");
			ros::Duration(0.5).sleep();
			group_->setPoseTarget(group_->getRandomPose());
			group_->move();

			ros::Duration(0.5).sleep();
			group_->setPoseTarget(pose_);

			if (maxRetries_ != -1 && ++retries > maxRetries_)
			{
				ROS_WARN(".....too many planning retries, aborting");
				break;
			}
		}

		// Perform the movement
		bool moveOk = false;
		if (planOk)
		{
			moveOk = group_->move();
			ros::Duration(1).sleep();
		}

		return moveOk && planOk;
	}


private:
	// Constructor
	RobotUtils();
	// Destructor
	~RobotUtils();
};
