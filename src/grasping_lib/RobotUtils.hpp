/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <utility>
#include <string>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

enum Effector
{
	RIGHT_ARM,
	LEFT_ARM
};


#define FRAME_KINNECT		"head_mount_kinect_ir_optical_frame"
#define FRAME_BASE			"base_footprint"


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

private:
	// Constructor
	RobotUtils();
	// Destructor
	~RobotUtils();
};
