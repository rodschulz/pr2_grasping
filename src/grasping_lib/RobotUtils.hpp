/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <utility>
#include <string>
#include <ros/ros.h>

enum Effector
{
	RIGHT_ARM,
	LEFT_ARM
};


#define FRAME_KINNECT		"head_mount_kinect_ir_optical_frame"
#define FRAME_BASE			"base_link"


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

private:
	// Constructor
	RobotUtils();
	// Destructor
	~RobotUtils();
};
