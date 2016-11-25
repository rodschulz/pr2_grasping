/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <string>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <actionlib_msgs/GoalStatus.h>
#include <control_msgs/FollowJointTrajectoryResult.h>


#define PACKAGE_NAME		"pr2_grasping"
#define PKG_CONFIG_DIR		"config"
#define PKG_OUTPUT_DIR		"output"


class PkgUtils
{
public:
	/**************************************************/
	static std::string getConfigPath();

	/**************************************************/
	static std::string getOutputPath();

	/**************************************************/
	static std::string getTimestamp(const std::string &format_);

	/**************************************************/
	static std::string toString(const moveit_msgs::MoveItErrorCodes &code_);

	/**************************************************/
	static std::string toString(const actionlib_msgs::GoalStatus &code_);

	/**************************************************/
	static std::string toString(const control_msgs::FollowJointTrajectoryResult &code_);

private:
	// Constructor
	PkgUtils();
	// Destructor
	~PkgUtils();
};
