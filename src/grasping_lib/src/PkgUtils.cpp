/**
 * Author: rodrigo
 * 2016
 */
#include "PkgUtils.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <ctime>
#include <boost/preprocessor/stringize.hpp>


#define CONFIG_LOCATION			PKG_CONFIG_DIR "/config.yaml"
#define TIMESTAMP_BUFFER_SIZE	100


std::string PkgUtils::getConfigPath()
{
	std::string packagePath = ros::package::getPath(PACKAGE_NAME);
	std::string fullpath = packagePath + "/" + CONFIG_LOCATION;
	return fullpath;
}

std::string PkgUtils::getOutputPath()
{
	std::string packagePath = ros::package::getPath(PACKAGE_NAME);
	std::string fullpath = packagePath + "/" + PKG_OUTPUT_DIR + "/";
	return fullpath;
}

std::string PkgUtils::getTimestamp(const std::string &format_)
{
	time_t rawtime;
	time (&rawtime);

	struct tm *timeinfo;
	timeinfo = localtime(&rawtime);

	char timestamp[TIMESTAMP_BUFFER_SIZE];
	strftime(timestamp, TIMESTAMP_BUFFER_SIZE, format_.c_str(), timeinfo);

	return std::string(timestamp);
}

std::string PkgUtils::toString(const moveit_msgs::MoveItErrorCodes &code_)
{
	switch (code_.val)
	{
	case moveit_msgs::MoveItErrorCodes::SUCCESS:
		return BOOST_STRINGIZE(SUCCESS);
	case moveit_msgs::MoveItErrorCodes::FAILURE:
		return BOOST_STRINGIZE(FAILURE);
	case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
		return BOOST_STRINGIZE(PLANNING_FAILED);
	case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
		return BOOST_STRINGIZE(INVALID_MOTION_PLAN);
	case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
		return BOOST_STRINGIZE(MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE);
	case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
		return BOOST_STRINGIZE(CONTROL_FAILED);
	case moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
		return BOOST_STRINGIZE(UNABLE_TO_AQUIRE_SENSOR_DATA);
	case moveit_msgs::MoveItErrorCodes::TIMED_OUT:
		return BOOST_STRINGIZE(TIMED_OUT);
	case moveit_msgs::MoveItErrorCodes::PREEMPTED:
		return BOOST_STRINGIZE(PREEMPTED);
	case moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION:
		return BOOST_STRINGIZE(START_STATE_IN_COLLISION);
	case moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
		return BOOST_STRINGIZE(START_STATE_VIOLATES_PATH_CONSTRAINTS);
	case moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION:
		return BOOST_STRINGIZE(GOAL_IN_COLLISION);
	case moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
		return BOOST_STRINGIZE(GOAL_VIOLATES_PATH_CONSTRAINTS);
	case moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
		return BOOST_STRINGIZE(GOAL_CONSTRAINTS_VIOLATED);
	case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME:
		return BOOST_STRINGIZE(INVALID_GROUP_NAME);
	case moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
		return BOOST_STRINGIZE(INVALID_GOAL_CONSTRAINTS);
	case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE:
		return BOOST_STRINGIZE(INVALID_ROBOT_STATE);
	case moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME:
		return BOOST_STRINGIZE(INVALID_LINK_NAME);
	case moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME:
		return BOOST_STRINGIZE(INVALID_OBJECT_NAME);
	case moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
		return BOOST_STRINGIZE(FRAME_TRANSFORM_FAILURE);
	case moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
		return BOOST_STRINGIZE(COLLISION_CHECKING_UNAVAILABLE);
	case moveit_msgs::MoveItErrorCodes::ROBOT_STATE_STALE:
		return BOOST_STRINGIZE(ROBOT_STATE_STALE);
	case moveit_msgs::MoveItErrorCodes::SENSOR_INFO_STALE:
		return BOOST_STRINGIZE(SENSOR_INFO_STALE);
	case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
		return BOOST_STRINGIZE(NO_IK_SOLUTION);
	default:
		return "";
	}
}

std::string PkgUtils::toString(const actionlib_msgs::GoalStatus &code_)
{
	static std::string str[] = {
		BOOST_STRINGIZE(PENDING),
		BOOST_STRINGIZE(ACTIVE),
		BOOST_STRINGIZE(PREEMPTED),
		BOOST_STRINGIZE(SUCCEEDED),
		BOOST_STRINGIZE(ABORTED),
		BOOST_STRINGIZE(REJECTED),
		BOOST_STRINGIZE(PREEMPTING),
		BOOST_STRINGIZE(RECALLING),
		BOOST_STRINGIZE(RECALLED),
		BOOST_STRINGIZE(LOST)
	};
	return str[code_.status];
}

std::string PkgUtils::toString(const control_msgs::FollowJointTrajectoryResult &code_)
{
	static std::string str[] = {
		BOOST_STRINGIZE(SUCCESSFUL),
		BOOST_STRINGIZE(INVALID_GOAL),
		BOOST_STRINGIZE(INVALID_JOINTS),
		BOOST_STRINGIZE(OLD_HEADER_TIMESTAMP),
		BOOST_STRINGIZE(PATH_TOLERANCE_VIOLATED),
		BOOST_STRINGIZE(GOAL_TOLERANCE_VIOLATED)
	};
	return str[-code_.error_code];
}
