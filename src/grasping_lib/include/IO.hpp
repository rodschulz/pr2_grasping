/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <string>
#include <ros/time.h>
#include <ros/duration.h>
#include <moveit_msgs/Grasp.h>
#include <pr2_grasping/DescriptorCalc.h>
#include <yaml-cpp/yaml.h>
#include <moveit/move_group_interface/move_group.h>


class IO
{
public:
	/**************************************************/
	static std::string nextExperimentId(const std::string &targetObject_);

	/**************************************************/
	static std::string getExperimentId();

	/**************************************************/
	static void saveResults(const std::string &targetObject_,
							const bool attemptCompleted_,
							const bool attemptSuccessful_,
							const int clusterLabel_,
							const float gripperAngle_,
							const moveit_msgs::Grasp &grasp_,
							const int gripperAngleSplitNum_,
							const float gripperAngleStep_,
							const moveit::planning_interface::MoveItErrorCode &errCode_,
							const pr2_grasping::DescriptorCalc::Response &descriptor_);
private:
	IO();
	~IO();
};


/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const ros::Time &obj_)
{
	out << YAML::BeginMap
		<< YAML::Key << "secs" << YAML::Value << obj_.sec
		<< YAML::Key << "nsecs" << YAML::Value << obj_.nsec
		<< YAML::EndMap;
	return out;
}

/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const ros::Duration &obj_)
{
	out << YAML::BeginMap
		<< YAML::Key << "secs" << YAML::Value << obj_.sec
		<< YAML::Key << "nsecs" << YAML::Value << obj_.nsec
		<< YAML::EndMap;
	return out;
}

/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Point &msg_)
{
	out << YAML::BeginMap
		<< YAML::Key << "x" << YAML::Value << msg_.x
		<< YAML::Key << "y" << YAML::Value << msg_.y
		<< YAML::Key << "z" << YAML::Value << msg_.z
		<< YAML::EndMap;
	return out;
}

/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Quaternion &msg_)
{
	out << YAML::BeginMap
		<< YAML::Key << "x" << YAML::Value << msg_.x
		<< YAML::Key << "y" << YAML::Value << msg_.y
		<< YAML::Key << "z" << YAML::Value << msg_.z
		<< YAML::Key << "w" << YAML::Value << msg_.w
		<< YAML::EndMap;
	return out;
}

/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Vector3 &msg_)
{
	out << YAML::BeginMap
		<< YAML::Key << "x" << YAML::Value << msg_.x
		<< YAML::Key << "y" << YAML::Value << msg_.y
		<< YAML::Key << "z" << YAML::Value << msg_.z
		<< YAML::EndMap;
	return out;
}

/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const std_msgs::Header &msg_)
{
	out << YAML::BeginMap
		<< YAML::Key << "seq" << YAML::Value << msg_.seq
		<< YAML::Key << "stamp" << YAML::Value << msg_.stamp
		<< YAML::Key << "frame_id" << YAML::Value << msg_.frame_id
		<< YAML::EndMap;
	return out;
}

/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const trajectory_msgs::JointTrajectoryPoint &msg_)
{
	out << YAML::BeginMap
		<< YAML::Key << "positions" << YAML::Value << msg_.positions
		<< YAML::Key << "velocities" << YAML::Value << msg_.velocities
		<< YAML::Key << "accelerations" << YAML::Value << msg_.accelerations
		<< YAML::Key << "effort" << YAML::Value << msg_.effort
		<< YAML::Key << "time_from_start" << YAML::Value << msg_.time_from_start
		<< YAML::EndMap;
	return out;
}

/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const trajectory_msgs::JointTrajectory &msg_)
{
	out << YAML::BeginMap
		<< YAML::Key << "header" << YAML::Value << msg_.header
		<< YAML::Key << "joint_names" << YAML::Value << msg_.joint_names
		<< YAML::Key << "points" << YAML::Value;

	out << YAML::BeginSeq << YAML::Flow;
	for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = msg_.points.begin(); it != msg_.points.end(); it++)
		out << *it;
	out << YAML::EndSeq;

	out << YAML::EndMap;
	return out;
}

/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Pose &msg_)
{
	out << YAML::BeginMap
		<< YAML::Key << "position" << YAML::Value << msg_.position
		<< YAML::Key << "orientation" << YAML::Value << msg_.orientation
		<< YAML::EndMap;
	return out;
}

/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::PoseStamped &msg_)
{
	out << YAML::BeginMap
		<< YAML::Key << "header" << YAML::Value << msg_.header
		<< YAML::Key << "pose" << YAML::Value << msg_.pose
		<< YAML::EndMap;
	return out;
}

/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Vector3Stamped &msg_)
{
	out << YAML::BeginMap
		<< YAML::Key << "header" << YAML::Value << msg_.header
		<< YAML::Key << "vector" << YAML::Value << msg_.vector
		<< YAML::EndMap;
	return out;
}

/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const moveit_msgs::GripperTranslation &msg_)
{
	out << YAML::BeginMap
		<< YAML::Key << "direction" << YAML::Value << msg_.direction
		<< YAML::Key << "desired_distance" << YAML::Value << msg_.desired_distance
		<< YAML::Key << "min_distance" << YAML::Value << msg_.min_distance
		<< YAML::EndMap;
	return out;
}

/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const moveit_msgs::Grasp &msg_)
{
	out << YAML::BeginMap
		<< YAML::Key << "id" << YAML::Value << msg_.id
		<< YAML::Key << "pre_grasp_posture" << YAML::Value << msg_.pre_grasp_posture
		<< YAML::Key << "grasp_posture" << YAML::Value << msg_.grasp_posture
		<< YAML::Key << "grasp_pose" << YAML::Value << msg_.grasp_pose
		<< YAML::Key << "grasp_quality" << YAML::Value << msg_.grasp_quality
		<< YAML::Key << "pre_grasp_approach" << YAML::Value << msg_.pre_grasp_approach
		<< YAML::Key << "post_grasp_retreat" << YAML::Value << msg_.post_grasp_retreat
		<< YAML::Key << "post_place_retreat" << YAML::Value << msg_.post_place_retreat
		<< YAML::Key << "max_contact_force" << YAML::Value << msg_.max_contact_force
		<< YAML::Key << "allowed_touch_objects" << YAML::Value << msg_.allowed_touch_objects
		<< YAML::EndMap;
	return out;
}


/**************************************************/
YAML::Emitter& operator << (YAML::Emitter& out, const pr2_grasping::DescriptorCalc::Response &msg_)
{
	std::vector<double> desc;
	desc.resize(msg_.descriptor.size());
	for (size_t i = 0; i < msg_.descriptor.size(); i++)
		desc[i] = msg_.descriptor[i].data;

	std::string stat = msg_.params.sequenceStat == pr2_grasping::DescriptorParams::STAT_MEAN ? "mean" : "median";

	out << YAML::BeginMap
		<< YAML::Key << "point_index" << YAML::Value << msg_.index.data
		<< YAML::Key << "data" << YAML::Value << desc
		<< YAML::Key << "params"
		<< YAML::BeginMap
		<< YAML::Key << "patchSize" << YAML::Value << msg_.params.patchSize
		<< YAML::Key << "bandNumber" << YAML::Value << msg_.params.bandNumber
		<< YAML::Key << "bandWidth" << YAML::Value << msg_.params.bandWidth
		<< YAML::Key << "bidirectional" << YAML::Value << (msg_.params.bidirectional == true)
		<< YAML::Key << "useProjection" << YAML::Value << (msg_.params.useProjection == true)
		<< YAML::Key << "sequenceBin" << YAML::Value << msg_.params.sequenceBin
		<< YAML::Key << "sequenceStat" << YAML::Value << stat
		<< YAML::EndMap
		<< YAML::EndMap;
	return out;
}