/**
 * Author: rodrigo
 * 2016
 */
#include "IO.hpp"
#include "PkgUtils.hpp"
#include <iostream>
#include <fstream>


// Variable tracking the id of current experiment
static std::string experimentId = "unknown";


std::string IO::getExperimentId()
{
	return experimentId;
}

std::string IO::nextExperimentId(const std::string &targetObject_)
{
	experimentId = targetObject_ + "_" + PkgUtils::getTimestamp("%Y-%m-%d_%H%M%S");
	return experimentId;
}

void IO::saveResults(const std::string &targetObject_,
					 const bool attemptCompleted_,
					 const bool attemptSuccessful_,
					 const int clusterLabel_,
					 const float gripperAngle_,
					 const int gripperAngleSplitNum_,
					 const float gripperAngleStep_,
					 const moveit_msgs::Grasp &grasp_,
					 const moveit::planning_interface::MoveItErrorCode &errCode_)
{
	std::string filename = PkgUtils::getOutputPath() + IO::getExperimentId() + ".yaml";

	ROS_DEBUG_STREAM("Saving to: " << filename);

	// generate a YAML file with the results
	YAML::Emitter emitter;
	emitter << YAML::BeginMap
			<< YAML::Key << "target_object" << YAML::Value <<  targetObject_
			<< YAML::Key << "cluster_label" << YAML::Value << clusterLabel_

			<< YAML::Key << "result"
			<< YAML::BeginMap
			<< YAML::Key << "attempt_completed" << YAML::Value << attemptCompleted_
			<< YAML::Key << "success" << YAML::Value << attemptSuccessful_
			<< YAML::Key << "pick_error_code" << YAML::Value << errCode_.val
			<< YAML::EndMap

			<< YAML::Key << "orientation"
			<< YAML::BeginMap
			<< YAML::Key << "angle" << YAML::Value << gripperAngle_
			<< YAML::Key << "split_number" << YAML::Value << gripperAngleSplitNum_
			<< YAML::Key << "angle_step" << YAML::Value << gripperAngleStep_
			<< YAML::EndMap

			<< YAML::Key << "grasp" << YAML::Value << grasp_
			<< YAML::EndMap;

	std::ofstream output;
	output.open(filename.c_str(), std::fstream::out);
	output << emitter.c_str();
	output.close();

	ROS_INFO("...saved");
}