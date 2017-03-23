/**
 * Author: rodrigo
 * 2016
 */
#include "IO.hpp"
#include "PkgUtils.hpp"
#include <iostream>
#include <fstream>


// Variable tracking the id of current experiment
std::string experimentId = "unknown";


std::string IO::getExperimentId()
{
	return experimentId;
}

std::string IO::nextExperimentId(const std::string &targetObject_)
{
	experimentId = targetObject_ + "_" + PkgUtils::getTimestamp("%Y-%m-%d_%H%M%S");
	return experimentId;
}

void IO::saveResults(const std::string &object_,
					 const bool completed_,
					 const bool successful_,
					 const GraspData &gdata_,
					 const moveit::planning_interface::MoveItErrorCode &errCode_,
					 const bool predicted_,
					 const std::string classifier_,
					 const int nsplits_,
					 const float angleStep_)
{
	std::string filename = PkgUtils::getOutputPath() + IO::getExperimentId() + ".yaml";

	ROS_DEBUG_STREAM("Saving to: " << filename);

	std::vector<double> desc;
	desc.resize(gdata_.descriptor.descriptor.size());
	for (size_t i = 0; i < gdata_.descriptor.descriptor.size(); i++)
		desc[i] = gdata_.descriptor.descriptor[i].data;

	// generate a YAML file with the results
	YAML::Emitter emitter;
	emitter << YAML::BeginMap
			<< YAML::Key << "target_object" << YAML::Value <<  object_
			<< YAML::Key << "cluster_label" << YAML::Value << gdata_.label

			<< YAML::Key << "prediction"
			<< YAML::BeginMap
			<< YAML::Key << "used" << YAML::Value << predicted_
			<< YAML::Key << "score" << YAML::Value << gdata_.score
			<< YAML::Key << "classifier" << YAML::Value << classifier_
			<< YAML::EndMap

			<< YAML::Key << "result"
			<< YAML::BeginMap
			<< YAML::Key << "attempt_completed" << YAML::Value << completed_
			<< YAML::Key << "success" << YAML::Value << successful_
			<< YAML::Key << "pick_error_code" << YAML::Value << errCode_.val
			<< YAML::EndMap

			<< YAML::Key << "descriptor" << YAML::Value << gdata_.descriptor

			<< YAML::Key << "orientation"
			<< YAML::BeginMap
			<< YAML::Key << "angle" << YAML::Value << gdata_.angle
			<< YAML::Key << "split_number" << YAML::Value << nsplits_
			<< YAML::Key << "angle_step" << YAML::Value << angleStep_
			<< YAML::EndMap

			<< YAML::Key << "grasp" << YAML::Value << gdata_.grasp
			<< YAML::EndMap;

	std::ofstream output;
	output.open(filename.c_str(), std::fstream::out);
	output << emitter.c_str();
	output.close();

	ROS_INFO("...saved");
}