/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <utility>
#include <string>
#include <fstream>
#include <streambuf>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>


// Class implementing utilities for gazebo simulation interaction
class GazeboUtils
{
public:
	/**************************************************/
	static bool getSpawnedModels(std::vector<std::string> &models_,
								 const std::map<std::string, bool> &ignore_ = std::map<std::string, bool>());

	/**************************************************/
	static bool getModelState(const std::string &modelName_,
							  geometry_msgs::Pose &state_);

	/**************************************************/
	static bool getWorldState(std::map<std::string, geometry_msgs::Pose> &stateMap_,
							  const std::map<std::string, bool> &ignore_ = std::map<std::string, bool>());

	/**************************************************/
	static bool setModelState(const std::string &modelName_,
							  const geometry_msgs::Pose &pose_,
							  const std::string &referenceFrame_);

	/**************************************************/
	static bool deleteModel(const std::string &modelName_);

	/**************************************************/
	static bool spawnModel(const std::string &modelName_,
						   const std::string &modelFileLocation_,
						   const geometry_msgs::Pose &state_);

private:
	// Constructor
	GazeboUtils();
	// Destructor
	~GazeboUtils();


	/**************************************************/
	static std::string loadModelFile(const std::string &modelLocation_)
	{
		std::ifstream modelFile(modelLocation_.c_str(), std::ifstream::in);
		std::string str;
		str.assign((std::istreambuf_iterator<char>(modelFile)), std::istreambuf_iterator<char>());

		return str;
	}
};
