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
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>


// Class implementing utilities for gazebo simulation interaction
class GazeboUtils
{
public:
	/**************************************************/
	static inline bool getSpawnedModels(std::vector<std::string> &models_, const std::map<std::string, bool> &ignore_ = std::map<std::string, bool>())
	{
		models_.clear();

		gazebo_msgs::GetWorldProperties props;
		if (ros::service::call("/gazebo/get_world_properties", props))
		{
			for (std::vector<std::string>::const_iterator it = props.response.model_names.begin(); it != props.response.model_names.end(); it++)
				if (ignore_.find(*it) == ignore_.end())
					models_.push_back(*it);
			return props.response.success;
		}
		return false;
	}


	/**************************************************/
	static inline bool getModelState(const std::string &modelName_, geometry_msgs::Pose &state_)
	{
		gazebo_msgs::GetModelState state;
		state.request.model_name = modelName_;
		if (ros::service::call("/gazebo/get_model_state", state))
		{
			if (state.response.success)
				state_ = state.response.pose;
			return state.response.success;
		}
		return false;
	}


	/**************************************************/
	static inline bool getWorldState(std::map<std::string, geometry_msgs::Pose> &stateMap_, const std::map<std::string, bool> &ignore_ = std::map<std::string, bool>())
	{
		bool result = false;

		std::vector<std::string> models;
		if (getSpawnedModels(models, ignore_))
		{
			result = true;

			stateMap_.clear();
			for (std::vector<std::string>::const_iterator it = models.begin(); it != models.end(); it++)
			{
				geometry_msgs::Pose modelState;
				if (getModelState(*it, modelState))
					stateMap_[*it] = modelState;
				else
				{
					ROS_WARN("Cant get model state: %s", it->c_str());
					result = false;
				}
			}
		}
		else
			ROS_WARN("Can't get spawned models");

		return result;
	}


	/**************************************************/
	static inline bool setModelState(const std::string &modelName_, const geometry_msgs::Pose &pose_, const std::string &referenceFrame_)
	{
		gazebo_msgs::SetModelState state;
		state.request.model_state.model_name = modelName_;
		state.request.model_state.pose = pose_;
		state.request.model_state.reference_frame = referenceFrame_;
		if (ros::service::call("/gazebo/set_model_state", state))
			return state.response.success;
		return false;
	}


	/**************************************************/
	static inline bool deleteModel(const std::string &modelName_)
	{
		gazebo_msgs::DeleteModel remove;
		remove.request.model_name = modelName_;
		if (ros::service::call("/gazebo/delete_model", remove))
			return remove.response.success;
		return false;
	}


	/**************************************************/
	static inline bool spawnModel(const std::string &modelName_, const std::string &modelFileLocation_, const geometry_msgs::Pose &state_)
	{
		gazebo_msgs::SpawnModel spawn;
		spawn.request.model_name = modelName_;
		spawn.request.model_xml = loadModelFile(modelFileLocation_);
		spawn.request.robot_namespace = "";
		spawn.request.initial_pose = state_;
		spawn.request.reference_frame = "world";

		if (ros::service::call("/gazebo/spawn_sdf_model", spawn))
		{
			ROS_INFO_STREAM(spawn.response.status_message);
			return spawn.response.success;
		}
		return false;
	}


private:
	// Constructor
	GazeboUtils();
	// Destructor
	~GazeboUtils();


	/**************************************************/
	static inline std::string loadModelFile(const std::string &modelLocation_)
	{
		std::ifstream modelFile(modelLocation_.c_str(), std::ifstream::in);
		std::string str;
		str.assign((std::istreambuf_iterator<char>(modelFile)), std::istreambuf_iterator<char>());

		return str;
	}
};
