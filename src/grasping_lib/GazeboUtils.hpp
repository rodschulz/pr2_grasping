/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <utility>
#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>
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
	static inline bool getModelState(const std::string &modelName_, gazebo_msgs::GetModelState::Response &state_)
	{
		gazebo_msgs::GetModelState state;
		state.request.model_name = modelName_;
		if (ros::service::call("/gazebo/get_model_state", state))
		{
			if (state.response.success)
				state_ = state.response;
			return state.response.success;
		}
		return false;
	}


	/**************************************************/
	static inline bool getWorldState(std::map<std::string, gazebo_msgs::GetModelStateResponse> &state_, const std::map<std::string, bool> &ignore_ = std::map<std::string, bool>())
	{
		bool result = false;

		std::vector<std::string> models;
		if (getSpawnedModels(models, ignore_))
		{
			result = true;

			state_.clear();
			for (std::vector<std::string>::const_iterator it = models.begin(); it != models.end(); it++)
			{
				gazebo_msgs::GetModelState::Response modelState;
				if (getModelState(*it, modelState))
					state_[*it] = modelState;
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
	static inline bool deleteModel(const std::string &modelName_)
	{
		gazebo_msgs::DeleteModel remove;
		remove.request.model_name = modelName_;
		if (ros::service::call("/gazebo/delete_model", remove))
			return remove.response.success;
		return false;
	}


	/**************************************************/
	static inline bool spawnModel(const std::string &modelName_, const std::string &modelXML_, const gazebo_msgs::GetModelState::Response &state_)
	{
		gazebo_msgs::SpawnModel spawn;
		spawn.request.model_name = modelName_;
		spawn.request.model_xml = modelXML_;
		spawn.request.initial_pose = state_.pose;

		if (ros::service::call("/gazebo/spawn_sdf_model", spawn))
			return spawn.response.success;

		return true;
	}


	// /**************************************************/
	// static inline bool respawnModel(const std::string &modelName_, const std::string &modelXML_, const gazebo_msgs::GetModelState::Response &state_)
	// {
	// 	if (deleteModel(modelName_))
	// 		if (spawnModel(modelName_, modelXML_, state_))
	// 			return true;
	// 	return false;
	// }

private:
	// Constructor
	GazeboUtils();
	// Destructor
	~GazeboUtils();
};
