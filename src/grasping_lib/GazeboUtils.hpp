/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <utility>
#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelStateResponse.h>
// #include <gazebo_msgs/DeleteModel.h>
// #include <gazebo_msgs/SpawnModel.h>


// Class implementing utilities for gazebo simulation interaction
class GazeboUtils
{
public:
	/**************************************************/
	static bool getWorldObjects(std::vector<std::string> &objects_, const std::map<std::string, bool> &ignore_ = std::map<std::string, bool>())
	{
		objects_.clear();

		gazebo_msgs::GetWorldProperties props;
		if (ros::service::call("/gazebo/get_world_properties", props))
		{
			for (std::vector<std::string>::const_iterator it = props.response.model_names.begin(); it != props.response.model_names.end(); it++)
				if (ignore_.find(*it) == ignore_.end())
					objects_.push_back(*it);

			return props.response.success;
		}

		return false;
	}


	/**************************************************/
	static bool getWorldState(std::map<std::string, gazebo_msgs::GetModelStateResponse> &state_)
	{
		return true;
	}


	/**************************************************/
	static bool deleteObject()
	{
		return true;
	}


	/**************************************************/
	static bool spawnObject()
	{
		return true;
	}

private:
	// Constructor
	GazeboUtils();
	// Destructor
	~GazeboUtils();
};
