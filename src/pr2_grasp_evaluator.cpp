/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <pr2_grasping/GazeboSetup.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "Config.hpp"
#include "GraspingUtils.hpp"


/**************************************************/
bool evaluateGraspingResult(pr2_grasping::GazeboSetup::Request  &request_,
			  pr2_grasping::GazeboSetup::Response &response_)
{
	return true;
}


/**************************************************/
int main(int argn_, char** argv_)
{
	// Setup node
	ros::init(argn_, argv_, "pr2_grasp_evaluator");
	ros::NodeHandle handler;

	// Load the node's configuration
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(GraspingUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + GraspingUtils::getConfigPath());

	// Service for result evaluation
	ros::ServiceServer evaluationService = handler.advertiseService("/pr2_grasping/result_evaluator", evaluateGraspingResult);

	// Start the service
	ROS_INFO("Starting grasping evaluation service");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
