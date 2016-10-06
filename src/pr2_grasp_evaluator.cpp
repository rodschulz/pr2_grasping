/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pr2_grasping/GraspEvaluator.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/transforms.h>
// #include <pcl_ros/impl/transforms.hpp>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/common/common.h>

#include "Config.hpp"
#include "GraspingUtils.hpp"


/***** Global variables *****/
tf::TransformListener *tfListener;


/**************************************************/
bool evaluateGraspingResult(pr2_grasping::GraspEvaluator::Request  &request_,
							pr2_grasping::GraspEvaluator::Response &response_)
{
	return true;
}


/**************************************************/
int main(int argn_, char** argv_)
{
	// Setup node
	ros::init(argn_, argv_, "pr2_grasp_evaluator");
	ros::NodeHandle handler;
	tfListener = new tf::TransformListener(ros::Duration(10.0));

	// Load the node's configuration
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(GraspingUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + GraspingUtils::getConfigPath());


	// Set subscription
	std::string topicName = Config::get()["labeler"]["pointcloudTopic"].as<std::string>();
	ros::Subscriber subscriber = handler.subscribe<sensor_msgs::PointCloud2>(topicName, 1, boost::bind(cloudCallback, _1, voxelSize, clippingPlaneZ, debugEnabled, writeClouds));

	// Set service
	ros::ServiceServer evaluationService = handler.advertiseService("/pr2_grasping/grasp_evaluator", evaluateGraspingResult);


	// Start the service
	ROS_INFO("Starting grasping evaluation service");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
