/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pr2_grasping/GraspEvaluator.h>
// #include <moveit/move_group_interface/move_group.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/transforms.h>
// #include <pcl_ros/impl/transforms.hpp>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/common/common.h>

#include <boost/signals2/mutex.hpp>
#include "Config.hpp"
#include "GraspingUtils.hpp"
#include "RobotUtils.hpp"


/***** Global variables *****/
boost::mutex mutex;
bool evaluationScheduled = false;
tf::TransformListener *tfListener;


/**************************************************/
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg_)
{
	if (evaluationScheduled)
	{
	}
}


/**************************************************/
bool evaluateGraspingResult(pr2_grasping::GraspEvaluator::Request  &request_,
							pr2_grasping::GraspEvaluator::Response &response_)
{
	mutex.lock();
	evaluationScheduled = true;
	mutex.unlock();

	// Prepare the planning framework
	moveit::planning_interface::MoveGroup::Plan plan;
	std::pair<std::string, std::string> effectorNames = RobotUtils::getEffectorNames(request_.effectorName);
	MoveGroupPtr effector = MoveGroupPtr(new moveit::planning_interface::MoveGroup(effectorNames.first));
	effector->setEndEffector(effectorNames.second);
	effector->setPlannerId("RRTConnectkConfigDefault");


	//
	geometry_msgs::PoseStamped pose1;
	pose1.header.frame_id = "r_wrist_roll_link";
	pose1.pose = GraspingUtils::genPose(0, 0, 0, DEG2RAD(45), 1, 0, 0);


	response_.result = true;
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
	ros::Subscriber subscriber = handler.subscribe<sensor_msgs::PointCloud2>("/move_group/filtered_cloud", 1, cloudCallback);

	// Set service
	ros::ServiceServer evaluationService = handler.advertiseService("/pr2_grasping/grasp_evaluator", evaluateGraspingResult);


	// Start the service
	ROS_INFO("Starting grasping evaluation service");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
