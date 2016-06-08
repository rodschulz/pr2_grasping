/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include "Config.hpp"
#include "CloudUtils.hpp"
#include "Calculator.hpp"

#define CONFIG_LOCATION "src/grasping/config/config_dense_evaluation.yaml"

//static cv::Mat BoW;

void graspingCallback(const sensor_msgs::PointCloud2ConstPtr &_msg)
{
	ROS_INFO("New cloud received");

	// Convert and clean input cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*_msg, *cloudXYZ);

	// Prepare cloud
	ROS_INFO("...preparing cloud");
	CloudUtils::removeNANs(cloudXYZ);
	//... remove additional data

	// Estimate normals and generate an unique cloud
	ROS_INFO("...estimating normals");
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::Normal>::Ptr normals = CloudUtils::estimateNormals(cloudXYZ, -1);
	pcl::concatenateFields(*cloudXYZ, *normals, *cloud);

	// Descriptor dense evaluation over the point cloud
	ROS_INFO("..performing dense evaluation");
	cv::Mat descriptors;
	Calculator::calculateDescriptors(cloud, Config::getDescriptorParams(), descriptors);

	// Perform the cloud segmentation using the descriptors
}

int main(int _argn, char **_argv)
{
	// Initialize the node as "graping_node"
	ros::init(_argn, _argv, "grasping_node");

	// Set the node handler
	ros::NodeHandle nodeHandler;

	// Subscribe to topic "/head_mount_kinect/depth/points"
	ros::Subscriber sub = nodeHandler.subscribe("/head_mount_kinect/depth/points", 10, graspingCallback);

	// Load the node's configuration
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(CONFIG_LOCATION))
		throw std::runtime_error((std::string) "Error reading config at " + Utils::getWorkingDirectory() + CONFIG_LOCATION);

	// Keep looping
	ROS_INFO("Node looping");
	ros::spin();

	return EXIT_SUCCESS;
}
