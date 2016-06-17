/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/io/pcd_io.h>
#include <tf/transform_listener.h>
#include "Config.hpp"
#include "CloudUtils.hpp"
#include "Calculator.hpp"

#define CONFIG_LOCATION		"src/grasping/config/config_dense_evaluation.yaml"
#define FRAME_KINNECT		"head_mount_kinect_ir_optical_frame"
#define FRAME_BASE			"base_link"

//static cv::Mat BoW;
static tf::TransformListener *transformationListener;
bool done = false;

bool getTransformation(tf::StampedTransform &transform_, const std::string &target_, const std::string &source_, const ros::Time &time_ = ros::Time::now(), const ros::Duration &timeout_ = ros::Duration(1.0))
{
	try
	{
		transformationListener->waitForTransform(target_, source_, time_, timeout_);
		transformationListener->lookupTransform(target_, source_, ros::Time(0), transform_);
		return true;
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		return false;
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr basicObjectExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_)
{
	// Transform the points to kinect's ref system
	tf::StampedTransform transformation;
	while (!getTransformation(transformation, FRAME_KINNECT, FRAME_BASE)){}
	tf::Vector3 point = transformation * tf::Vector3(0, 0, 0.77);

	// Calculate the clipping plane
	tf::Vector3 localNormal = transformation.getBasis().getColumn(2);
	Eigen::Vector3f globalNormal = Eigen::Vector3f(localNormal.x(), localNormal.y(), localNormal.z());
	Eigen::Vector3f globalPoint = Eigen::Vector3f(point.x(), point.y(), point.z());

	ROS_INFO("normal: %f %f %f", globalNormal.x(), globalNormal.y(), globalNormal.z());
	ROS_INFO("point: %f %f %f", globalPoint.x(), globalPoint.y(), globalPoint.z());

	Eigen::Hyperplane<float, 3> clippingPlane = Eigen::Hyperplane<float, 3>(globalNormal, globalPoint);
	Eigen::Hyperplane<float, 3>::Coefficients planeCoeffs = clippingPlane.coeffs();

	if (!done)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlane = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
		for (float i = 0; i < 2; i += 0.1)
			for (float j = 0; j < 2; j += 0.1)
			cloudPlane->push_back(PointFactory::createPointXYZ(clippingPlane.projection(Eigen::Vector3f(i, j, 0))));

		pcl::io::savePCDFileASCII("./plane.pcd", *cloudPlane);
	}

	// Get the filtered point indices
	std::vector<int> clippedPoints, idxs;
	pcl::PlaneClipper3D<pcl::PointXYZ> filter = pcl::PlaneClipper3D<pcl::PointXYZ>(planeCoeffs);
	filter.clipPointCloud3D(*cloud_, clippedPoints, idxs);

	ROS_INFO("clipped size %zu - idxs size %zu", clippedPoints.size(), idxs.size());

	// Generate the filtered cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	for (size_t i = 0; i < clippedPoints.size(); i++)
		filteredCloud->push_back(cloud_->at(clippedPoints[i]));

	return filteredCloud;
}

void graspingCallback(const sensor_msgs::PointCloud2ConstPtr &_msg)
{
	ROS_INFO("Cloud received");

	// Convert cloud to PCL format
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*_msg, *cloudXYZ);

	// Prepare cloud
	ROS_INFO("...size rev %zu", cloudXYZ->size());
	CloudUtils::removeNANs(cloudXYZ);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = basicObjectExtraction(cloudXYZ);
	ROS_INFO("...size after %zu", filtered->size());

	if (!done)
	{
		pcl::io::savePCDFileASCII("./orig.pcd", *cloudXYZ);
		pcl::io::savePCDFileASCII("./filtered.pcd", *filtered);
		done = true;
	}

	// Estimate normals and generate an unique cloud
	ROS_INFO("...estimating normals");
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::Normal>::Ptr normals = CloudUtils::estimateNormals(filtered, Config::getNormalEstimationRadius());
	pcl::concatenateFields(*filtered, *normals, *cloud);

	// Descriptor dense evaluation over the point cloud
	ROS_INFO("...performing dense evaluation");
	cv::Mat descriptors;
	Calculator::calculateDescriptors(cloud, Config::getDescriptorParams(), descriptors);

	// Perform the cloud segmentation using the descriptors
}

int main(int _argn, char **_argv)
{
	ros::init(_argn, _argv, "grasping_node");
	ros::NodeHandle nodeHandler;
	transformationListener = new tf::TransformListener(ros::Duration(10.0));
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
