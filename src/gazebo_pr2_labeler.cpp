/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include "Config.hpp"
#include "Loader.hpp"
#include "CloudUtils.hpp"
#include "Calculator.hpp"
#include "ClusteringUtils.hpp"
#include "GraspingUtils.hpp"
#include "Writer.hpp"

#define CONFIG_LOCATION		"src/grasping/config/config_dense_evaluation.yaml"
#define FRAME_KINNECT		"head_mount_kinect_ir_optical_frame"
#define FRAME_BASE			"base_link"

tf::TransformListener *transformationListener;
CvSVMPtr svm;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &_msg)
{
	ROS_INFO("Cloud received");

	static bool debugEnabled = Config::get()["nodeDebug"].as<bool>();
	static float voxelSize = Config::get()["voxelSize"].as<float>();

	// Convert cloud to PCL format
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*_msg, *cloudXYZ);

	// Get the required transformation
	tf::StampedTransform transformation;
	while (!GraspingUtils::getTransformation(transformation, transformationListener, FRAME_KINNECT, FRAME_BASE))
		;

	// Prepare cloud
	CloudUtils::removeNANs(cloudXYZ);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sampled = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	GraspingUtils::downsampleCloud(cloudXYZ, voxelSize, sampled);

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = GraspingUtils::basicObjectExtraction(sampled, transformation, debugEnabled);

	static bool debugDone = false;
	if (!debugDone && debugEnabled)
	{
		pcl::io::savePCDFileASCII("./orig.pcd", *cloudXYZ);
		pcl::io::savePCDFileASCII("./filtered.pcd", *filtered);
	}

	// Estimate normals and generate an unique cloud
	ROS_INFO("...estimating normals");
	pcl::PointCloud<pcl::Normal>::Ptr normals = CloudUtils::estimateNormals(filtered, Config::getNormalEstimationRadius());
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
	pcl::concatenateFields(*filtered, *normals, *cloud);

	// Descriptor dense evaluation over the point cloud
	ROS_INFO("...performing dense evaluation (%zu points)", cloud->size());
	cv::Mat descriptors;
	Calculator::calculateDescriptors(cloud, Config::getDescriptorParams(), descriptors);

	cv::Mat labels;
	ROS_INFO("...labeling cloud");
	svm->predict(descriptors, labels);

	if (!debugDone && debugEnabled)
		Writer::writeClusteredCloud("./labeled.pcd", cloud, labels);

	debugDone = true;



	/**
	 * 	HERE THE EXTRACTED CLOUD HAS TO BE PUBLISHED AS PointXYZL, WHERE THE LABELS ARE THE CLUSTER NUMBER ACORDING TO
	 * 	THE LABELING DONE BASED ON THE BOW, SO THE NEXT NODE CAN USE IT FOR THE DBSCAN ALGORITHM
	 */


}

int main(int _argn, char **_argv)
{
	ros::init(_argn, _argv, "gazebo_pr2_labeler");
	ros::NodeHandle nodeHandler;
	transformationListener = new tf::TransformListener(ros::Duration(10.0));

	// Load the node's configuration
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(CONFIG_LOCATION))
		throw std::runtime_error((std::string) "Error reading config at " + Utils::getWorkingDirectory() + CONFIG_LOCATION);

	// Load the BoW definition and prepare the clasificator
	ROS_INFO("Training labeling classifier");
	cv::Mat BoW;
	std::map<std::string, std::string> metadata;
	Loader::loadMatrix(Config::get()["bowLocation"].as<std::string>(), BoW, &metadata);
	svm = ClusteringUtils::prepareClasificator(BoW, metadata);

	// Set the subscription to get the point clouds
	ros::Subscriber sub = nodeHandler.subscribe("/head_mount_kinect/depth/points", 1, cloudCallback);

	// Set the publisher
	ros::Publisher pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/labeled_cloud", 1);

	// Keep looping
	ROS_INFO("Node looping");
	ros::spin();

	return EXIT_SUCCESS;
}
