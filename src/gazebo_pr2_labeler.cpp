/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
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

//Global variables
tf::TransformListener *transformationListener;
ros::Publisher pub;
CvSVMPtr svm;

// Method that generates a labeled cloud ready to be published
pcl::PointCloud<pcl::PointXYZL>::Ptr generateLabeledCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_, const cv::Mat &labels_, const bool debug_ = false)
{
	pcl::PointCloud<pcl::PointXYZL>::Ptr labeledCloud = pcl::PointCloud<pcl::PointXYZL>::Ptr(new pcl::PointCloud<pcl::PointXYZL>());

	// Get the indices of the sorted data according to the labels
	cv::Mat indices;
	cv::sortIdx(labels_, indices, cv::SORT_EVERY_COLUMN + cv::SORT_ASCENDING);

	// Generate the output cloud
	int lastLabel = labels_.at<float>(indices.at<int>(0));
	int lastLabelIndex = 0;
	for (size_t i = 0; i < cloud_->size(); i++)
	{
		int index = indices.at<int>(i);
		int label = labels_.at<float>(index);

		pcl::PointNormal p = cloud_->at(index);
		labeledCloud->push_back(PointFactory::createPointXYZL(p.x, p.y, p.z, label));

		if (debug_ && lastLabel != label)
		{
			ROS_INFO(".....label %d, %zu pts", lastLabel, i - lastLabelIndex);
			lastLabel = label;
			lastLabelIndex = i;
		}
	}
	if (debug_)
		ROS_INFO(".....label %d, %zu pts", lastLabel, cloud_->size() - lastLabelIndex);

	return labeledCloud;
}

// Callback called when a new pointcloud received from the sensor
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg_)
{
	ROS_INFO("Cloud received");

	static bool debugEnabled = Config::get()["labelerDebug"].as<bool>();
	static float voxelSize = Config::get()["voxelSize"].as<float>();

	// Convert cloud to PCL format
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*msg_, *cloudXYZ);

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

	// Publish the extrated and labeled cloud
	pcl::PointCloud<pcl::PointXYZL>::Ptr labeledCloud = generateLabeledCloud(cloud, labels, debugEnabled);
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg<pcl::PointXYZL>(*labeledCloud, output);
	output.header.stamp = ros::Time::now();
	output.header.frame_id = FRAME_KINNECT;
	pub.publish(output);

	// Write debug data
	if (!debugDone && debugEnabled)
	{
		Writer::writeClusteredCloud("./clustered.pcd", cloud, labels);
		pcl::io::savePCDFileASCII("./labeled.pcd", *labeledCloud);
	}

	debugDone = true;
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

	// Set the publisher
	pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/labeled_cloud", 1);

	// Set the subscription to get the point clouds
	ros::Subscriber sub = nodeHandler.subscribe("/head_mount_kinect/depth/points", 1, cloudCallback);

	// Keep looping
	ROS_INFO("Node looping");
	ros::spin();

	return EXIT_SUCCESS;
}
