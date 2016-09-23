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


/***** Definition of a custom point type *****/
#define PCL_NO_PRECOMPILE
struct PointXYZNL
{
	PCL_ADD_POINT4D;	// Add x,y,z coordinates
	PCL_ADD_NORMAL4D;	// Add normal coordinates
	uint32_t label;		// Add label

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW		// make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT ( PointXYZNL,
									( float, x, x )
									( float, y, y )
									( float, z, z )
									( float, normal_x, normal_x )
									( float, normal_y, normal_y )
									( float, normal_z, normal_z )
									( uint32_t, label, label )
								  )


/***** Global variables *****/
tf::TransformListener *tfListener;
ros::Publisher pub;
CvSVMPtr svm;
float clippingPlaneZ = 0.615;


/**************************************************/
pcl::PointCloud<PointXYZNL>::Ptr generateLabeledCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
		const cv::Mat &labels_,
		const bool debug_ = false)
{
	pcl::PointCloud<PointXYZNL>::Ptr labeledCloud = pcl::PointCloud<PointXYZNL>::Ptr(new pcl::PointCloud<PointXYZNL>());

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
		PointXYZNL newPoint;
		newPoint.x = p.x;
		newPoint.y = p.y;
		newPoint.z = p.z;
		newPoint.normal_x = p.normal_x;
		newPoint.normal_y = p.normal_y;
		newPoint.normal_z = p.normal_z;
		newPoint.label = label;
		labeledCloud->push_back(newPoint);

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


/**************************************************/
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg_)
{
	if (msg_->height == 0 || msg_->width == 0)
	{
		ROS_WARN("Empty cloud received (h=%d, w=%d), skipping", msg_->height, msg_->width);
		return;
	}

	ROS_INFO("Cloud received");


	/***** STAGE 1: retrieve data *****/
	static bool debugEnabled = Config::get()["labelerDebug"].as<bool>();
	static float voxelSize = Config::get()["voxelSize"].as<float>();

	// Convert cloud to PCL format
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*msg_, *cloudXYZ);
	if (cloudXYZ->empty())
	{
		ROS_WARN("Cloud empty after transformation to PCL, skipping");
		return;
	}


	/***** STAGE 2: prepare the data *****/

	// Get the required transformation
	tf::StampedTransform transformation;
	while (!GraspingUtils::getTransformation(transformation, tfListener, FRAME_KINNECT, FRAME_BASE));

	// Prepare cloud
	CloudUtils::removeNANs(cloudXYZ);
	if (cloudXYZ->empty())
	{
		ROS_WARN("Cloud empty after NaN removal, skipping");
		return;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr sampled = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	GraspingUtils::downsampleCloud(cloudXYZ, voxelSize, sampled);

	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = GraspingUtils::basicPlaneClippingZ(sampled, transformation, clippingPlaneZ, debugEnabled);

	static bool debugDone = false;
	if (!debugDone && debugEnabled)
	{
		pcl::io::savePCDFileASCII("./orig.pcd", *cloudXYZ);
		pcl::io::savePCDFileASCII("./filtered.pcd", *filtered);
	}

	if (filtered->empty())
	{
		ROS_WARN("Cloud empty after filtering, skipping");
		return;
	}


	/***** STAGE 3: calculate relevant info *****/

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


	/***** STAGE 4: publish data *****/
	pcl::PointCloud<PointXYZNL>::Ptr labeledCloud = generateLabeledCloud(cloud, labels, debugEnabled);

	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg<PointXYZNL>(*labeledCloud, output);
	output.header.stamp = ros::Time::now();
	output.header.frame_id = FRAME_KINNECT;
	pub.publish(output);


	// Write debug data
	if (!debugDone && debugEnabled)
	{
		Writer::writeClusteredCloud("./clustered.pcd", cloud, labels);
		pcl::io::savePCDFileASCII("./labeled.pcd", *labeledCloud);
	}
	debugDone = true; // generate debug only once
}


/**************************************************/
int main(int argn_, char **argv_)
{
	ros::init(argn_, argv_, "gazebo_pr2_labeler");
	ros::NodeHandle nodeHandler;
	tfListener = new tf::TransformListener(ros::Duration(10.0));

	// Load the node's configuration
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(GraspingUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + GraspingUtils::getConfigPath());

	clippingPlaneZ = Config::get()["labeler"]["clippingPlaneZ"].as<float>(0.5);

	// Load the BoW definition and prepare the classificator
	ROS_INFO("Training labeling classifier");
	cv::Mat BoW;
	std::map<std::string, std::string> metadata;
	Loader::loadMatrix(Config::get()["bowLocation"].as<std::string>(), BoW, &metadata);
	svm = ClusteringUtils::prepareClasificator(BoW, metadata);

	// Begin spinner
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// Set the publisher
	pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/labeled_cloud", 1);

	// Set the subscription to get the point clouds
	// ros::Subscriber sub = nodeHandler.subscribe("/head_mount_kinect/depth/points", 1, cloudCallback);
	ros::Subscriber sub = nodeHandler.subscribe("/move_group/filtered_cloud", 1, cloudCallback);

	// Keep looping
	ROS_INFO("Labeler node looping");
	// ros::spin();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
