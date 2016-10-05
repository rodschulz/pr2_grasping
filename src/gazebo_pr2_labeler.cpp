/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <pr2_grasping/ObjectCloudData.h>
#include <pr2_grasping/GazeboLabeler.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <boost/signals2/mutex.hpp>
#include "Config.hpp"
#include "Loader.hpp"
#include "CloudUtils.hpp"
#include "Calculator.hpp"
#include "ClusteringUtils.hpp"
#include "GraspingUtils.hpp"
#include "RobotUtils.hpp"
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
ros::Publisher cloudPublisher, dataPublisher;
CvSVMPtr svm;
boost::mutex mutex;
bool labelingScheduled = false;

/***** Debug variables *****/
ros::Publisher limitsPublisher, planePublisher;


/**************************************************/
std::pair<geometry_msgs::PointStamped, geometry_msgs::PointStamped> getBoundingBoxLimits(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
		const std::string &frameId_)
{
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D<pcl::PointXYZ>(*cloud_, minPt, maxPt);

	ros::Time nowStamp = ros::Time::now();

	geometry_msgs::PointStamped minLimit;
	minLimit.header.stamp = nowStamp;
	minLimit.header.frame_id = frameId_;
	minLimit.point.x = minPt.x;
	minLimit.point.y = minPt.y;
	minLimit.point.z = minPt.z;

	geometry_msgs::PointStamped maxLimit;
	maxLimit.header.stamp = nowStamp;
	maxLimit.header.frame_id = frameId_;
	maxLimit.point.x = maxPt.x;
	maxLimit.point.y = maxPt.y;
	maxLimit.point.z = maxPt.z;

	return std::pair<geometry_msgs::PointStamped, geometry_msgs::PointStamped>(minLimit, maxLimit);
}


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
	int nlabels = 0;
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

		if (lastLabel != label)
		{
			lastLabel = label;
			lastLabelIndex = i;
			nlabels++;

			if (debug_)
				ROS_INFO(".....label %d, %zu pts", lastLabel, i - lastLabelIndex);
		}
	}
	if (debug_)
		ROS_INFO(".....label %d, %zu pts", lastLabel, cloud_->size() - lastLabelIndex);

	ROS_INFO(".....%d labels found", nlabels);

	return labeledCloud;
}


/**************************************************/
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg_,
				   const float voxelSize_,
				   const float clippingPlaneZ_,
				   const bool debugEnabled_,
				   const bool writeClouds_)
{
	if (labelingScheduled)
	{
		if (msg_->height == 0 || msg_->width == 0)
		{
			ROS_WARN("Empty cloud received (h=%d, w=%d), skipping", msg_->height, msg_->width);
			return;
		}

		ROS_INFO("Cloud received");


		/***** STAGE 1: retrieve data *****/

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
		while (!GraspingUtils::getTransformation(transformation, tfListener, msg_->header.frame_id, FRAME_BASE));

		// Prepare cloud
		ROS_DEBUG("Removing NANs");
		CloudUtils::removeNANs(cloudXYZ);
		if (cloudXYZ->empty())
		{
			ROS_WARN("Cloud empty after NaN removal, skipping");
			return;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr sampled = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
		GraspingUtils::downsampleCloud(cloudXYZ, voxelSize_, sampled);

		ROS_DEBUG("Clipping cloud");
		pcl::PointCloud<pcl::PointXYZ>::Ptr clippingPlane = debugEnabled_ ? pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()) : pcl::PointCloud<pcl::PointXYZ>::Ptr();
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = GraspingUtils::basicPlaneClippingZ(sampled, transformation, clippingPlaneZ_, clippingPlane);

		ROS_DEBUG("Cloud clipped");
		if (filtered->empty())
		{
			ROS_WARN("Cloud empty after filtering, skipping");
			return;
		}

		// Transform the filtered cloud to base's frame
		ROS_DEBUG("Transforming cloud from '%s' to '%s'", msg_->header.frame_id.c_str(), FRAME_BASE);
		while (!GraspingUtils::getTransformation(transformation, tfListener, FRAME_BASE, msg_->header.frame_id));
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>());
		pcl_ros::transformPointCloud(*filtered, *transformed, transformation);
		std::pair<geometry_msgs::PointStamped, geometry_msgs::PointStamped> limits = getBoundingBoxLimits(transformed, FRAME_BASE);

		if (debugEnabled_)
		{
			ROS_DEBUG("Publishing debug data");

			geometry_msgs::PoseArray arrayMsg;
			arrayMsg.header.frame_id = limits.first.header.frame_id;
			arrayMsg.poses.push_back(GraspingUtils::genPose(limits.first.point.x, limits.first.point.y, limits.first.point.z));
			arrayMsg.poses.push_back(GraspingUtils::genPose(limits.second.point.x, limits.second.point.y, limits.second.point.z));
			limitsPublisher.publish(arrayMsg);

			sensor_msgs::PointCloud2 planeMsg;
			pcl::toROSMsg<pcl::PointXYZ>(*clippingPlane, planeMsg);
			planeMsg.header.stamp = ros::Time::now();
			planeMsg.header.frame_id = msg_->header.frame_id;
			planePublisher.publish(planeMsg);
		}


		/***** STAGE 3: calculate relevant info *****/

		// Estimate normals and generate an unique cloud
		ROS_INFO("...estimating normals");
		pcl::PointCloud<pcl::Normal>::Ptr normals = CloudUtils::estimateNormals(transformed, Config::getNormalEstimationRadius());
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
		pcl::concatenateFields(*transformed, *normals, *cloud);

		// Descriptor dense evaluation over the point cloud
		ROS_INFO("...performing dense evaluation (%zu points)", cloud->size());
		cv::Mat descriptors;
		Calculator::calculateDescriptors(cloud, Config::getDescriptorParams(), descriptors);

		cv::Mat labels;
		ROS_INFO("...labeling cloud");
		svm->predict(descriptors, labels);


		/***** STAGE 4: publish data *****/
		pcl::PointCloud<PointXYZNL>::Ptr labeledCloud = generateLabeledCloud(cloud, labels, debugEnabled_);

		sensor_msgs::PointCloud2 objectCloud;
		pcl::toROSMsg<PointXYZNL>(*labeledCloud, objectCloud);
		objectCloud.header.stamp = ros::Time::now();
		objectCloud.header.frame_id = FRAME_BASE;

		pr2_grasping::ObjectCloudData objectData;
		objectData.cloud = objectCloud;
		objectData.boundingBoxMin = limits.first;
		objectData.boundingBoxMax = limits.second;

		ROS_DEBUG("Publishing data");
		cloudPublisher.publish(objectCloud);
		dataPublisher.publish(objectData);


		// Write debug data
		static bool cloudsWritten = false;
		if (!cloudsWritten && writeClouds_)
		{
			Writer::writeClusteredCloud("./cluster_colored.pcd", cloud, labels);
			pcl::io::savePCDFileASCII("./labeled.pcd", *labeledCloud);
			cloudsWritten =  true;
		}

		mutex.lock();
		labelingScheduled = false;
		mutex.unlock();
	}
}


/**************************************************/
bool scheduleLabeling(pr2_grasping::GazeboLabeler::Request  &request_,
					  pr2_grasping::GazeboLabeler::Response &response_)
{
	mutex.lock();
	labelingScheduled = true;
	mutex.unlock();

	response_.result = true;
	return true;
}


/**************************************************/
int main(int argn_, char **argv_)
{
	ros::init(argn_, argv_, "gazebo_pr2_labeler");
	ros::NodeHandle handler;
	tfListener = new tf::TransformListener(ros::Duration(10.0));

	// Load the node's configuration
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(GraspingUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + GraspingUtils::getConfigPath());


	// Retrieve execution params
	bool debugEnabled = Config::get()["labelerDebug"].as<bool>();
	float voxelSize = Config::get()["labeler"]["voxelSize"].as<float>();
	float clippingPlaneZ = Config::get()["labeler"]["clippingPlaneZ"].as<float>();
	bool writeClouds = Config::get()["labeler"]["writeClouds"].as<bool>();


	// Load the BoW definition and prepare the classificator
	ROS_INFO("Training labeling classifier");
	cv::Mat BoW;
	std::map<std::string, std::string> metadata;
	Loader::loadMatrix(Config::get()["labeler"]["bowLocation"].as<std::string>(), BoW, &metadata);
	svm = ClusteringUtils::prepareClasificator(BoW, metadata);


	// Begin spinner
	ros::AsyncSpinner spinner(2);
	spinner.start();


	// Set the publishers
	cloudPublisher = handler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/labeled_cloud", 5);
	dataPublisher = handler.advertise<pr2_grasping::ObjectCloudData>("/pr2_grasping/object_cloud_data", 5);


	// Service for setup configuration
	ros::ServiceServer labelerService = handler.advertiseService("/pr2_grasping/gazebo_labeler", scheduleLabeling);


	if (debugEnabled)
	{
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
			ros::console::notifyLoggerLevelsChanged();

		limitsPublisher = handler.advertise<geometry_msgs::PoseArray>("/pr2_grasping/debug_limits", 1);
		planePublisher = handler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/debug_clipping_plane", 1);
	}


	// Set the subscription to get the point clouds
	std::string topicName = Config::get()["labeler"]["pointcloudTopic"].as<std::string>();
	ros::Subscriber sub = handler.subscribe<sensor_msgs::PointCloud2>(topicName, 1, boost::bind(cloudCallback, _1, voxelSize, clippingPlaneZ, debugEnabled, writeClouds));


	// Keep looping
	ROS_INFO("Labeler node looping");
	// ros::spin();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
