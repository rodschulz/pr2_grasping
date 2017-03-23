/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <pr2_grasping/ObjectCloudData.h>
#include <pr2_grasping/CloudLabeler.h>
#include <pr2_grasping/DescriptorCalc.h>
#include <pr2_grasping/ExperimentId.h>
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
#include "DCH.hpp"
#include "ClusteringUtils.hpp"
#include "GraspingUtils.hpp"
#include "PkgUtils.hpp"
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


/***** Enumeration with the labeling states *****/
enum LabelingState
{
	STATE_IDLE,
	STATE_RECEIVING_CLOUD,
	STATE_LABELING,
};


/***** Global variables *****/
tf::TransformListener *tfListener;
ros::Publisher cloudPub, dataPub;
LabelingState state = STATE_IDLE;
pcl::PointCloud<pcl::PointXYZ>::Ptr receivedCloud;
pcl::PointCloud<pcl::PointNormal>::Ptr writtenCloud;
std::string cloudFrameId;
boost::mutex mutex;
SVMPtr svm;
std::string outputDir;

/***** Debug variables *****/
ros::Publisher limitsPub, planePub;


/**************************************************/
template <typename PointType>
std::pair<geometry_msgs::PointStamped, geometry_msgs::PointStamped> getBoundingBoxLimits(const typename pcl::PointCloud<PointType>::Ptr cloud_,
		const std::string &frameId_)
{
	PointType minPt, maxPt;
	pcl::getMinMax3D< PointType >(*cloud_, minPt, maxPt);

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
	int nlabels = 1;
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
			if (debug_)
				ROS_INFO(".....label %d, %zu pts", lastLabel, i - lastLabelIndex);

			lastLabel = label;
			lastLabelIndex = i;
			nlabels++;
		}
	}
	if (debug_)
		ROS_INFO(".....label %d, %zu pts", lastLabel, cloud_->size() - lastLabelIndex);

	ROS_INFO(".....%d labels found", nlabels);

	return labeledCloud;
}


/**************************************************/
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg_)
{
	static int labelingTries = 0;
	int labelingMaxAttempts = 5;

	if (state == STATE_RECEIVING_CLOUD)
	{
		if (msg_->height == 0 || msg_->width == 0)
		{
			ROS_WARN("Empty cloud received (h=%d, w=%d), skipping", msg_->height, msg_->width);
			return;
		}

		// Stop if too many try have been done
		if (labelingTries++ >= labelingMaxAttempts)
		{
			ROS_WARN("Too many labeling attempts failed, stopping...");
			labelingTries = 0;

			mutex.lock();
			state = STATE_IDLE;
			mutex.unlock();
			return;
		}
		else
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

		cloudFrameId = msg_->header.frame_id;
		receivedCloud = cloudXYZ;

		// Reset control variables
		labelingTries = 0;
		mutex.lock();
		state = STATE_LABELING;
		mutex.unlock();
	}
}


/**************************************************/
bool scheduleLabeling(pr2_grasping::CloudLabeler::Request &request_,
					  pr2_grasping::CloudLabeler::Response &response_)
{
	if (state == STATE_IDLE)
	{
		mutex.lock();
		state = STATE_RECEIVING_CLOUD;
		mutex.unlock();
	}

	response_.result = true;
	return true;
}


/**************************************************/
void labelCloud(const float voxelSize_,
				const float clippingPlaneZ_,
				const bool debugEnabled_,
				const bool writeClouds_)
{
	/***** STAGE 1: prepare the data *****/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ = receivedCloud;

	// Get the required transformation
	tf::StampedTransform transformation;
	while (!GraspingUtils::getTransformation(transformation, tfListener, cloudFrameId, FRAME_BASE));

	// Prepare cloud
	ROS_DEBUG("Removing NANs");
	CloudUtils::removeNANs(cloudXYZ);
	if (cloudXYZ->empty())
	{
		ROS_WARN("Cloud empty after NaN removal, skipping");
		mutex.lock();
		state = STATE_IDLE;
		receivedCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr();
		mutex.unlock();
		return;
	}


	// Downsample if requested
	pcl::PointCloud<pcl::PointXYZ>::Ptr sampled = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	if (voxelSize_ > 0)
		GraspingUtils::downsampleCloud(cloudXYZ, voxelSize_, sampled);
	else
		sampled = cloudXYZ;


	ROS_DEBUG("Clipping cloud");
	pcl::PointCloud<pcl::PointXYZ>::Ptr clippingPlane = debugEnabled_ ? pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()) : pcl::PointCloud<pcl::PointXYZ>::Ptr();
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = GraspingUtils::planeClipping(sampled, transformation, AXIS_Z, clippingPlaneZ_, 1, clippingPlane);


	ROS_DEBUG("Cloud clipped");
	if (filtered->empty())
	{
		ROS_WARN("Cloud empty after filtering, skipping");
		mutex.lock();
		state = STATE_IDLE;
		receivedCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr();
		mutex.unlock();
		return;
	}


	// Transform the filtered cloud to base's frame
	ROS_DEBUG("Transforming cloud from '%s' to '%s'", cloudFrameId.c_str(), FRAME_BASE);
	while (!GraspingUtils::getTransformation(transformation, tfListener, FRAME_BASE, cloudFrameId));
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>());
	pcl_ros::transformPointCloud(*filtered, *transformed, transformation);
	std::pair<geometry_msgs::PointStamped, geometry_msgs::PointStamped> limits = getBoundingBoxLimits<pcl::PointXYZ>(transformed, FRAME_BASE);


	if (debugEnabled_)
	{
		ROS_DEBUG("Publishing debug data");

		geometry_msgs::PoseArray arrayMsg;
		arrayMsg.header.frame_id = limits.first.header.frame_id;
		arrayMsg.poses.push_back(GraspingUtils::genPose(limits.first.point.x, limits.first.point.y, limits.first.point.z));
		arrayMsg.poses.push_back(GraspingUtils::genPose(limits.second.point.x, limits.second.point.y, limits.second.point.z));
		limitsPub.publish(arrayMsg);

		sensor_msgs::PointCloud2 planeMsg;
		pcl::toROSMsg<pcl::PointXYZ>(*clippingPlane, planeMsg);
		planeMsg.header.stamp = ros::Time::now();
		planeMsg.header.frame_id = cloudFrameId;
		planePub.publish(planeMsg);
	}


	/***** STAGE 2: calculate relevant info *****/

	// Calculate and update the cloud's viewpoint to perform a correct normals estimation
	geometry_msgs::Point viewPoint = GraspingUtils::transformPoint(tfListener, FRAME_BASE, cloudFrameId, geometry_msgs::Point());
	transformed->sensor_origin_.x() = viewPoint.x;
	transformed->sensor_origin_.y() = viewPoint.y;
	transformed->sensor_origin_.z() = viewPoint.z;


	// Estimate normals and generate an unique cloud
	ROS_INFO("...estimating normals");
	pcl::PointCloud<pcl::Normal>::Ptr normals = CloudUtils::estimateNormals(transformed, Config::getNormalEstimationRadius());
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
	pcl::concatenateFields(*transformed, *normals, *cloud);
	cloud->sensor_origin_ = transformed->sensor_origin_;


	// Write the object's cloud
	if (writeClouds_)
	{
		pr2_grasping::ExperimentId srv;
		if (ros::service::call("/pr2_grasping/experiment_id", srv))
		{
			writtenCloud = cloud;
			std::string filename = outputDir + srv.response.id + ".pcd";
			ROS_DEBUG("Writing cloud to %s", filename.c_str());
			pcl::io::savePCDFileASCII(filename, *cloud);
		}
	}


	// Descriptor dense evaluation over the point cloud
	ROS_INFO("...performing dense evaluation (%zu points)", cloud->size());
	cv::Mat descriptors;
	DCH::computeDense(cloud, Config::getLabelingDescriptorParams(), descriptors);

	cv::Mat labels;
	ROS_INFO("...labeling cloud");
	svm->predict(descriptors, labels);


	/***** STAGE 3: publish data *****/
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
	cloudPub.publish(objectCloud);
	dataPub.publish(objectData);


	// Write debug data
	static bool cloudsWritten = false;
	if (debugEnabled_ && !cloudsWritten)
	{
		Writer::writeClusteredCloud(outputDir + "DEBUG_cluster_colored.pcd", cloud, labels);
		pcl::io::savePCDFileASCII(outputDir + "DEBUG_labeled.pcd", *labeledCloud);
		pcl::io::savePCDFileASCII(outputDir + "DEBUG_full.pcd", *sampled);
		cloudsWritten =  true;
	}


	// Reset control variables
	mutex.lock();
	state = STATE_IDLE;
	receivedCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr();
	mutex.unlock();
}


/**************************************************/
bool computeDescriptor(pr2_grasping::DescriptorCalc::Request &request_,
					   pr2_grasping::DescriptorCalc::Response &response_,
					   const bool debugEnabled_)
{
	if (writtenCloud)
	{
		static size_t k = 0;
		ROS_INFO("Computing descriptor on-demand (%zu)", k++);

		// Extract the target point
		int nearest = GraspingUtils::findNearestPoint(writtenCloud, request_.target);
		ROS_DEBUG("\ttarget point: %d", nearest);

		// Calculate the descriptor
		DescriptorParamsPtr params = Config::getGraspingDescriptorParams();
		response_.params.type = params->type;

		if (params->type == Params::DESCRIPTOR_DCH)
		{
			DCHParams *dchParams = dynamic_cast<DCHParams *>(params.get());
			if (dchParams == NULL)
			{
				ROS_ERROR("Unable to cast parameters to DCH params");
				return false;
			}


			// Compute the descriptor
			dchParams->angle = request_.angle.data;
			std::vector<BandPtr> desc = DCH::calculateDescriptor(writtenCloud, params, nearest);

			size_t nbands = desc.size();
			size_t bandSize = desc[0]->descriptor.size();
			size_t descriptorSize = nbands * bandSize;

			ROS_DEBUG("\tDCH computed (size: %zu - angle: %f)", descriptorSize, dchParams->angle);


			// Add params to the response
			response_.descriptor.resize(descriptorSize);
			response_.index.data = nearest;
			response_.params.searchRadius = dchParams->searchRadius;
			response_.params.bandNumber = dchParams->bandNumber;
			response_.params.bandWidth = dchParams->bandWidth;
			response_.params.bidirectional = dchParams->bidirectional;
			response_.params.useProjection = dchParams->useProjection;
			response_.params.binNumber = dchParams->binNumber;
			response_.params.stat = dchParams->stat;


			// Produce debug if requested
			if (request_.sendDebug.data)
			{
				// Add band axes to the response
				response_.bandAxes.resize(desc.size());
				for (size_t band = 0; band < desc.size(); band++)
				{
					Eigen::Vector3f origin = desc[band]->axis.pointAt(0);
					response_.bandAxes[band].origin.x = origin.x();
					response_.bandAxes[band].origin.y = origin.y();
					response_.bandAxes[band].origin.z = origin.z();

					Eigen::Vector3f director = desc[band]->axis.pointAt(10);
					response_.bandAxes[band].director.x = director.x();
					response_.bandAxes[band].director.y = director.y();
					response_.bandAxes[band].director.z = director.z();
				}

				// Add the point cloud to the response
				pcl::toROSMsg(*writtenCloud, response_.cloud);
			}


			// Add descriptor data to the response
			ROS_DEBUG("\tcopying message");
			for (size_t band = 0; band < nbands; band++)
				for (size_t seq = 0; seq < bandSize; seq++)
					response_.descriptor[band * bandSize + seq].data = desc[band]->descriptor[seq];


			if (debugEnabled_)
			{
				std::vector<Eigen::ParametrizedLine<float, 3> > axes;
				for (size_t band = 0; band < desc.size(); band++)
					axes.push_back(desc[band]->axis);

				static long idx = 0;
				std::string filename = DEBUG_PREFIX "computed_descriptor_" + boost::lexical_cast<std::string>(idx++);
				GraspingUtils::generateGraspCloud(filename,
												  writtenCloud,
												  dchParams->searchRadius,
												  dchParams->bidirectional,
												  request_.target,
												  nearest,
												  axes);
			}
		}
		else
		{
			ROS_WARN("Unable to compute required descriptor type (%s)", Params::descType[params->type].c_str());
			return false;
		}
	}

	return true;
}


/**************************************************/
int main(int argn_, char **argv_)
{
	ros::init(argn_, argv_, "pr2_cloud_labeler");
	ros::NodeHandle handler;
	tfListener = new tf::TransformListener(ros::Duration(10.0));
	outputDir = PkgUtils::getOutputPath();


	/********** Start spinning **********/
	ros::AsyncSpinner spinner(3);
	spinner.start();


	/********** Load the node's configuration **********/
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(PkgUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + PkgUtils::getConfigPath());

	bool debugEnabled = Config::get()["labelerDebug"].as<bool>();
	float voxelSize = Config::get()["labeler"]["voxelSize"].as<float>();
	float clippingPlaneZ = Config::get()["labeler"]["clippingPlaneZ"].as<float>();
	bool writeClouds = Config::get()["labeler"]["writeClouds"].as<bool>();
	std::string topicName = Config::get()["labeler"]["pointcloudTopic"].as<std::string>();


	/********** Load the codebook and prepare the classifier **********/
	ROS_INFO("Loading codebook");
	cv::Mat codebook;
	std::string codebookFile = ros::package::getPath(PACKAGE_NAME) + "/" + Config::get()["labeler"]["codebookLocation"].as<std::string>();
	std::map<std::string, std::string> metadata;
	while (!Loader::loadMatrix(codebookFile, codebook, &metadata))
		ROS_WARN("...unable to load codebook at %s", codebookFile.c_str());

	ROS_INFO("Training labeling classifier");
	svm = ClusteringUtils::prepareClassifier(codebook, metadata);


	/********** Set subscriptions/publishers **********/
	cloudPub = handler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/labeled_cloud", 1, true);
	dataPub = handler.advertise<pr2_grasping::ObjectCloudData>("/pr2_grasping/object_cloud_data", 1);
	ros::Subscriber sub = handler.subscribe<sensor_msgs::PointCloud2>(topicName, 1, cloudCallback);

	if (debugEnabled)
	{
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
			ros::console::notifyLoggerLevelsChanged();

		limitsPub = handler.advertise<geometry_msgs::PoseArray>("/pr2_grasping/debug_limits", 1);
		planePub = handler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/debug_clipping_plane", 1);
	}


	/********** Set services **********/
	ROS_INFO("Starting labeler service");
	ros::ServiceServer labelerService = handler.advertiseService("/pr2_grasping/cloud_labeler", scheduleLabeling);
	ROS_INFO("Starting calculation service");
	ros::ServiceServer calculationService = handler.advertiseService<pr2_grasping::DescriptorCalc::Request, pr2_grasping::DescriptorCalc::Response>("/pr2_grasping/descriptor_calculator", boost::bind(computeDescriptor, _1, _2, debugEnabled));


	// Spin at 20 Hz
	ros::Rate r(20);
	while (ros::ok())
	{
		if (receivedCloud)
			labelCloud(voxelSize, clippingPlaneZ, debugEnabled, writeClouds);
		r.sleep();
	}


	ros::shutdown();
	return EXIT_SUCCESS;
}
