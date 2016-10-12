/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pr2_grasping/GraspEvaluator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <boost/signals2/mutex.hpp>
#include "Config.hpp"
#include "GraspingUtils.hpp"
#include "RobotUtils.hpp"


#define CLOUDS_FOR_EVALUATION	3


/***** Global variables *****/
boost::mutex mutex;
bool evaluateCloud = false;
tf::TransformListener *tfListener;
std::vector<bool> status;
int successThreshold  = 1;
int maxRetries = -1;

/***** Debug variables *****/
ros::Publisher cloudPublisher, planePublisher;


/**************************************************/
inline int countTrue(const std::vector<bool> &array_)
{
	int count = 0;
	for (std::vector<bool>::const_iterator it = array_.begin(); it != array_.end(); it++)
		count += (*it ? 1 : 0);

	return count;
}


/**************************************************/
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg_,
				   const float clippingPlaneZ_,
				   const bool debugEnabled_)
{
	if (evaluateCloud)
	{
		// Array to store evaluations
		static std::vector<bool> graspState;

		ROS_DEBUG("===== Begining cloud evaluation =====");

		/***** STAGE 1: retrieve data *****/
		ROS_DEBUG("...transforming cloud from ROS to PCL");
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromROSMsg(*msg_, *cloudXYZ);
		if (cloudXYZ->empty())
		{
			ROS_WARN("...cloud empty after transformation to PCL, skipping");
			return;
		}

		/***** STAGE 2: transform and remove data *****/
		tf::StampedTransform transformation;
		while (!GraspingUtils::getTransformation(transformation, tfListener, msg_->header.frame_id, FRAME_BASE));

		ROS_DEBUG("...clipping cloud");
		pcl::PointCloud<pcl::PointXYZ>::Ptr plane = debugEnabled_ ? pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()) : pcl::PointCloud<pcl::PointXYZ>::Ptr();
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = GraspingUtils::basicPlaneClippingZ(cloudXYZ, transformation, clippingPlaneZ_, plane);

		// Publish debug data
		if (debugEnabled_)
		{
			sensor_msgs::PointCloud2 cloudMsg;
			pcl::toROSMsg<pcl::PointXYZ>(*filtered, cloudMsg);
			cloudMsg.header.stamp = ros::Time::now();
			cloudMsg.header.frame_id = msg_->header.frame_id;
			cloudPublisher.publish(cloudMsg);

			sensor_msgs::PointCloud2 planeMsg;
			pcl::toROSMsg<pcl::PointXYZ>(*plane, planeMsg);
			planeMsg.header.stamp = ros::Time::now();
			planeMsg.header.frame_id = msg_->header.frame_id;
			planePublisher.publish(planeMsg);
		}

		// Check if there's any point after clipping
		graspState.push_back(!filtered->empty());
		ROS_INFO("...filtered cloud size: %zu pts", filtered->size());

		// If some clouds have been evaluated, then set the result
		if (graspState.size() == CLOUDS_FOR_EVALUATION)
		{
			bool result = countTrue(graspState) >= (CLOUDS_FOR_EVALUATION / 2.0);
			ROS_INFO("...position tested: %s", result ? "POSITIVE" : "NEGATIVE");

			mutex.lock();
			evaluateCloud = false;
			status.push_back(result);
			mutex.unlock();

			// Clear the state vector
			graspState.clear();
		}
	}
}


/**************************************************/
bool evaluateGrasping(pr2_grasping::GraspEvaluator::Request  &request_,
					  pr2_grasping::GraspEvaluator::Response &response_)
{
	// Clear the status vector to begin a new evaluation
	mutex.lock();
	status.clear();
	mutex.unlock();

	// Prepare the planning framework
	moveit::planning_interface::MoveGroup::Plan plan;
	std::pair<std::string, std::string> effectorNames = RobotUtils::getEffectorNames(request_.effectorName);
	MoveGroupPtr effector = MoveGroupPtr(new moveit::planning_interface::MoveGroup(effectorNames.first));
	effector->setEndEffector(effectorNames.second);
	effector->setPlannerId("RRTConnectkConfigDefault");

	Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(1, 0, 0), Eigen::Vector3f(0, 1, 0));
	Eigen::Quaternionf incremental = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(0, 0, -1));

	geometry_msgs::PoseStamped evalPose;
	evalPose.header.frame_id = FRAME_BASE;
	evalPose.pose.position.x = 0.5;
	evalPose.pose.position.y = -0.15;
	evalPose.pose.position.z = 1.1;

	// Iterate testing a set of poses to evaluate the grasping result
	while (status.size() < 4)
	{
		evalPose.pose.orientation.w = rotation.w();
		evalPose.pose.orientation.x = rotation.x();
		evalPose.pose.orientation.y = rotation.y();
		evalPose.pose.orientation.z = rotation.z();

		ROS_DEBUG_STREAM("*** Moving gripper to pose " << status.size());
		effector->setPoseTarget(evalPose);
		if (!RobotUtils::move(effector, evalPose, maxRetries))
		{
			ROS_WARN("Unable to move gripper for grasp evaluation, aborting");
			return false;
		}

		// Schedule the evaluation over the point clouds
		mutex.lock();
		evaluateCloud = true;
		mutex.unlock();

		// Wait until the pose was evaluated
		size_t current = status.size();
		ROS_DEBUG_STREAM("Testing gripper pose " << current);
		while (current == status.size())
			ros::Duration(1.0).sleep();

		// Check early finish
		if (countTrue(status) >= successThreshold)
			break;

		rotation = rotation * incremental;
	}

	// If at least 2 poses give a positive result, then the grasping was successful
	int count = countTrue(status);
	ROS_INFO_STREAM("Positive tested poses: " << count);
	response_.result = count >= successThreshold;
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

	bool debugEnabled = Config::get()["evaluatorDebug"].as<bool>();
	float clippingPlaneZ = Config::get()["evaluator"]["clippingPlaneZ"].as<float>();
	successThreshold = Config::get()["evaluator"]["successThreshold"].as<int>();
	maxRetries = Config::get()["evaluator"]["maxRetries"].as<int>();

	// Set subscription
	ros::Subscriber subscriber = handler.subscribe<sensor_msgs::PointCloud2>("/move_group/filtered_cloud", 1, boost::bind(cloudCallback, _1, clippingPlaneZ, debugEnabled));

	// Set service
	ros::ServiceServer evaluationService = handler.advertiseService("/pr2_grasping/grasp_evaluator", evaluateGrasping);

	// Set debug behavior
	if (debugEnabled)
	{
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
			ros::console::notifyLoggerLevelsChanged();

		cloudPublisher = handler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/debug_evaluation_cloud", 1);
		planePublisher = handler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/debug_evaluation_plane", 1);
	}

	// Start the service
	ROS_INFO("Starting grasping evaluation service");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
