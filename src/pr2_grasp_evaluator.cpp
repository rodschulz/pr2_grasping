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

/***** Debug variables *****/
ros::Publisher cloudPublisher, planeXPublisher, planeZPublisher;


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
				   const std::map<std::string, float> clippingX_,
				   const std::map<std::string, float> clippingZ_,
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

		ROS_DEBUG("...clipping cloud (x:%f -- z:%f)", clippingX_.find("p")->second, clippingZ_.find("p")->second);

		// Clip along X
		pcl::PointCloud<pcl::PointXYZ>::Ptr planeX = debugEnabled_ ? pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()) : pcl::PointCloud<pcl::PointXYZ>::Ptr();
		pcl::PointCloud<pcl::PointXYZ>::Ptr clipped1 = GraspingUtils::planeClipping(cloudXYZ, transformation, AXIS_X, clippingX_.find("p")->second, clippingX_.find("n")->second, planeX);

		// Clip along Z
		pcl::PointCloud<pcl::PointXYZ>::Ptr planeZ = debugEnabled_ ? pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()) : pcl::PointCloud<pcl::PointXYZ>::Ptr();
		pcl::PointCloud<pcl::PointXYZ>::Ptr clipped2 = GraspingUtils::planeClipping(clipped1, transformation, AXIS_Z,  clippingZ_.find("p")->second, clippingZ_.find("n")->second, planeZ);


		// Publish debug data
		if (debugEnabled_)
		{
			sensor_msgs::PointCloud2 cloudMsg;
			pcl::toROSMsg<pcl::PointXYZ>(*clipped2, cloudMsg);
			cloudMsg.header.stamp = ros::Time::now();
			cloudMsg.header.frame_id = msg_->header.frame_id;
			cloudPublisher.publish(cloudMsg);

			sensor_msgs::PointCloud2 planeXMsg;
			pcl::toROSMsg<pcl::PointXYZ>(*planeX, planeXMsg);
			planeXMsg.header.stamp = ros::Time::now();
			planeXMsg.header.frame_id = msg_->header.frame_id;
			planeXPublisher.publish(planeXMsg);

			sensor_msgs::PointCloud2 planeZMsg;
			pcl::toROSMsg<pcl::PointXYZ>(*planeZ, planeZMsg);
			planeZMsg.header.stamp = ros::Time::now();
			planeZMsg.header.frame_id = msg_->header.frame_id;
			planeZPublisher.publish(planeZMsg);
		}


		/***** STAGE 3: check the results *****/
		// Check if there's any point after clipping
		graspState.push_back(!clipped2->empty());
		ROS_INFO("...clipped cloud size: %zu pts", clipped2->size());

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
					  pr2_grasping::GraspEvaluator::Response &response_,
					  const int successThreshold_,
					  const int maxRetries_,
					  const std::map<std::string, float> &object_,
					  const std::map<std::string, float> &head_)
{
	// Prevent excessive logging from the action client library
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME ".actionlib", ros::console::levels::Info))
		ros::console::notifyLoggerLevelsChanged();

	// Clear the status vector to begin a new evaluation
	mutex.lock();
	status.clear();
	mutex.unlock();


	/***** STAGE 1: prepare the robot *****/
	// Point the head to the evaluation pose
	RobotUtils::moveHead(head_.find("x")->second, head_.find("y")->second, head_.find("z")->second);
	ros::Duration(0.5).sleep();


	/***** STAGE 2: evaluate different object's positions *****/
	// Prepare the planning framework
	moveit::planning_interface::MoveGroup::Plan plan;
	std::pair<std::string, std::string> effectorNames = RobotUtils::getEffectorNames(request_.effectorName);
	MoveGroupPtr effector = MoveGroupPtr(new moveit::planning_interface::MoveGroup(effectorNames.first));
	effector->setEndEffector(effectorNames.second);
	effector->setPlannerId("RRTConnectkConfigDefault");


	// Stop any previous movement
	effector->stop();
	ros::Duration(0.5).sleep();


	Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(1, 0, 0), Eigen::Vector3f(0, 0, 1));
	Eigen::Quaternionf incremental = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(0, 0, -1));


	geometry_msgs::PoseStamped evalPose;
	evalPose.header.frame_id = FRAME_BASE;
	evalPose.pose.position.x = object_.find("x")->second;
	evalPose.pose.position.y = object_.find("y")->second;;
	evalPose.pose.position.z = object_.find("z")->second;;


	// Iterate testing a set of poses to evaluate the grasping result
	while (status.size() < 4)
	{
		evalPose.pose.orientation.w = rotation.w();
		evalPose.pose.orientation.x = rotation.x();
		evalPose.pose.orientation.y = rotation.y();
		evalPose.pose.orientation.z = rotation.z();
		effector->setPoseTarget(evalPose);

		ROS_DEBUG_STREAM("*** Moving gripper to pose " << status.size());
		ROS_DEBUG("f:%s - p:(%.3f, %.3f, %.3f) - o:(%.3f, %.3f, %.3f, %.3f)",
				  evalPose.header.frame_id.c_str(),
				  evalPose.pose.position.x,
				  evalPose.pose.position.y,
				  evalPose.pose.position.z,
				  evalPose.pose.orientation.x,
				  evalPose.pose.orientation.y,
				  evalPose.pose.orientation.z,
				  evalPose.pose.orientation.w);

		if (!RobotUtils::move(effector, evalPose, maxRetries_))
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
		if (countTrue(status) >= successThreshold_)
			break;

		rotation = rotation * incremental;
	}

	// Check if there's enough POSITIVE tested poses
	int count = countTrue(status);
	response_.result = count >= successThreshold_;

	ROS_DEBUG_STREAM("Positive tested poses: " << count);
	ROS_INFO("Grasp result: %s", response_.result ? "SUCCESSFUL" : "FAILED");

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
	std::map<std::string, float> clippingX = Config::get()["evaluator"]["clippingX"].as<std::map<std::string, float> >();
	std::map<std::string, float> clippingZ = Config::get()["evaluator"]["clippingZ"].as<std::map<std::string, float> >();
	int successThreshold = Config::get()["evaluator"]["successThreshold"].as<int>();
	int maxRetries = Config::get()["evaluator"]["maxRetries"].as<int>();
	std::map<std::string, float> object = Config::get()["evaluator"]["position"].as<std::map<std::string, float> >();
	std::map<std::string, float> head = Config::get()["evaluator"]["head"].as<std::map<std::string, float> >();


	// Set subscription
	ros::Subscriber subscriber = handler.subscribe<sensor_msgs::PointCloud2>("/move_group/filtered_cloud", 1, boost::bind(cloudCallback, _1, clippingX, clippingZ, debugEnabled));

	// Set service
	ros::ServiceServer evaluationService = handler.advertiseService<pr2_grasping::GraspEvaluator::Request, pr2_grasping::GraspEvaluator::Response>("/pr2_grasping/grasp_evaluator", boost::bind(evaluateGrasping, _1, _2, successThreshold, maxRetries, object, head));

	// Set debug behavior
	if (debugEnabled)
	{
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
			ros::console::notifyLoggerLevelsChanged();

		cloudPublisher = handler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/debug_evaluation_cloud", 1, true);
		planeXPublisher = handler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/debug_evaluation_plane_x", 1, true);
		planeZPublisher = handler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/debug_evaluation_plane_z", 1, true);
	}

	// Start the service
	ROS_INFO("Starting grasping evaluation service");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
