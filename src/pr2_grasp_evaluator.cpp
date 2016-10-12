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


/***** Global variables *****/
boost::mutex mutex;
bool evaluateCloud = false;
tf::TransformListener *tfListener;
std::vector<bool> status;
int successThreshold  = 1;
int maxRetries = -1;

/***** Debug variables *****/
ros::Publisher planePublisher;


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

		/***** STAGE 1: retrieve data *****/
		ROS_DEBUG("Transforming cloud from ROS to PCL");
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromROSMsg(*msg_, *cloudXYZ);
		if (cloudXYZ->empty())
		{
			ROS_WARN("Cloud empty after transformation to PCL, skipping");
			return;
		}

		/***** STAGE 2: transform and remove data *****/
		ROS_DEBUG("Wait transformation");
		tf::StampedTransform transformation;
		while (!GraspingUtils::getTransformation(transformation, tfListener, msg_->header.frame_id, FRAME_BASE));

		ROS_DEBUG("Clipping cloud");
		pcl::PointCloud<pcl::PointXYZ>::Ptr plane = debugEnabled_ ? pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()) : pcl::PointCloud<pcl::PointXYZ>::Ptr();
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = GraspingUtils::basicPlaneClippingZ(cloudXYZ, transformation, clippingPlaneZ_, plane);

		// Publish debug data
		if (debugEnabled_)
			planePublisher.publish(plane);

		// Check if there's any point after clipping
		graspState.push_back(!filtered->empty());

		// If some clouds have been evaluated, then set the result
		if (graspState.size() == 3)
		{
			// Prevent more clouds to be evaluated and set the result
			mutex.lock();
			evaluateCloud = false;
			status.push_back(countTrue(graspState) >= 1);
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

	// Generate the base of the pose
	geometry_msgs::PoseStamped gripperPose;
	gripperPose.header.frame_id = "r_wrist_roll_link";

	// Iterate testing a set of poses to evaluate the grasping result
	while (status.size() < 4)
	{
		// Move the gripper to the evaluation pose
		ROS_DEBUG_STREAM("Moving gripper to pose " << status.size());
		gripperPose.pose = GraspingUtils::genPose(0, 0, 0, DEG2RAD(90 * status.size()), 1, 0, 0);
		if (!RobotUtils::moveGroup(effector, gripperPose, maxRetries))
		{
			ROS_WARN("Unable to move gripper for grasp evaluation, aborting");
			return false;
		}

		// Schedule the evaluation from the sensorial data
		mutex.lock();
		evaluateCloud = true;
		mutex.unlock();

		// Wait until the pose was evaluated
		ROS_DEBUG_STREAM("Testing gripper pose " << status.size());
		size_t current = status.size();
		while (current == status.size())
			ros::Duration(1.0).sleep();
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

		planePublisher = handler.advertise<sensor_msgs::PointCloud2>("/pr2_grasping/debug_evaluation_plane", 1);
	}

	// Start the service
	ROS_INFO("Starting grasping evaluation service");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
