/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pr2_grasping/GraspingData.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>
#include <boost/signals2/mutex.hpp>
#include <deque>
#include "Config.hpp"
#include "GraspingUtils.hpp"
#include "RobotUtils.hpp"

#include "ros/timer_options.h"


#define TARGET_OBJECT		"target_object"
// #define PRINT_POSE(msg, point, frame)	ROS_INFO("..." msg "= (%.2f, %.2f, %.2f) - ref: %s", (point).x, (point).y, (point).z, (frame).c_str())


// Pointer to a move group interface
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;


// Queue of grasping points
std::deque<pr2_grasping::GraspingData> queue;
// Max length allowed for the message queue
unsigned int queueMaxsize = 5;
// Mutex for concurrent access to the queue
boost::mutex mutex;
// Margin used in the collision object generation
float collisionMargin = 0.01;


/** DEBUG ONLY **/
ros::Publisher posearrayPublisher;
/** DEBUG ONLY **/


/**************************************************/
void graspingPointsCallback(const pr2_grasping::GraspingDataConstPtr &msg_)
{
	// Prevent queue from growing endlessly
	if (queue.size() < queueMaxsize)
	{
		ROS_INFO("New points received");

		// Add the new point to the queue
		mutex.lock();
		queue.push_back(*msg_);
		mutex.unlock();
	}
	else
		ROS_INFO_ONCE("Message queue full at size %zu, discarding...", queue.size());
}


/**********************************************************************/
moveit_msgs::CollisionObject genCollisionObject(const std::string objectId_,
		const std::string frameId_,
		const geometry_msgs::Pose &objectPose_,
		const float dimX_,
		const float dimY_,
		const float dimZ_)
{
	moveit_msgs::CollisionObject collision;

	collision.id = objectId_;
	collision.header.stamp = ros::Time::now();
	collision.header.frame_id = frameId_;

	// Add the bounding box of the object for collisions and its pose
	shape_msgs::SolidPrimitive primitive;
	primitive.type = shape_msgs::SolidPrimitive::BOX;
	primitive.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = dimX_;
	primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dimY_;
	primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dimZ_;

	collision.primitives.push_back(primitive);
	collision.primitive_poses.push_back(objectPose_);

	collision.operation = moveit_msgs::CollisionObject::ADD;

	return collision;
}


/**************************************************/
void tt(const ros::TimerEvent &event_, std::string _caca
		//const MoveGroupPtr &effector_
		//,
		//tf::TransformListener *tfListener_
		//,
		//const ros::Publisher &posePublisher_
	   )
{
	while (!queue.empty())
	{
		/**ROS_INFO("***** p=(%.2f, %.2f, %.2f) + l=%d *****", queue.front().point.x, queue.front().point.y, queue.front().point.z, queue.front().label);

		std::string targetFrame = FRAME_BASE;
		// effector->setEndEffector("right_eef");
		effector->setPoseReferenceFrame(targetFrame);
		ROS_INFO("...plan frame: %s - pose frame: %s", effector->getPlanningFrame().c_str(), effector->getPoseReferenceFrame().c_str());

		// Generate the target pose
		geometry_msgs::Pose target = GraspingUtils::genPose(0.546, 0.011, 0.77, DEG2RAD(45), 1, 1, 0);
		//		geometry_msgs::Pose armPose = GraspingUtils::genPose(queue.front().x, queue.front().y, queue.front().z, DEG2RAD(225), 0, 1, 1);
		//		armPose.position.x = queue.front().x;
		//		armPose.position.y = queue.front().y;
		//		armPose.position.z = queue.front().z;
		//		armPose.orientation.w = 1.0;

		// Print positions
		geometry_msgs::PoseStamped current = effector->getCurrentPose();
		PRINT_POSE("curr", current.pose.position, current.header.frame_id);
		geometry_msgs::Point transformed = GraspingUtils::transformPose(tfListener, targetFrame, current.header.frame_id, current.pose.position);
		PRINT_POSE("curr", transformed, targetFrame);
		PRINT_POSE("tget", target.position, targetFrame);
		transformed = GraspingUtils::transformPose(tfListener, current.header.frame_id, targetFrame, target.position);
		PRINT_POSE("tget", transformed, current.header.frame_id);

		// Publish the target pose so it can be visualized using RViz
		geometry_msgs::PoseStamped msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = targetFrame;
		msg.pose = target;
		posePublisher.publish(msg);

		// Pop a point from the queue
		mutex.lock();
		queue.pop_front();
		mutex.unlock();

		effector->setPoseTarget(msg);

		// Plan the trajectory
		moveit::planning_interface::MoveGroup::Plan armPlan;
		ROS_INFO("...planing effector trajectory");
		bool planningOk = effector->plan(armPlan);
		ROS_INFO("...trajectory plan %s", planningOk ? "SUCCESSFUL" : "FAILED");

		// Move the effector
		if (planningOk)
		{
			ROS_INFO("...moving right effector");
			effector->move();
			ROS_INFO("...effector movement completed");

			current = effector->getCurrentPose();
			PRINT_POSE("curr", current.pose.position, current.header.frame_id);
			geometry_msgs::Point transformed = GraspingUtils::transformPose(tfListener, targetFrame, current.header.frame_id, current.pose.position);
			PRINT_POSE("curr", transformed, targetFrame);
		}
		else
			ROS_INFO("...movement aborted");**/
	}
}


/**************************************************/
void timerCallback(const ros::TimerEvent &event_,
				   moveit::planning_interface::PlanningSceneInterface *planningScene_,
				   MoveGroupPtr &effector_,
				   tf::TransformListener *tfListener_,
				   ros::Publisher &posePubslisher_)
{
	while (!queue.empty())
	{
		ROS_INFO("Processing grasping points");

		/********** STAGE 1: add collisions to the planning scene **********/
		geometry_msgs::PointStamped minPt = queue.front().boundingBoxMin;
		geometry_msgs::PointStamped maxPt = queue.front().boundingBoxMax;

		geometry_msgs::Point minTf = GraspingUtils::transformPoint(tfListener_, FRAME_BASE, minPt.header.frame_id, minPt.point);
		geometry_msgs::Point maxTf = GraspingUtils::transformPoint(tfListener_, FRAME_BASE, maxPt.header.frame_id, maxPt.point);

		float dimX = maxTf.x - minTf.x + collisionMargin;
		float dimY = maxTf.y - minTf.y + collisionMargin;
		float dimZ = maxTf.z - minTf.z + collisionMargin;

		// float dimX = maxPt.point.x - minPt.point.x + collisionMargin;
		// float dimY = maxPt.point.y - minPt.point.y + collisionMargin;
		// float dimZ = maxPt.point.z - minPt.point.z + collisionMargin;

		// geometry_msgs::Pose targetPose = GraspingUtils::genPose(minTf.x - dimX * 0.5, minTf.y + dimY * 0.5, minTf.z + dimZ * 0.5);
		geometry_msgs::Pose targetPose = GraspingUtils::genPose(minTf.x, minTf.y, minTf.z);
		moveit_msgs::CollisionObject targetCollision = genCollisionObject(TARGET_OBJECT, FRAME_BASE, targetPose, dimX, dimY, dimZ);

		std::vector<moveit_msgs::CollisionObject> collisions;
		collisions.push_back(targetCollision);

		ROS_INFO("...adding collisions to scene");
		planningScene_->addCollisionObjects(collisions);
		ros::Duration(2.0).sleep();


		/** DEBUG ONLY **/
		geometry_msgs::PoseStamped msg;
		msg.header.frame_id = FRAME_BASE;
		msg.pose = targetPose;
		posePubslisher_.publish(msg);

		geometry_msgs::PoseArray array;
		array.header.frame_id = minPt.header.frame_id;
		array.poses.push_back(GraspingUtils::genPose(minPt.point.x, minPt.point.y, minPt.point.z));
		array.poses.push_back(GraspingUtils::genPose(maxPt.point.x, maxPt.point.y, maxPt.point.z));
		posearrayPublisher.publish(array);
		/** DEBUG ONLY **/


		/********** STAGE 2: generate grasp **********/
	}
}


/**************************************************/
int main(int _argn, char **_argv)
{
	ros::init(_argn, _argv, "pr2_grasper");
	ros::NodeHandle handler;
	tf::TransformListener *tfListener = new tf::TransformListener(ros::Duration(10.0));


	// Load the node's configuration
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(GraspingUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + GraspingUtils::getConfigPath());


	// Get the max allowed size for the message queue
	queueMaxsize = Config::get()["grasper"]["queueMaxsize"].as<unsigned int>();
	collisionMargin = Config::get()["grasper"]["collisionMargin"].as<float>();


	// Asynchronous spinning
	ros::AsyncSpinner spinner(1); // Use 2 threads
	spinner.start();


	// For some reason this MUST be declared before using the group planning capabilities, otherwise
	// collision objects won't be added to the planning interface (WTF)
	moveit::planning_interface::PlanningSceneInterface planningScene;


	// Group to plane movement for both arms
	ROS_INFO("Setting effector control");
	std::pair<std::string, std::string> effectorNames = RobotUtils::getEffectorNames(Config::get()["grasper"]["arm"].as<std::string>());
	MoveGroupPtr effector = MoveGroupPtr(new moveit::planning_interface::MoveGroup(effectorNames.first));
	effector->setEndEffector(effectorNames.second);


	// Set subscriptions
	ROS_INFO("Setting publishers/subscribers");
	ros::Publisher posePublisher = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/grasping_pose", 1);
	ros::Subscriber sub = handler.subscribe("/pr2_grasping/grasping_data", 10, graspingPointsCallback);
	ros::Timer timer = handler.createTimer(ros::Duration(1), boost::bind(timerCallback, _1, &planningScene, effector, tfListener, posePublisher));


	posearrayPublisher = handler.advertise<geometry_msgs::PoseArray>("/pr2_grasping/pose_array", 1);

	ros::waitForShutdown();
	return EXIT_SUCCESS;
}
