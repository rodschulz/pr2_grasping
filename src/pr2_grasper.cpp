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


#define TARGET_OBJECT		"target_object"

// Pointer to a move group interface
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

/***** Global variables *****/
ros::Publisher posePublisher;
std::deque<pr2_grasping::GraspingData> queue;
unsigned int queueMaxsize = 5;
boost::mutex mutex;
float collisionMargin = 0.01;

/***** Debug variables *****/
ros::Publisher collisionPosePublisher;


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
void timerCallback(const ros::TimerEvent &event_,
				   moveit::planning_interface::PlanningSceneInterface *planningScene_,
				   MoveGroupPtr &effector_,
				   tf::TransformListener *tfListener_,
				   const bool debugEnabled_)
{
	while (!queue.empty())
	{
		ROS_INFO("Processing grasping points");

		/********** STAGE 1: add collisions to the planning scene **********/
		ROS_INFO("...generating collision object");
		geometry_msgs::PointStamped minPt = queue.front().boundingBoxMin;
		geometry_msgs::PointStamped maxPt = queue.front().boundingBoxMax;

		// geometry_msgs::Point minTf = GraspingUtils::transformPoint(tfListener_, FRAME_BASE, minPt.header.frame_id, minPt.point);
		// geometry_msgs::Point maxTf = GraspingUtils::transformPoint(tfListener_, FRAME_BASE, maxPt.header.frame_id, maxPt.point);

		// float dimX = maxTf.x - minTf.x + collisionMargin;
		// float dimY = maxTf.y - minTf.y + collisionMargin;
		// float dimZ = maxTf.z - minTf.z + collisionMargin;

		float dimX = maxPt.point.x - minPt.point.x + collisionMargin;
		float dimY = maxPt.point.y - minPt.point.y + collisionMargin;
		float dimZ = maxPt.point.z - minPt.point.z + collisionMargin;

		// geometry_msgs::Pose collisionPose = GraspingUtils::genPose(minTf.x - dimX * 0.5, minTf.y + dimY * 0.5, minTf.z + dimZ * 0.5);
		geometry_msgs::Pose collisionPose = GraspingUtils::genPose(minPt.point.x, minPt.point.y, minPt.point.z);
		moveit_msgs::CollisionObject targetCollision = genCollisionObject(TARGET_OBJECT, FRAME_BASE, collisionPose, dimX, dimY, dimZ);

		if (debugEnabled_)
		{
			geometry_msgs::PoseStamped collisionPoseMsg;
			collisionPoseMsg.header.frame_id = FRAME_BASE;
			collisionPoseMsg.pose = collisionPose;
			collisionPosePublisher.publish(collisionPoseMsg);
		}

		std::vector<moveit_msgs::CollisionObject> collisions;
		collisions.push_back(targetCollision);

		ROS_INFO("...adding collisions to scene");
		planningScene_->addCollisionObjects(collisions);
		ros::Duration(2.0).sleep();


		/********** STAGE 2: generate grasp **********/
		ROS_INFO("...generating grasp");

		// geometry_msgs::PoseStamped graspPose;
		// graspPose.header = targetPose_.header;
		// graspPose.pose = GraspingUtils::genPose(targetPose_.pose.position.x,
		// targetPose_.pose.position.y - 0.18,
		// targetPose_.pose.position.z,
		// DEG2RAD(90), 0, 0, 1);


		// ROS_INFO("...publishing grasping pose");
		// posePublisher_.publish(graspPose);


		// moveit_msgs::Grasp grasp;
		// grasp.id = "TARGET_GRASP";
		// grasp.grasp_pose = graspPose;

		// grasp.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
		// grasp.pre_grasp_approach.direction.vector.x = 1;
		// grasp.pre_grasp_approach.direction.vector.y = 0;
		// grasp.pre_grasp_approach.direction.vector.z = 0;
		// grasp.pre_grasp_approach.min_distance = 0.05;
		// grasp.pre_grasp_approach.desired_distance = 0.18;


		// grasp.pre_grasp_posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
		// grasp.pre_grasp_posture.points.resize(1);
		// grasp.pre_grasp_posture.points[0].positions.resize(1);
		// grasp.pre_grasp_posture.points[0].positions[0] = 1;
		// grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(45.0);


		// grasp.grasp_posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
		// grasp.grasp_posture.points.resize(1);
		// grasp.grasp_posture.points[0].positions.resize(1);
		// grasp.grasp_posture.points[0].positions[0] = 0;
		// grasp.grasp_posture.points[0].time_from_start = ros::Duration(45.0);


		// grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint";
		// grasp.post_grasp_retreat.direction.vector.x = 0;
		// grasp.post_grasp_retreat.direction.vector.y = 0;
		// grasp.post_grasp_retreat.direction.vector.z = 1;
		// grasp.post_grasp_retreat.min_distance = 0.08;
		// grasp.post_grasp_retreat.desired_distance = 0.3;

		// grasp.allowed_touch_objects.clear();
		// grasp.allowed_touch_objects.push_back(TARGET_OBJECT);
		// grasp.allowed_touch_objects.push_back(SUPPORT_OBJECT);



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
	bool debugEnabled = Config::get()["grasperDebug"].as<bool>();
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
	posePublisher = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/grasping_pose", 1);
	ros::Subscriber sub = handler.subscribe("/pr2_grasping/grasping_data", 10, graspingPointsCallback);
	ros::Timer timer = handler.createTimer(ros::Duration(1), boost::bind(timerCallback, _1, &planningScene, effector, tfListener, debugEnabled));

	if (debugEnabled)
	{
		collisionPosePublisher = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/debug_collision_pose", 1);
	}


	ros::waitForShutdown();
	return EXIT_SUCCESS;
}
