/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pr2_grasping/GraspingData.h>
#include <pr2_grasping/GraspingPoint.h>
#include <pr2_grasping/GazeboSetup.h>
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
#define GRASP_ID			"target_grasp"

// Pointer to a move group interface
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

/***** Global variables *****/
ros::Publisher posePublisher;
std::deque<pr2_grasping::GraspingData> queue;
unsigned int queueMaxsize = 5;
boost::mutex mutex;
float collisionMargin = 0.01;
float graspPadding = 0.1;

/***** Debug variables *****/
ros::Publisher collisionPosePublisher, graspingPointPublisher;


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
geometry_msgs::PoseStamped genGraspingPose(const pr2_grasping::GraspingPoint &point_)
{
	Eigen::Vector3f p(point_.position.x, point_.position.y, point_.position.z);
	Eigen::Vector3f n(point_.normal.x, point_.normal.y, point_.normal.z);
	n.normalize();

	// Generate a line going through the point using the direction given by the normal
	Eigen::ParametrizedLine<float, 3> line(p, n);

	// Generate a point moving along the line and going outside the object
	Eigen::Vector3f g = line.pointAt(graspPadding);

	geometry_msgs::PoseStamped pose;
	pose.header = point_.header;
	pose.pose = GraspingUtils::genPose(g.x(), g.y(), g.z(), DEG2RAD(0), n.x(), n.y(), n.z());

	return pose;
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
		ROS_INFO("Processing grasping data");

		/********** STAGE 1: add collisions to the planning scene **********/
		ROS_INFO("...generating collision object");
		geometry_msgs::PointStamped minPt = queue.front().boundingBoxMin;
		geometry_msgs::PointStamped maxPt = queue.front().boundingBoxMax;

		float colX = (minPt.point.x + maxPt.point.x) * 0.5;
		float colY = (minPt.point.y + maxPt.point.y) * 0.5;
		float colZ = (minPt.point.z + maxPt.point.z) * 0.5;
		geometry_msgs::Pose collisionPose = GraspingUtils::genPose(colX, colY, colZ);

		if (debugEnabled_)
		{
			ROS_DEBUG("Publishing collision pose");
			geometry_msgs::PoseStamped collisionPoseMsg;
			collisionPoseMsg.header.frame_id = minPt.header.frame_id;
			collisionPoseMsg.pose = collisionPose;
			collisionPosePublisher.publish(collisionPoseMsg);
		}

		float dimX = maxPt.point.x - minPt.point.x + collisionMargin;
		float dimY = maxPt.point.y - minPt.point.y + collisionMargin;
		float dimZ = maxPt.point.z - minPt.point.z + collisionMargin;
		moveit_msgs::CollisionObject targetCollision = genCollisionObject(TARGET_OBJECT, FRAME_BASE, collisionPose, dimX, dimY, dimZ);

		std::vector<moveit_msgs::CollisionObject> collisions;
		collisions.push_back(targetCollision);


		/********** STAGE 2: generate grasp **********/
		size_t npoints = queue.front().graspingPoints.size();
		for (size_t i = 0; i < npoints; i++)
		{
			ROS_INFO("*** processing point %zu of %zu ***", i + 1, npoints);

			ROS_INFO("...adding collisions to scene");
			planningScene_->addCollisionObjects(collisions);
			ros::Duration(2.0).sleep();

			ROS_DEBUG("Generating grasping point");
			pr2_grasping::GraspingPoint point = queue.front().graspingPoints[i];
			geometry_msgs::PoseStamped graspingPose = genGraspingPose(point);

			if (debugEnabled_)
			{
				ROS_DEBUG("Publishing grasping point");
				geometry_msgs::PointStamped pointMsg;
				pointMsg.header = point.header;
				pointMsg.point = point.position;
				graspingPointPublisher.publish(pointMsg);
			}


			ROS_DEBUG("Publishing grasping pose");
			posePublisher.publish(graspingPose);


			ROS_INFO("...synthesizing grasp");
			moveit_msgs::Grasp grasp;
			grasp.id = GRASP_ID;
			grasp.grasp_pose = graspingPose;

			grasp.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
			grasp.pre_grasp_approach.direction.vector.x = 1;
			grasp.pre_grasp_approach.direction.vector.y = 0;
			grasp.pre_grasp_approach.direction.vector.z = 0;
			grasp.pre_grasp_approach.min_distance = 0.05;
			grasp.pre_grasp_approach.desired_distance = 0.18;


			grasp.pre_grasp_posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
			grasp.pre_grasp_posture.points.resize(1);
			grasp.pre_grasp_posture.points[0].positions.resize(1);
			grasp.pre_grasp_posture.points[0].positions[0] = 1;
			grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(45.0);


			grasp.grasp_posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
			grasp.grasp_posture.points.resize(1);
			grasp.grasp_posture.points[0].positions.resize(1);
			grasp.grasp_posture.points[0].positions[0] = 0;
			grasp.grasp_posture.points[0].time_from_start = ros::Duration(45.0);


			grasp.post_grasp_retreat.direction.header.frame_id = FRAME_BASE;
			grasp.post_grasp_retreat.direction.vector.x = 0;
			grasp.post_grasp_retreat.direction.vector.y = 0;
			grasp.post_grasp_retreat.direction.vector.z = 1;
			grasp.post_grasp_retreat.min_distance = 0.08;
			grasp.post_grasp_retreat.desired_distance = 0.3;

			grasp.allowed_touch_objects.clear();
			grasp.allowed_touch_objects.push_back(TARGET_OBJECT);



			ROS_INFO("...attempting grasp");
			std::vector<moveit_msgs::Grasp> grasps;
			grasps.push_back(grasp);
			moveit::planning_interface::MoveItErrorCode errCode = effector_->pick(TARGET_OBJECT, grasps);
			ros::Duration(1).sleep();
			ROS_INFO("...grasp finished (error code: %d)", errCode.val);


			// Move the object
			ROS_INFO("...moving object");
			effector_->setPoseTarget(GraspingUtils::genPose(0.15, -0.5, 0.3, DEG2RAD(90), 0, 1, 0));
			effector_->move();
			ros::Duration(1).sleep();


			ROS_INFO("...detaching collision objects");
			effector_->detachObject(TARGET_OBJECT);
			ros::Duration(1.0).sleep();


			ROS_INFO("...removing collision objects");
			std::vector<std::string> ids;
			ids.push_back(TARGET_OBJECT);
			planningScene_->removeCollisionObjects(ids);
			ros::Duration(1.0).sleep();

			ROS_INFO("*** point processing finished ***");
		}

		// Remove the processed grasping data
		ROS_DEBUG("Removing processed grasping data");
		queue.pop_front();
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
	graspPadding = Config::get()["grasper"]["graspPadding"].as<float>();



	ROS_INFO("Calling setup service");
	ros::ServiceClient setupService = handler.serviceClient<pr2_grasping::GazeboSetup>("/pr2_grasping/gazebo_setup");
	pr2_grasping::GazeboSetup srv;
	if (setupService.call(srv))
		ROS_INFO("Setup succeeded: %s", srv.response.result ? "TRUE" : "FALSE");
	ROS_INFO("After setup service call");




	// For some reason this MUST be declared before using the group planning capabilities, otherwise
	// collision objects won't be added to the planning interface (WTF)
	moveit::planning_interface::PlanningSceneInterface planningScene;


	// Group to plane movement for both arms
	ROS_INFO("Setting effector control");
	std::pair<std::string, std::string> effectorNames = RobotUtils::getEffectorNames(Config::get()["grasper"]["arm"].as<std::string>());
	MoveGroupPtr effector = MoveGroupPtr(new moveit::planning_interface::MoveGroup(effectorNames.first));
	effector->setEndEffector(effectorNames.second);
	effector->setPlannerId("RRTConnectkConfigDefault");


	// Set subscriptions
	ROS_INFO("Setting publishers/subscribers");
	posePublisher = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/grasping_pose", 1, true);
	ros::Subscriber sub = handler.subscribe("/pr2_grasping/grasping_data", 10, graspingPointsCallback);
	ros::Timer timer = handler.createTimer(ros::Duration(1), boost::bind(timerCallback, _1, &planningScene, effector, tfListener, debugEnabled));

	if (debugEnabled)
	{
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
			ros::console::notifyLoggerLevelsChanged();

		collisionPosePublisher = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/debug_collision_pose", 1, true);
		graspingPointPublisher = handler.advertise<geometry_msgs::PointStamped>("/pr2_grasping/debug_grasping_point", 1, true);
	}


	// Asynchronous spinning
	ros::AsyncSpinner spinner(2); // Use 2 threads
	spinner.start();
	ros::waitForShutdown();


	return EXIT_SUCCESS;
}
