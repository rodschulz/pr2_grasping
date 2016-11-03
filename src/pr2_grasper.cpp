/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pr2_grasping/GraspingData.h>
#include <pr2_grasping/GraspingPoint.h>
#include <pr2_grasping/CloudLabeler.h>
#include <pr2_grasping/GazeboSetup.h>
#include <pr2_grasping/GraspEvaluator.h>
#include <pr2_grasping/GraspingGroup.h>
#include <std_msgs/Bool.h>
#include <actionlib_msgs/GoalID.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>
#include <deque>
#include <iostream>
#include <fstream>
#include "Config.hpp"
#include "GraspingUtils.hpp"
#include "RobotUtils.hpp"
#include "IO.hpp"


#define OBJECT_FAKE_AUX		"object_fake_aux"
#define ANGLE_SPLIT_NUM		4
#define ANGLE_STEP			M_PI / ANGLE_SPLIT_NUM


enum GripperState
{
	STATE_IDLE,
	STATE_PERFORMING_GRASP,
	STATE_STUCK
};

/***** Global variables *****/
ros::Publisher posePublisher, cancelPublisher;
boost::mutex pmutex, gmutex;
std::deque<pr2_grasping::GraspingData> queue;
unsigned int queueMaxsize = 5;
float collisionMargin = 0.01;
float graspPadding = 0.1;
GripperState gState = STATE_IDLE;
std::vector<std::string> objects;

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
		pmutex.lock();
		queue.push_back(*msg_);
		pmutex.unlock();
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
geometry_msgs::PoseStamped genGraspingPose(const pr2_grasping::GraspingPoint &point_,
		const float angle_)
{
	Eigen::Vector3f p(point_.position.x, point_.position.y, point_.position.z);
	Eigen::Vector3f n(point_.normal.x, point_.normal.y, point_.normal.z);
	n.normalize();

	// Generate a line going through the point using the direction given by the normal
	Eigen::ParametrizedLine<float, 3> line(p, n);

	// Generate a point moving along the line and going outside the object
	Eigen::Vector3f g = line.pointAt(graspPadding);

	geometry_msgs::PoseStamped graspingPose;
	graspingPose.header = point_.header;
	graspingPose.pose.position.x = g.x();
	graspingPose.pose.position.y = g.y();
	graspingPose.pose.position.z = g.z();

	// Generate a quaternion for the orientation according to the grasping point's normal
	Eigen::Quaternionf normalOrientation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(1, 0, 0), -n);

	// Generate a quaternion for the rotation according to the given angle
	Eigen::Vector3f axis(1, 0, 0);
	Eigen::Quaternionf angleOrientation = Eigen::Quaternionf(cos(angle_ / 2),
										  axis.x() * sin(angle_ / 2),
										  axis.y() * sin(angle_ / 2),
										  axis.z() * sin(angle_ / 2));

	// Set the final orientation
	Eigen::Quaternionf orientation = normalOrientation * angleOrientation;
	graspingPose.pose.orientation.w = orientation.w();
	graspingPose.pose.orientation.x = orientation.x();
	graspingPose.pose.orientation.y = orientation.y();
	graspingPose.pose.orientation.z = orientation.z();

	return graspingPose;
}


/**************************************************/
moveit_msgs::Grasp genGrasp(const std::string &graspId_,
							const EffectorSide &side_,
							const geometry_msgs::PoseStamped &graspingPose_,
							const std::string &objectTarget_,
							const std::string &objectSupport_)
{
	moveit_msgs::Grasp grasp;
	grasp.id = graspId_;
	grasp.grasp_pose = graspingPose_;

	std::string prefix = (side_ == LEFT_ARM ? "l" : "r");

	grasp.pre_grasp_approach.direction.header.frame_id = prefix + "_wrist_roll_link";
	grasp.pre_grasp_approach.direction.vector.x = 1;
	grasp.pre_grasp_approach.direction.vector.y = 0;
	grasp.pre_grasp_approach.direction.vector.z = 0;
	grasp.pre_grasp_approach.min_distance = 0.05;
	grasp.pre_grasp_approach.desired_distance = 0.15;


	grasp.pre_grasp_posture.joint_names.resize(1, prefix + "_gripper_motor_screw_joint");
	grasp.pre_grasp_posture.points.resize(1);
	grasp.pre_grasp_posture.points[0].positions.resize(1);
	grasp.pre_grasp_posture.points[0].positions[0] = 1;
	grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(45.0);


	grasp.grasp_posture.joint_names.resize(1, prefix + "_gripper_motor_screw_joint");
	grasp.grasp_posture.points.resize(1);
	grasp.grasp_posture.points[0].positions.resize(1);
	grasp.grasp_posture.points[0].positions[0] = 0;
	grasp.grasp_posture.points[0].time_from_start = ros::Duration(45.0);


	grasp.post_grasp_retreat.direction.header.frame_id = FRAME_BASE;
	grasp.post_grasp_retreat.direction.vector.x = 0;
	grasp.post_grasp_retreat.direction.vector.y = 0;
	grasp.post_grasp_retreat.direction.vector.z = 1;
	grasp.post_grasp_retreat.min_distance = 0.05;
	grasp.post_grasp_retreat.desired_distance = 0.2;

	grasp.allowed_touch_objects.clear();
	grasp.allowed_touch_objects.push_back(objectTarget_);
	grasp.allowed_touch_objects.push_back(objectSupport_);

	return grasp;
}


/**************************************************/
void releaseObject(MoveGroupPtr &effector_,
				   moveit::planning_interface::PlanningSceneInterface *planningScene_,
				   const EffectorSide &side_)
{
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME ".actionlib", ros::console::levels::Info))
		ros::console::notifyLoggerLevelsChanged();

	// Stop any previous movement
	effector_->stop();

	ROS_INFO(".....setting aux collision");
	std::string prefix = (side_ == LEFT_ARM ? "l" : "r");
	std::vector<std::string> allowedTouch;
	allowedTouch.push_back(prefix + "_wrist_roll_link");
	allowedTouch.push_back(prefix + "_gripper_palm_link");
	allowedTouch.push_back(prefix + "_gripper_r_finger_link");
	allowedTouch.push_back(prefix + "_gripper_r_finger_tip_link");
	allowedTouch.push_back(prefix + "_gripper_l_finger_link");
	allowedTouch.push_back(prefix + "_gripper_l_finger_tip_link");

	std::vector<moveit_msgs::CollisionObject> collisions;
	collisions.push_back(genCollisionObject(OBJECT_FAKE_AUX, FRAME_R_GRIPPER, geometry_msgs::Pose(), 0.25, 0.2, 0.25));
	planningScene_->addCollisionObjects(collisions);
	effector_->attachObject(OBJECT_FAKE_AUX, "", allowedTouch);
	ros::Duration(0.5).sleep();


	// Move the effector to an adequate pose to release the object
	ROS_INFO(".....moving effector to release pose");
	Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(1, 0, 0), Eigen::Vector3f(0, 0, -1));
	geometry_msgs::PoseStamped current;
	current.header.frame_id = FRAME_BASE;
	current.pose.position.x = 0.35;
	current.pose.position.y = -0.5;
	current.pose.position.z = 0.9;
	current.pose.orientation.w = rotation.w();
	current.pose.orientation.x = rotation.x();
	current.pose.orientation.y = rotation.y();
	current.pose.orientation.z = rotation.z();
	effector_->setPoseTarget(current);

	if (!RobotUtils::move(effector_, 25))
		ROS_WARN("Unable to move gripper to release pose. Attempting release 'as is'");


	ROS_INFO(".....detaching aux collision");
	effector_->detachObject(OBJECT_FAKE_AUX);
	ros::Duration(0.5).sleep();

	ROS_INFO(".....removing aux collision");
	std::vector<std::string> ids;
	ids.push_back(OBJECT_FAKE_AUX);
	planningScene_->removeCollisionObjects(ids);
	ros::Duration(0.5).sleep();


	ROS_INFO(".....opening gripper");
	RobotUtils::moveGripper(effector_->getName(), 1);


	ROS_INFO("Release action completed");
}


/**************************************************/
void saveResult(const moveit_msgs::Grasp &grasp_,
				const moveit::planning_interface::MoveItErrorCode &errCode_,
				const bool success_,
				const std::vector<std::string> &objects_)
{
	std::string filename = GraspingUtils::getOutputPath() +
						   "grasp_" +
						   GraspingUtils::getTimestamp("%d-%m-%Y_%H:%M:%S") + ".yaml";

	ROS_DEBUG_STREAM("Saving to: " << filename);

	// generate a YAML file with the results
	YAML::Emitter emitter;
	emitter << YAML::BeginMap
			<< YAML::Key << "success" << YAML::Value << success_
			<< YAML::Key << "code" << YAML::Value << errCode_.val
			<< YAML::Key << "objects" << YAML::Value <<  objects_
			<< YAML::Key << "grasp" << YAML::Value << grasp_
			<< YAML::EndMap;

	std::ofstream output;
	output.open(filename.c_str(), std::fstream::out);
	output << emitter.c_str();
	output.close();

	ROS_INFO("...saved");
}


/**************************************************/
void timerCallback(const ros::TimerEvent &event_,
				   moveit::planning_interface::PlanningSceneInterface *planningScene_,
				   MoveGroupPtr &effector_,
				   const EffectorSide &side_,
				   const bool debugEnabled_)
{
	while (!queue.empty())
	{
		ROS_INFO("===== Processing grasping data =====");
		static int graspIdx = 0;


		/*********************************************************/
		/********** STAGE 1: generate collision objects **********/
		/*********************************************************/
		ROS_INFO("...generating collision object");
		geometry_msgs::PointStamped minPt = queue.front().boundingBoxMin;
		geometry_msgs::PointStamped maxPt = queue.front().boundingBoxMax;

		float colX = (minPt.point.x + maxPt.point.x) * 0.5;
		float colY = (minPt.point.y + maxPt.point.y) * 0.5;
		float colZ = (minPt.point.z + maxPt.point.z) * 0.5;
		geometry_msgs::Pose targetPose = GraspingUtils::genPose(colX, colY, colZ);

		if (debugEnabled_)
		{
			ROS_DEBUG("Publishing collision pose");
			geometry_msgs::PoseStamped targetPoseMsg;
			targetPoseMsg.header.frame_id = minPt.header.frame_id;
			targetPoseMsg.pose = targetPose;
			collisionPosePublisher.publish(targetPoseMsg);
		}

		float dimX = maxPt.point.x - minPt.point.x + collisionMargin;
		float dimY = maxPt.point.y - minPt.point.y + collisionMargin;
		float dimZ = maxPt.point.z - minPt.point.z + collisionMargin;
		moveit_msgs::CollisionObject targetCollision = genCollisionObject(OBJECT_TARGET, FRAME_BASE, targetPose, dimX, dimY, dimZ);

		// Generate collision for the support object
		dimX = 1.5;
		dimY = 0.82;
		dimZ = 0.46;
		geometry_msgs::Pose supportPose = GraspingUtils::genPose(1.35, 0, 0.235);
		moveit_msgs::CollisionObject supportCollision = genCollisionObject(OBJECT_SUPPORT, FRAME_BASE, supportPose, dimX, dimY, dimZ);


		std::vector<moveit_msgs::CollisionObject> collisions;
		collisions.push_back(targetCollision);
		collisions.push_back(supportCollision);


		/*********************************************************/
		/********** STAGE 2: test every grasping points **********/
		/*********************************************************/
		size_t npoints = queue.front().graspingPoints.size();
		for (size_t i = 0; i < npoints; i++)
		{
			ROS_INFO("*** processing point %zu of %zu ***", i + 1, npoints);


			for (int j = 0; j < ANGLE_SPLIT_NUM; j++)
			{
				ROS_INFO("### using angle %d of %d ###", j + 1, ANGLE_SPLIT_NUM);
				float angle = j + ANGLE_STEP;


				/********** STAGE 2.1: add collisions to the scene **********/
				ROS_INFO("...adding collisions to scene");
				planningScene_->addCollisionObjects(collisions);
				ros::Duration(0.5).sleep();


				/********** STAGE 2.2: generate grasping pose **********/
				ROS_DEBUG("Generating grasping point");
				pr2_grasping::GraspingPoint point = queue.front().graspingPoints[i];
				geometry_msgs::PoseStamped graspingPose = genGraspingPose(point, angle);

				ROS_DEBUG("Publishing grasping pose");
				posePublisher.publish(graspingPose);

				if (debugEnabled_)
				{
					ROS_DEBUG("Publishing grasping point");
					geometry_msgs::PoseStamped pointMsg = graspingPose;
					pointMsg.pose.position = point.position;
					graspingPointPublisher.publish(pointMsg);
				}


				/********** STAGE 2.3: synthesize grasp **********/
				ROS_INFO("...synthesizing grasp");
				std::string id = GRASP_ID + boost::lexical_cast<std::string>(graspIdx) + "_" + boost::lexical_cast<std::string>(j);
				moveit_msgs::Grasp grasp = genGrasp(id, side_, graspingPose, OBJECT_TARGET, OBJECT_SUPPORT);


				/********** STAGE 2.4: attempt grasp **********/
				ROS_INFO("...attempting grasp");
				std::vector<moveit_msgs::Grasp> grasps;
				grasps.push_back(grasp);

				// Change gripper state
				gmutex.lock();
				gState = STATE_PERFORMING_GRASP;
				gmutex.unlock();

				// Perform the grasp
				int maxAttempts = 10;
				int counter = 0;
				moveit::planning_interface::MoveItErrorCode code;
				while (code.val != moveit_msgs::MoveItErrorCodes::SUCCESS &&
						code.val != moveit_msgs::MoveItErrorCodes::CONTROL_FAILED &&
						counter++ < maxAttempts)
				{
					code = effector_->pick(OBJECT_TARGET, grasps);
					ROS_INFO(".....finished with code: %d (attempt %d of %d)", code.val, counter, maxAttempts);
					ros::Duration(1).sleep();
				}


				// Skip the rest if the planning failed
				if (code.val == moveit_msgs::MoveItErrorCodes::SUCCESS ||
						(code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED && gState == STATE_STUCK))
				{
					// Detach so the object can be 'seen'
					ROS_INFO("...detaching object for evaluation");
					effector_->detachObject(OBJECT_TARGET);
					ros::Duration(0.5).sleep();


					/********** STAGE 2.5: check the grasping attempt result **********/
					ROS_INFO("...evaluating result");
					pr2_grasping::GraspEvaluator srv;
					while (!ros::service::call("/pr2_grasping/grasp_evaluator", srv))
						ros::Duration(0.5).sleep();


					// Store the result of the grasping attempt
					saveResult(grasp, code, srv.response.result, objects);
					ROS_INFO("...grasp attempt %s", srv.response.result ? "SUCCESSFUL" : "FAILED");
					ros::Duration(0.5).sleep();


					/********** STAGE 2.6: release the object **********/
					// This MUST be done before restoring, otherwise the robot is also moved
					ROS_INFO("...releasing object");
					releaseObject(effector_, planningScene_, side_);
					ros::Duration(0.5).sleep();
				}
				else
				{
					ROS_INFO("...attempt failed, skipping evaluation");
					ros::Duration(0.5).sleep();// little wait to be able to read the message
				}


				/********** STAGE 2.7: remove collision objects **********/
				ROS_INFO("...detaching collision objects");
				effector_->detachObject(OBJECT_TARGET);
				ros::Duration(0.5).sleep();

				ROS_INFO("...removing collision objects");
				std::vector<std::string> ids;
				ids.push_back(OBJECT_TARGET);
				ids.push_back(OBJECT_SUPPORT);
				planningScene_->removeCollisionObjects(ids);
				ros::Duration(0.5).sleep();


				/********** STAGE 2.8: restore the testing setup **********/
				ROS_INFO("...restoring setup");
				pr2_grasping::GazeboSetup setup;
				if (ros::service::call("/pr2_grasping/gazebo_setup", setup))
					ROS_INFO("...setup %s", setup.response.result ? "RESTORED" : "restore FAILED");
				ros::Duration(0.5).sleep();

				ROS_INFO("### angle %d of %d processed ###", j + 1, ANGLE_SPLIT_NUM);
			}

			ROS_INFO("*** point %zu of %zu processed ***", i + 1, npoints);
		}

		// Remove the processed grasping data
		ROS_DEBUG("Removing processed grasping data");
		pmutex.lock();
		queue.pop_front();
		pmutex.unlock();
	}

	// Call the labeling service if no points are queued
	if (queue.empty())
	{
		pr2_grasping::CloudLabeler labeler;
		if (ros::service::call("/pr2_grasping/cloud_labeler", labeler))
			ROS_INFO("...labeling %s", labeler.response.result ? "SCHEDULED" : "NOT SCHEDULED");
	}
}


/**************************************************/
bool queryName(pr2_grasping::GraspingGroup::Request  &request_,
			   pr2_grasping::GraspingGroup::Response &response_,
			   const MoveGroupPtr effector_)
{
	if (effector_.get() != NULL)
	{
		response_.groupName = effector_->getName();
		response_.result = true;
	}
	else
	{
		response_.groupName = "";
		response_.result = false;
	}
	return true;
}


/**************************************************/
void gripperStuckCallback(const std_msgs::Bool &msg_)
{
	if (gState == STATE_PERFORMING_GRASP)
	{
		ROS_DEBUG("Gripper stuck");
		gmutex.lock();
		gState = STATE_STUCK;
		gmutex.unlock();

		ROS_DEBUG("Canceling gripper action");
		actionlib_msgs::GoalID goalCancel;
		cancelPublisher.publish(goalCancel);
	}
}


/**************************************************/
int main(int _argn, char **_argv)
{
	ros::init(_argn, _argv, "pr2_grasper");
	ros::NodeHandle handler;
	moveit::planning_interface::PlanningSceneInterface planningScene;


	/********** Start spinning **********/
	ros::AsyncSpinner spinner(3);
	spinner.start();


	/********** Load the node's configuration **********/
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(GraspingUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + GraspingUtils::getConfigPath());

	// Get the max allowed size for the message queue
	bool debugEnabled = Config::get()["grasperDebug"].as<bool>();
	queueMaxsize = Config::get()["grasper"]["queueMaxsize"].as<unsigned int>();
	collisionMargin = Config::get()["grasper"]["collisionMargin"].as<float>();
	graspPadding = Config::get()["grasper"]["graspPadding"].as<float>();


	/********** Setup control group **********/
	ROS_INFO("Setting effector control");
	std::string arm = Config::get()["grasper"]["arm"].as<std::string>();
	EffectorSide side = RobotUtils::getEffectorSide(arm);
	std::pair<std::string, std::string> effectorNames = RobotUtils::getEffectorNames(arm);
	MoveGroupPtr effector = MoveGroupPtr(new moveit::planning_interface::MoveGroup(effectorNames.first));
	effector->setEndEffector(effectorNames.second);
	effector->setPlannerId("RRTConnectkConfigDefault");
	effector->allowReplanning(true);


	/********** Set services **********/
	ROS_INFO("Starting name query service");
	ros::ServiceServer nameService = handler.advertiseService<pr2_grasping::GraspingGroup::Request, pr2_grasping::GraspingGroup::Response>("/pr2_grasping/effector_name", boost::bind(queryName, _1, _2, effector));


	/********** Setup the robot and environment **********/
	std::string serviceName = "/pr2_grasping/gazebo_setup";
	while (!ros::service::waitForService(serviceName, ros::Duration(1)))
		ros::Duration(1.0).sleep();

	ROS_INFO("Calling setup service");
	pr2_grasping::GazeboSetup srv;
	srv.response.result = false;
	while (!srv.response.result)
	{
		if (ros::service::call(serviceName, srv))
			ROS_INFO("Setup %s", srv.response.result ? "SUCCEEDED" : "FAILED, retrying...");
		ros::Duration(1.0).sleep();
	}
	objects = srv.response.objects;


	/********** Set subscriptions/publishers **********/
	ROS_INFO("Setting publishers/subscribers");
	cancelPublisher = handler.advertise<actionlib_msgs::GoalID>(RobotUtils::getGripperTopic(arm) + "/cancel", 1);
	posePublisher = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/grasping_pose", 1, true);

	ros::Subscriber pointsSub = handler.subscribe("/pr2_grasping/grasping_data", 10, graspingPointsCallback);
	ros::Subscriber stuckSub = handler.subscribe("/pr2_grasping/gripper_action_stuck", 1, gripperStuckCallback);

	ros::Timer timer = handler.createTimer(ros::Duration(2), boost::bind(timerCallback, _1, &planningScene, effector, side, debugEnabled));

	if (debugEnabled)
	{
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
			ros::console::notifyLoggerLevelsChanged();

		collisionPosePublisher = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/debug_collision_pose", 1, true);
		graspingPointPublisher = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/debug_grasping_point", 1, true);
	}


	ros::waitForShutdown();
	return EXIT_SUCCESS;
}
