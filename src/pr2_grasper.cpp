/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <pr2_grasping/GraspingData.h>
#include <pr2_grasping/GraspingPoint.h>
#include <pr2_grasping/CloudLabeler.h>
#include <pr2_grasping/GazeboSetup.h>
#include <pr2_grasping/GraspEvaluator.h>
#include <pr2_grasping/GraspingGroup.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
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
#include "ClusteringUtils.hpp"
#include "GraspingUtils.hpp"
#include "RobotUtils.hpp"
#include "GazeboUtils.hpp"
#include "IO.hpp"


#define OBJECT_FAKE_AUX		"object_fake_aux"
#define ANGLE_SPLIT_NUM		4
#define ANGLE_STEP			M_PI / ANGLE_SPLIT_NUM


/***** Enumeration defining the possible states for the gripper *****/
enum GripperState
{
	STATE_IDLE,
	STATE_PERFORMING_GRASP,
	STATE_STUCK
};

/***** Global variables *****/
ros::Publisher posePub, cancelPub, scenePub, statusPub;
boost::mutex pmutex, gmutex;
std::deque<pr2_grasping::GraspingData> queue;
float collisionMargin = 0.01;
float graspPadding = 0.1;
GripperState gState = STATE_IDLE;
std::string trackedObject = "";
std::string supportObject = "";
CvSVMPtr classifier = CvSVMPtr();

/***** Debug variables *****/
ros::Publisher collisionPub, graspingPointPub;


/**************************************************/
void graspingPointsCallback(const pr2_grasping::GraspingDataConstPtr &msg_)
{
	// Prevent queue from growing endlessly
	if (queue.size() < 1)
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
std::vector<moveit_msgs::CollisionObject> genCollisions(const bool debugEnabled_,
		const geometry_msgs::PointStamped &minPt_,
		const geometry_msgs::PointStamped &maxPt_)
{
	ROS_INFO("...generating collision object");

	float colX = (minPt_.point.x + maxPt_.point.x) * 0.5;
	float colY = (minPt_.point.y + maxPt_.point.y) * 0.5;
	float colZ = (minPt_.point.z + maxPt_.point.z) * 0.5;
	geometry_msgs::Pose targetPose = GraspingUtils::genPose(colX, colY, colZ);

	if (debugEnabled_)
	{
		ROS_DEBUG("Publishing collision pose");
		geometry_msgs::PoseStamped targetPoseMsg;
		targetPoseMsg.header.frame_id = minPt_.header.frame_id;
		targetPoseMsg.pose = targetPose;
		collisionPub.publish(targetPoseMsg);
	}

	float dimX = maxPt_.point.x - minPt_.point.x + collisionMargin;
	float dimY = maxPt_.point.y - minPt_.point.y + collisionMargin;
	float dimZ = maxPt_.point.z - minPt_.point.z + collisionMargin;
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

	return collisions;
}


/**************************************************/
moveit_msgs::Grasp genGrasp(const std::string &graspId_,
							const EffectorSide &side_,
							const geometry_msgs::PoseStamped &graspingPose_,
							const std::string &targetId_,
							const std::string &supportId_)
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
	grasp.pre_grasp_approach.desired_distance = 0.1;


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
	grasp.post_grasp_retreat.desired_distance = 0.1;

	grasp.allowed_touch_objects.clear();
	grasp.allowed_touch_objects.push_back(targetId_);
	grasp.allowed_touch_objects.push_back(supportId_);
	grasp.allowed_touch_objects.push_back(prefix + "_forearm_roll_link");
	grasp.allowed_touch_objects.push_back(prefix + "_forearm_link");
	grasp.allowed_touch_objects.push_back(prefix + "_wrist_flex_link");
	grasp.allowed_touch_objects.push_back(prefix + "_wrist_roll_link");
	grasp.allowed_touch_objects.push_back(prefix + "_gripper_palm_link");
	grasp.allowed_touch_objects.push_back(prefix + "_gripper_r_finger_link");
	grasp.allowed_touch_objects.push_back(prefix + "_gripper_r_finger_tip_link");
	grasp.allowed_touch_objects.push_back(prefix + "_gripper_l_finger_link");
	grasp.allowed_touch_objects.push_back(prefix + "_gripper_l_finger_tip_link");


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
	allowedTouch.push_back(prefix + "_forearm_roll_link");
	allowedTouch.push_back(prefix + "_forearm_link");
	allowedTouch.push_back(prefix + "_wrist_flex_link");
	allowedTouch.push_back(prefix + "_wrist_roll_link");
	allowedTouch.push_back(prefix + "_gripper_palm_link");
	allowedTouch.push_back(prefix + "_gripper_r_finger_link");
	allowedTouch.push_back(prefix + "_gripper_r_finger_tip_link");
	allowedTouch.push_back(prefix + "_gripper_l_finger_link");
	allowedTouch.push_back(prefix + "_gripper_l_finger_tip_link");

	std::vector<moveit_msgs::CollisionObject> collisions;
	collisions.push_back(genCollisionObject(OBJECT_FAKE_AUX, FRAME_R_GRIPPER, geometry_msgs::Pose(), 0.25, 0.2, 0.25));
	planningScene_->addCollisionObjects(collisions);
	ros::Duration(1.0).sleep();

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


	ROS_INFO(".....release action completed");
}


/**************************************************/
void timerCallback(const ros::TimerEvent &event_)
{
	// Call the labeling service if no points are queued
	if (queue.empty())
	{
		pr2_grasping::CloudLabeler labeler;
		if (ros::service::call("/pr2_grasping/cloud_labeler", labeler))
			ROS_INFO("...labeling %s", labeler.response.result ? "SCHEDULED" : "NOT SCHEDULED");
	}
}


/**************************************************/
std::vector<std::pair<moveit_msgs::Grasp, float> > generateGrasps(const EffectorSide &side_,
		const bool debugEnabled_,
		std::vector<geometry_msgs::PoseStamped> &DEBUG_points_)
{
	// Number of grasping points processed
	static int pointIdx = 0;

	std::vector<std::pair<moveit_msgs::Grasp, float> > grasps;
	DEBUG_points_.clear();

	if (classifier)
	{
		ROS_INFO("...using classifier");
		size_t npoints = queue.front().graspingPoints.size();

		for (size_t i = 0; i < npoints; i++)
		{
			pr2_grasping::GraspingPoint point = queue.front().graspingPoints[i];

			for (int j = 0; j < ANGLE_SPLIT_NUM; j++)
			{
				int label = point.label;
				float angle = j * ANGLE_STEP;

				cv::Mat sample = cv::Mat(1, 2, CV_32FC1);
				sample.at<float>(0, 0) = label;
				sample.at<float>(0, 1) = angle;
				float distance = classifier->predict(sample, true);
				float cls = classifier->predict(sample, false);
				bool usePoint = abs(cls - 1) < 1E-8;

				ROS_DEBUG("...prediction: (%d, %.2f): %.3f / %s (%.0f)", label, angle, distance, usePoint ? "TRUE" : "FALSE", cls);

				if (usePoint)
				{
					geometry_msgs::PoseStamped graspingPose = genGraspingPose(point, angle);
					std::string id = GRASP_ID + boost::lexical_cast<std::string>(pointIdx) + "_" + boost::lexical_cast<std::string>(j);
					moveit_msgs::Grasp grasp = genGrasp(id, side_, graspingPose, OBJECT_TARGET, OBJECT_SUPPORT);

					grasps.push_back(make_pair(grasp, angle));
				}
			}
		}

		ROS_INFO("...predicted %zu grasps", grasps.size());
	}
	else
	{
		ROS_INFO("...sweeping point-angle space");

		size_t npoints = queue.front().graspingPoints.size();
		for (size_t i = 0; i < npoints; i++)
		{
			pr2_grasping::GraspingPoint point = queue.front().graspingPoints[i];

			for (int j = 0; j < ANGLE_SPLIT_NUM; j++)
			{
				// Synthesize the actual grasp
				float angle = j * ANGLE_STEP;
				geometry_msgs::PoseStamped graspingPose = genGraspingPose(point, angle);
				std::string id = GRASP_ID + boost::lexical_cast<std::string>(pointIdx) + "_" + boost::lexical_cast<std::string>(j);
				moveit_msgs::Grasp grasp = genGrasp(id, side_, graspingPose, OBJECT_TARGET, OBJECT_SUPPORT);

				grasps.push_back(make_pair(grasp, angle));

				/***** FOR DEBUG ONLY *****/
				geometry_msgs::PoseStamped gp = graspingPose;
				gp.pose.position = point.position;
				DEBUG_points_.push_back(gp);
			}

			pointIdx++;
		}
		ROS_INFO("...synthesized %zu grasps", grasps.size());
	}

	return grasps;
}


/**************************************************/
void graspingRoutine(moveit::planning_interface::PlanningSceneInterface *planningScene_,
					 MoveGroupPtr &effector_,
					 const EffectorSide &side_,
					 const bool debugEnabled_)
{
	// Number of sets of grasping points processed so far
	static int nsets = 0;

	// Process data until empty
	while (!queue.empty())
	{
		ROS_INFO("===== Processing grasping data =====");

		/*********************************************************/
		/********** STAGE 1: generate collision objects **********/
		/*********************************************************/
		std::vector<moveit_msgs::CollisionObject> collisions = genCollisions(debugEnabled_, queue.front().boundingBoxMin, queue.front().boundingBoxMax);


		/*********************************************************/
		/********** STAGE 2: generate points to grasp ************/
		/*********************************************************/
		std::vector<geometry_msgs::PoseStamped> DEBUG_points; // for debug only
		std::vector<std::pair<moveit_msgs::Grasp, float> > grasps = generateGrasps(side_, debugEnabled_, DEBUG_points);


		/*********************************************************/
		/******** STAGE 3: attempt to grasp the object ***********/
		/*********************************************************/
		size_t ngrasps = grasps.size();
		for (size_t i = 0; i < ngrasps; i++)
		{
			ROS_INFO("*** processing grasp %zu of %zu ***", i + 1, ngrasps);
			moveit_msgs::Grasp grasp = grasps[i].first;
			ROS_DEBUG("grasp id: %s", grasp.id.c_str());


			ROS_INFO("...clearing scene");
			moveit_msgs::PlanningSceneWorld cleanScene;
			scenePub.publish(cleanScene);
			ros::Duration(1).sleep();

			ROS_INFO("...adding collisions to scene");
			planningScene_->addCollisionObjects(collisions);
			ros::Duration(2).sleep();

			ROS_DEBUG("publishing grasping pose");
			posePub.publish(grasp.grasp_pose);
			if (debugEnabled_)
			{
				ROS_DEBUG("publishing grasping point");
				graspingPointPub.publish(DEBUG_points[i]);
			}

			/********** Grasp attempt **********/
			// Change gripper state
			gmutex.lock();
			gState = STATE_PERFORMING_GRASP;
			gmutex.unlock();

			// Perform the grasp
			ROS_INFO("...attempting grasp");
			int maxAttempts = 10;
			moveit::planning_interface::MoveItErrorCode code;
			for (int att = 0;
					att < maxAttempts &&
					code.val != moveit_msgs::MoveItErrorCodes::SUCCESS &&
					code.val != moveit_msgs::MoveItErrorCodes::CONTROL_FAILED ;
					att++)
			{
				ROS_INFO(".....attempt %d of %d", att + 1, maxAttempts);
				code = effector_->pick(OBJECT_TARGET, grasp);
				ROS_INFO(".....finished with code: %d (attempt %d of %d)", code.val, att + 1, maxAttempts);
			}


			/********** Evaluate result if the grasp was completed **********/
			pr2_grasping::GraspEvaluator srv;
			srv.response.result = false;

			// Skip the rest if the planning failed
			bool attemptCompleted = false;
			if (code.val == moveit_msgs::MoveItErrorCodes::SUCCESS ||
					(code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED &&
					 gState == STATE_STUCK))
			{
				attemptCompleted = true;

				// Move the support object so it doesn't obstructs the evaluation
				ROS_INFO("Setting table pose");
				geometry_msgs::Pose auxPose;
				auxPose.position.x = 4;
				while (!GazeboUtils::setModelState(supportObject, auxPose, "world"))
					ros::Duration(1).sleep();
				ROS_INFO("Table pose set");
				ros::Duration(2).sleep();


				// ROS_INFO("Removing support collision");
				// std::vector<std::string> ids;
				// ids.push_back(OBJECT_SUPPORT);
				// planningScene_->removeCollisionObjects(ids);
				// ros::Duration(2).sleep();


				// ROS_INFO("...attaching grasped object");
				// std::string prefix = (side_ == LEFT_ARM ? "l" : "r");
				// std::vector<std::string> allowedTouch;
				// allowedTouch.push_back(prefix + "_forearm_roll_link");
				// allowedTouch.push_back(prefix + "_forearm_link");
				// allowedTouch.push_back(prefix + "_wrist_flex_link");
				// allowedTouch.push_back(prefix + "_wrist_roll_link");
				// allowedTouch.push_back(prefix + "_gripper_palm_link");
				// allowedTouch.push_back(prefix + "_gripper_r_finger_link");
				// allowedTouch.push_back(prefix + "_gripper_r_finger_tip_link");
				// allowedTouch.push_back(prefix + "_gripper_l_finger_link");
				// allowedTouch.push_back(prefix + "_gripper_l_finger_tip_link");
				// allowedTouch.push_back(OBJECT_SUPPORT);
				// effector_->attachObject(OBJECT_TARGET, "", allowedTouch);


				// geometry_msgs::PoseStamped xx;
				// xx.header.frame_id = FRAME_BASE;
				// xx.pose.position.x = 0.4;
				// xx.pose.position.y = -0.4;
				// xx.pose.position.z = 0.7;
				// effector_->setPoseTarget(xx);
				// if (!RobotUtils::move(effector_, 10))
				// {
				// 	ROS_INFO("FUCK!!!");
				// }








				// Detach so the object can be 'seen'
				ROS_INFO("...detaching object for evaluation");
				effector_->detachObject(OBJECT_TARGET);
				// ROS_INFO("...attaching grasped object");
				// effector_->attachObject(OBJECT_TARGET);
				ros::Duration(0.5).sleep();

				// Call the evaluation node
				ROS_INFO("...evaluating result");
				while (!ros::service::call("/pr2_grasping/grasp_evaluator", srv))
					ros::Duration(0.5).sleep();

				ROS_INFO("...grasp attempt %s", srv.response.result ? "SUCCESSFUL" : "FAILED");


				// ROS_INFO("...detaching object for evaluation");
				// effector_->detachObject(OBJECT_TARGET);
			}
			else
				ROS_INFO("...attempt failed, skipping evaluation");


			// restore tracked state
			gmutex.lock();
			gState = STATE_IDLE;
			gmutex.unlock();


			/********** Store the result **********/
			IO::saveResults(trackedObject, attemptCompleted, srv.response.result, queue.front().graspingPoints[i].label, grasps[i].second, ANGLE_SPLIT_NUM, ANGLE_STEP, grasp, code);


			/********** Restore back everything **********/
			ROS_INFO("...detaching objects");
			effector_->detachObject(OBJECT_TARGET);
			ros::Duration(0.5).sleep();

			ROS_INFO("...removing collisions");
			std::vector<std::string> ids;
			ids.push_back(OBJECT_TARGET);
			ids.push_back(OBJECT_SUPPORT);
			planningScene_->removeCollisionObjects(ids);
			ros::Duration(0.5).sleep();

			ROS_INFO("...releasing object");
			releaseObject(effector_, planningScene_, side_);
			ros::Duration(0.5).sleep();

			ROS_INFO("...restoring setup");
			pr2_grasping::GazeboSetup setup;
			if (ros::service::call("/pr2_grasping/gazebo_setup", setup))
				ROS_INFO("...setup %s", setup.response.result ? "RESTORED" : "restore FAILED");
			// ros::Duration(2).sleep();
		}


		/*********************************************************/
		/*** STAGE 4: send a signal and remove processed data ****/
		/*********************************************************/
		// Publish the number of grasping point sets processed so far
		std_msgs::Int32 sets;
		sets.data = ++nsets;
		statusPub.publish(sets);

		// Remove the processed grasping data
		ROS_DEBUG("Removing processed grasping data");
		pmutex.lock();
		queue.pop_front();
		pmutex.unlock();
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
		cancelPub.publish(goalCancel);
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

	// Get the params
	bool debugEnabled = Config::get()["grasperDebug"].as<bool>();
	collisionMargin = Config::get()["grasper"]["collisionMargin"].as<float>();
	graspPadding = Config::get()["grasper"]["graspPadding"].as<float>();

	// Load the classifier if requested
	if (Config::get()["grasper"]["classifier"]["use"].as<bool>())
	{
		std::string location = ros::package::getPath(PACKAGE_NAME) + "/" + Config::get()["grasper"]["classifier"]["location"].as<std::string>();

		ROS_INFO("Loading classifier at %s", location.c_str());
		classifier = CvSVMPtr(new CvSVM());
		classifier->load(location.c_str());
	}


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

	/********** Set subscriptions/publishers **********/
	ROS_INFO("Setting publishers/subscribers");
	statusPub = handler.advertise<std_msgs::Int32>("/pr2_grasping/processed_sets", 1, true);
	cancelPub = handler.advertise<actionlib_msgs::GoalID>(RobotUtils::getGripperTopic(arm) + "/cancel", 1);
	posePub = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/grasping_pose", 1, true);
	scenePub = handler.advertise<moveit_msgs::PlanningSceneWorld>("/planning_scene_world", 1);

	ros::Subscriber pointsSub = handler.subscribe("/pr2_grasping/grasping_data", 1, graspingPointsCallback);
	ros::Subscriber stuckSub = handler.subscribe("/pr2_grasping/gripper_action_stuck", 1, gripperStuckCallback);

	if (debugEnabled)
	{
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
			ros::console::notifyLoggerLevelsChanged();

		collisionPub = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/debug_collision_pose", 1, true);
		graspingPointPub = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/debug_grasping_point", 1, true);
	}


	/********** Publish initial number of processed sets **********/
	std_msgs::Int32 sets;
	sets.data = 0;
	statusPub.publish(sets);


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
		ros::Duration(0.5).sleep();
	}
	trackedObject = srv.response.trackedObject;
	supportObject = srv.response.supportObject;


	/********** Start periodic check/ask for new grasping points **********/
	ros::Timer timer = handler.createTimer(ros::Duration(2), timerCallback);

	// Spin at 10 Hz
	ros::Rate r(10);
	while (ros::ok())
	{
		graspingRoutine(&planningScene, effector, side, debugEnabled);
		r.sleep();
	}

	ros::shutdown();
	return EXIT_SUCCESS;
}
