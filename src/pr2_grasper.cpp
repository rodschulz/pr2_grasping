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
#include <pr2_grasping/DescriptorCalc.h>
#include <pr2_grasping/GazeboSetup.h>
#include <pr2_grasping/GraspEvaluator.h>
#include <pr2_grasping/GraspingGroup.h>
#include <pr2_grasping/ExperimentId.h>
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
#include "PkgUtils.hpp"
#include "RobotUtils.hpp"
#include "GazeboUtils.hpp"
#include "IO.hpp"


#define OBJECT_FAKE_AUX		"object_fake_aux"


/***** Enumeration defining the possible states for the gripper *****/
enum GripperState
{
	STATE_IDLE,
	STATE_PERFORMING_GRASP,
	STATE_STUCK
};

/***** Structure holding the data relative to one grasp *****/
struct GraspData
{
	moveit_msgs::Grasp grasp;
	pr2_grasping::DescriptorCalc::Response descriptor;
	float angle;
	int label;

	GraspData(const moveit_msgs::Grasp &grasp_,
			  const pr2_grasping::DescriptorCalc::Response &descriptor_,
			  const float angle_,
			  const int label_)
	{
		grasp = grasp_;
		descriptor = descriptor_;
		angle = angle_;
		label = label_;
	}

	GraspData(const GraspData &other_)
	{
		grasp = other_.grasp;
		descriptor = other_.descriptor;
		angle = other_.angle;
		label = other_.label;
	}
};

/***** Auxiliary structure to easy the grasping candidate generation process *****/
struct CandidateData
{
	geometry_msgs::PoseStamped pose;
	pr2_grasping::DescriptorCalc::Response descriptor;
	pr2_grasping::GraspingPoint point;
	float angle;
	size_t indexPoint;
	size_t indexAngle;

	CandidateData()
	{}

	CandidateData(const CandidateData &other_)
	{
		pose = other_.pose;
		descriptor = other_.descriptor;
		point = other_.point;
		angle = other_.angle;
		indexPoint = other_.indexPoint;
		indexAngle = other_.indexAngle;
	}
};

/***** Global variables *****/
ros::Publisher posePub, cancelPub, scenePub, statusPub;
boost::mutex pmutex, gmutex;
std::deque<pr2_grasping::GraspingData> queue;

float collisionMargin = 0.01;
float graspPadding = 0.1;
float nsplits = 10;
bool mockExecution = true;

bool armGoalAbort = false;
GripperState gState = STATE_IDLE;

std::string trackedObject = "";
std::string supportObject = "";

bool usePredictions = false;
int npredictions = 0;
SVMPtr svm = SVMPtr();
BoostingPtr boosting = BoostingPtr();


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
	geometry_msgs::Pose targetPose = GraspingUtils::genPose(colX, colY, colZ + 0.01); // lift the object a bit to avoid collisions with the support

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
	if (!mockExecution)
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
		// ros::Duration(0.5).sleep();

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

		if (!RobotUtils::move(effector_, 10))
		{
			ROS_WARN("Unable to move gripper for release. Clearing scene and retrying");

			moveit_msgs::PlanningSceneWorld cleanScene;
			scenePub.publish(cleanScene);
			ros::Duration(0.5).sleep();

			if (!RobotUtils::move(effector_, 10))
				ROS_WARN("Unable to move gripper, releasing 'as is'");
		}

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
float getPredictionScore(const pr2_grasping::DescriptorCalc::Response &descriptor_)
{
	size_t descSize = descriptor_.descriptor.size();
	cv::Mat sample = cv::Mat(1, descSize, CV_32FC1);

	float label = -1;
	float score = 0;
	if (svm)
	{
		ROS_DEBUG("...predicting with SVM");
		score = -svm->predict(sample, true); // - sign so the bigger the distance the more likely class 1
		label = svm->predict(sample, false);
	}
	else if (boosting)
	{
		ROS_DEBUG("...predicting with boosting tree");
		score = boosting->predict(sample, cv::Mat(), cv::Range::all(), false, true);
		label = boosting->predict(sample, cv::Mat(), cv::Range::all(), false, false);
	}

	ROS_DEBUG("...prediction  -->  class: %.0f - score: %.3f", label, score);

	return 0;
}


/**************************************************/
std::vector<GraspData> genGraspData(const EffectorSide &side_,
									const std::vector<pr2_grasping::GraspingPoint> points_,
									const bool debugEnabled_,
									std::vector<geometry_msgs::PoseStamped> &DEBUG_points_)
{
	// Number of grasping points processed
	static int pointIdx = 0;

	std::vector<GraspData> grasps;
	DEBUG_points_.clear();


	ROS_DEBUG("grasp points size: %zu", points_.size());


	// Collect grasping candidate's data
	std::vector<CandidateData> data;
	float angleStep = M_PI / nsplits;
	size_t npoints = points_.size();

	for (size_t i = 0; i < npoints; i++)
	{
		pr2_grasping::GraspingPoint point = points_[i];
		for (int j = 0; j < nsplits; j++)
		{
			// Generate a grasping pose
			float angle = j * angleStep;
			geometry_msgs::PoseStamped graspingPose = genGraspingPose(point, angle);

			pr2_grasping::DescriptorCalc desc;
			desc.request.target = graspingPose.pose;
			desc.request.angle.data = angle;
			ros::service::call("/pr2_grasping/descriptor_calculator", desc);

			CandidateData candidate;
			candidate.pose = graspingPose;
			candidate.descriptor = desc.response;
			candidate.angle = angle;
			candidate.indexPoint = i;
			candidate.indexAngle = j;
			data.push_back(candidate);
		}
	}


	std::vector<CandidateData> candidates;
	std::vector<CandidateData>::iterator start, finish;


	// Produce the grasping candidates
	if (usePredictions)
	{
		// Compute the score for each grasp pose
		std::vector<std::pair<size_t, float> > predictions;
		for (size_t i = 0; i < data.size(); i++)
			predictions.push_back(std::make_pair(i, getPredictionScore(data[i].descriptor)));

		// Sort the predictions according to the score
		std::sort(predictions.begin(), predictions.end(),
				  boost::bind(&std::pair<size_t, float>::second, _1) < boost::bind(&std::pair<size_t, float>::second, _2));


		if (debugEnabled_)
		{
			LOGD << "Sorted predictions:";
			for (size_t i = 0; i < predictions.size(); i++)
				LOGD << "\tindex: " << predictions[i].first << " -- score: " << predictions[i].second;
		}


		// Select the required number or predictions
		for (int i  = 0; i < npredictions && i < (int)predictions.size(); i++)
			candidates.push_back(data[predictions[i].first]);
		start = candidates.begin();
		finish = candidates.end();

		LOGD << "Selected " << candidates.size() << " predictions";
	}
	else
	{
		start = data.begin();
		finish = data.end();
	}


	// Finally, generate the grasping candidates
	std::map<int, int> trackedPoints;
	for (std::vector<CandidateData>::iterator it = start; start != finish; start++)
	{
		// Track the number of grasping points used
		if (trackedPoints.find(it->indexPoint) == trackedPoints.end())
			trackedPoints[it->indexPoint] = pointIdx++;


		std::string id = GRASP_ID
						 + boost::lexical_cast<std::string>(trackedPoints[it->indexPoint])
						 + "_" + boost::lexical_cast<std::string>(it->indexPoint)
						 + "_" + boost::lexical_cast<std::string>(it->indexAngle);
		moveit_msgs::Grasp grasp = genGrasp(id, side_, it->pose, OBJECT_TARGET, OBJECT_SUPPORT);


		grasps.push_back(GraspData(grasp, it->descriptor, it->angle, it->point.label));


		/***** FOR DEBUG ONLY *****/
		geometry_msgs::PoseStamped gp = it->pose;
		gp.pose.position = it->point.position;
		DEBUG_points_.push_back(gp);
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
		std::vector<GraspData> grasps = genGraspData(side_, queue.front().graspingPoints, debugEnabled_, DEBUG_points);


		/*********************************************************/
		/******** STAGE 3: attempt to grasp the object ***********/
		/*********************************************************/
		size_t ngrasps = grasps.size();
		for (size_t i = 0; i < ngrasps; i++)
		{
			ROS_INFO("*** processing grasp %zu of %zu ***", i + 1, ngrasps);
			ROS_INFO("...attempt id: %s", IO::nextExperimentId(trackedObject).c_str());
			ROS_DEBUG("grasp id: %s -- label: %d", grasps[i].grasp.id.c_str(), grasps[i].label);


			ROS_INFO("...clearing scene");
			moveit_msgs::PlanningSceneWorld cleanScene;
			scenePub.publish(cleanScene);
			ros::Duration(1).sleep();

			ROS_INFO("...adding collisions to scene");
			planningScene_->addCollisionObjects(collisions);
			ros::Duration(1).sleep();

			ROS_DEBUG("publishing grasping pose");
			posePub.publish(grasps[i].grasp.grasp_pose);
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
			if (mockExecution)
			{
				ROS_INFO("...mocking grasp routine");
				ros::Duration(1).sleep();
			}
			else
			{
				ROS_INFO("...attempting grasp");
				int maxAttempts = 10;
				moveit::planning_interface::MoveItErrorCode code;
				for (int att = 0;
						att < maxAttempts && code.val != moveit_msgs::MoveItErrorCodes::SUCCESS;
						att++)
				{
					ROS_INFO(".....attempt %d of %d", att + 1, maxAttempts);
					code = effector_->pick(OBJECT_TARGET, grasps[i].grasp);
					ROS_INFO(".....finished: %d / %s (attempt %d of %d)", code.val, PkgUtils::toString(code).c_str(), att + 1, maxAttempts);

					// Break if there was an abort, but wasn't the arm controller
					if (code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED && !armGoalAbort)
						break;
				}


				/********** Evaluate result if the grasp was completed **********/
				pr2_grasping::GraspEvaluator eval;
				eval.response.result = false;

				// Skip the rest if the planning failed
				bool completed = false;
				if (code.val == moveit_msgs::MoveItErrorCodes::SUCCESS ||
						(code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED &&
						 gState == STATE_STUCK) ||
						(code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED &&
						 armGoalAbort))
				{
					completed = true;

					// Call the evaluation node
					ROS_INFO("...evaluating result");
					while (!ros::service::call("/pr2_grasping/grasp_evaluator", eval))
					{
						ROS_WARN("...clearing scene for evaluation");
						moveit_msgs::PlanningSceneWorld cleanScene;
						scenePub.publish(cleanScene);
					}

					ROS_INFO("...grasp attempt %s", eval.response.result ? "SUCCESSFUL" : "FAILED");
				}
				else
					ROS_INFO("...attempt failed, skipping evaluation");


				// pr2_grasping::DescriptorCalc desc;
				// desc.request.target = grasps[i].grasp.grasp_pose.pose;
				// ros::service::call("/pr2_grasping/descriptor_calculator", desc);


				/********** Store the result **********/
				IO::saveResults(trackedObject,
								completed,
								eval.response.result,
								grasps[i].label,
								grasps[i].angle,
								grasps[i].grasp,
								nsplits,
								(M_PI / nsplits),
								code,
								grasps[i].descriptor);
			}


			// restore tracked state
			gmutex.lock();
			gState = STATE_IDLE;
			gmutex.unlock();


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

			ROS_INFO("...restoring setup");
			pr2_grasping::GazeboSetup setup;
			if (ros::service::call("/pr2_grasping/gazebo_setup", setup))
			{
				IO::nextExperimentId(trackedObject);
				ROS_INFO("...setup %s", setup.response.result ? "RESTORED" : "restore FAILED");
			}
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
bool queryName(pr2_grasping::GraspingGroup::Request &request_,
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
bool queryId(pr2_grasping::ExperimentId::Request &request_,
			 pr2_grasping::ExperimentId::Response &response_)
{
	response_.id = IO::getExperimentId();
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
void armAbortedCallback(const control_msgs::FollowJointTrajectoryActionResult &msg_)
{
	// ROS_DEBUG("Arm status: %d -- result: %d ", msg_.status.status, msg_.result.error_code);
	ROS_DEBUG("Arm status: %s -- Arm result: %s ", PkgUtils::toString(msg_.status).c_str(), PkgUtils::toString(msg_.result).c_str());

	bool actionAborted = msg_.status.status == actionlib_msgs::GoalStatus::ABORTED;
	bool goalViolation = (msg_.result.error_code == control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED) ||
						 (msg_.result.error_code == control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED);

	if (actionAborted && goalViolation)
		armGoalAbort = true;
	else
		armGoalAbort = false;

	ROS_DEBUG("armGoalAbort: %s", armGoalAbort ? "TRUE" : "FALSE");
}


/**************************************************/
void loadClassifier()
{
	npredictions = Config::get()["grasper"]["predictions"]["npredictions"].as<int>();


	std::string location = ros::package::getPath(PACKAGE_NAME) + "/" + Config::get()["grasper"]["predictions"]["classifier"].as<std::string>();
	ROS_INFO("Loading classifier at %s", location.c_str());

	YAML::Node file = YAML::LoadFile(location);
	if (file["my_svm"])
	{
		ROS_INFO("...loading SVM");
		svm = SVMPtr(new cv::SVM());
		svm->load(location.c_str());
	}
	else if (file["my_boost_tree"])
	{
		ROS_INFO("...loading boosting tree");
		boosting = BoostingPtr(new cv::Boost());
		boosting->load(location.c_str());
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
	if (!Config::load(PkgUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + PkgUtils::getConfigPath());

	// Get the params
	bool debugEnabled = Config::get()["grasperDebug"].as<bool>();
	collisionMargin = Config::get()["grasper"]["collisionMargin"].as<float>();
	graspPadding = Config::get()["grasper"]["graspPadding"].as<float>();
	nsplits = Config::get()["grasper"]["angleSplits"].as<int>();
	mockExecution = Config::get()["grasper"]["mockExecution"].as<bool>();

	// Load the classifier if requested
	usePredictions = Config::get()["grasper"]["usePredictions"].as<bool>();
	if (usePredictions)
		loadClassifier();


	/********** Setup control group **********/
	ROS_INFO("Setting effector control");
	std::string arm = Config::get()["grasper"]["arm"].as<std::string>();
	std::pair<std::string, std::string> effectorNames = RobotUtils::getEffectorNames(arm);
	MoveGroupPtr effector = MoveGroupPtr(new moveit::planning_interface::MoveGroup(effectorNames.first));
	effector->setEndEffector(effectorNames.second);
	effector->setPlannerId("RRTConnectkConfigDefault");
	effector->allowReplanning(true);


	/********** Set services **********/
	ROS_INFO("Starting name query service");
	ros::ServiceServer nameService = handler.advertiseService<pr2_grasping::GraspingGroup::Request, pr2_grasping::GraspingGroup::Response>("/pr2_grasping/effector_name", boost::bind(queryName, _1, _2, effector));
	ROS_INFO("Starting id query service");
	ros::ServiceServer idService = handler.advertiseService("/pr2_grasping/experiment_id", queryId);


	/********** Set subscriptions/publishers **********/
	ROS_INFO("Setting publishers/subscribers");
	statusPub = handler.advertise<std_msgs::Int32>("/pr2_grasping/processed_sets", 1, true);
	cancelPub = handler.advertise<actionlib_msgs::GoalID>(RobotUtils::getGripperTopic(arm) + "/cancel", 1);
	posePub = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/grasping_pose", 1, true);
	scenePub = handler.advertise<moveit_msgs::PlanningSceneWorld>("/planning_scene_world", 1);

	ros::Subscriber pointsSub = handler.subscribe("/pr2_grasping/grasping_data", 1, graspingPointsCallback);
	ros::Subscriber gripperStuckSub = handler.subscribe("/pr2_grasping/gripper_action_stuck", 1, gripperStuckCallback);
	ros::Subscriber armAbortSub = handler.subscribe(RobotUtils::getArmTopic(arm) + "/result", 1, armAbortedCallback);

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
	IO::nextExperimentId(trackedObject);


	/********** Start periodic check/ask for new grasping points **********/
	ros::Timer timer = handler.createTimer(ros::Duration(2), timerCallback);

	// Spin at 10 Hz
	EffectorSide side = RobotUtils::getEffectorSide(arm);
	ros::Rate r(10);
	while (ros::ok())
	{
		graspingRoutine(&planningScene, effector, side, debugEnabled);
		r.sleep();
	}

	ros::shutdown();
	return EXIT_SUCCESS;
}
