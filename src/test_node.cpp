/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>
#include <shape_tools/solid_primitive_dims.h>
#include "GraspingUtils.hpp"


typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> HeadClient;

// flag to coordinate the displacement detention
bool stopDisplacement = false;
// Transformations listener
tf::TransformListener *tfListener;


void displacementCallback(const nav_msgs::Odometry::ConstPtr &msg_)
{
	if (!stopDisplacement && msg_->pose.pose.position.x >= 1.1)
		// if (!stopDisplacement && msg_->pose.pose.position.x >= 0.8)
	{
		ROS_INFO("Destination reached");
		stopDisplacement = true;
	}
}

void liftUpTorso()
{
	// define action client
	TorsoClient *torsoClient = new TorsoClient("torso_controller/position_joint_action", true);

	// wait for the action server to come up
	while (!torsoClient->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for action server to come up");

	ROS_INFO("Lifting up torso");

	pr2_controllers_msgs::SingleJointPositionGoal torsoGoal;
	torsoGoal.position = 0.19;
	torsoGoal.min_duration = ros::Duration(1.0);
	torsoGoal.max_velocity = 5.0;

	ROS_INFO("...sending torso goal");
	torsoClient->sendGoal(torsoGoal);
	torsoClient->waitForResult();

	ROS_INFO("...torso lifted up");
}

void moveBothArms()
{
	ROS_INFO("Preparing arms setup");

	// group to plane movement for both arms
	moveit::planning_interface::MoveGroup arms("arms");
	arms.setPoseReferenceFrame("base_link");

	// right arm pose
	geometry_msgs::Pose rightArmPose;
	rightArmPose.orientation.w = 1.0;
	rightArmPose.position.x = 0.3;
	rightArmPose.position.y = -0.5;
	rightArmPose.position.z = 1.3;

	// left arm pose
	geometry_msgs::Pose leftArmPose;
	leftArmPose.orientation.w = 1.0;
	leftArmPose.position.x = 0.3;
	leftArmPose.position.y = 0.5;
	leftArmPose.position.z = 1.3;

	// set the pose for each arm
	arms.setPoseTarget(rightArmPose, "r_wrist_roll_link");
	arms.setPoseTarget(leftArmPose, "l_wrist_roll_link");

	// plan the trajectory
	moveit::planning_interface::MoveGroup::Plan armsPlan;
	ROS_INFO("...planing arms trajectory");
	bool planningOk = arms.plan(armsPlan);
	ROS_INFO("...trajectory plan %s", planningOk ? "SUCCESSFUL" : "FAILED");

	// move the arms
	if (planningOk)
	{
		ROS_INFO("...moving both arms");
		arms.move();
	}

	ROS_INFO("...arms movement completed");
}

void moveArmToPose(const geometry_msgs::Pose &targetPose_,
				   const std::string &targetRef_)
{
	ROS_INFO("Preparing arm setup");

	// group to plane arm movement
	moveit::planning_interface::MoveGroup arm("right_arm");
	arm.setEndEffector("right_eef");

	// Set the pose for the arm
	arm.setPoseReferenceFrame(targetRef_);
	arm.setPoseTarget(targetPose_);
	arm.setPlanningTime(30);



	ROS_INFO("...plan ref: %s - pose ref: %s", arm.getPlanningFrame().c_str(), arm.getPoseReferenceFrame().c_str());
	std::string origRef = arm.getCurrentPose().header.frame_id;
	geometry_msgs::Pose curr = arm.getCurrentPose().pose;
	ROS_INFO("+++curr:(%.3f, %.3f, %.3f) // ref:%s", curr.position.x, curr.position.y, curr.position.z, origRef.c_str());

	tf::StampedTransform toTarget;
	while (!GraspingUtils::getTransformation(toTarget, tfListener, targetRef_, origRef));
	tf::Vector3 conv = toTarget * tf::Vector3(curr.position.x, curr.position.y, curr.position.z);
	ROS_INFO("+++curr:(%.3f, %.3f, %.3f) // ref:%s", conv.x(), conv.y(), conv.z(), toTarget.frame_id_.c_str());

	tf::StampedTransform toOrig;
	while (!GraspingUtils::getTransformation(toOrig, tfListener, origRef, targetRef_));
	conv = toOrig * tf::Vector3(targetPose_.position.x, targetPose_.position.y, targetPose_.position.z);
	ROS_INFO("+++target:(%.3f, %.3f, %.3f) // ref:%s", conv.x(), conv.y(), conv.z(), toOrig.frame_id_.c_str());



	// plan the trajectory
	ROS_INFO("...planing arm trajectory");
	moveit::planning_interface::MoveGroup::Plan armPlan;
	bool planningOk = arm.plan(armPlan);
	ROS_INFO("...trajectory plan %s", planningOk ? "SUCCESSFUL" : "FAILED");

	// move the arm
	if (planningOk)
	{
		ROS_INFO("...moving arm");
		arm.move();


		/**********/
		curr = arm.getCurrentPose().pose;
		ROS_INFO("***after:(%.3f, %.3f, %.3f) // ref:%s", curr.position.x, curr.position.y, curr.position.z, origRef.c_str());

		while (!GraspingUtils::getTransformation(toTarget, tfListener, targetRef_, origRef));
		conv = toTarget * tf::Vector3(curr.position.x, curr.position.y, curr.position.z);
		ROS_INFO("***after:(%.3f, %.3f, %.3f) // ref:%s", conv.x(), conv.y(), conv.z(), toTarget.frame_id_.c_str());
		/**********/
	}

	ROS_INFO("...arm movement completed");
}

void moveBase(ros::Publisher &publisher_)
{
	ROS_INFO("Moving robot base");

	geometry_msgs::Twist cmd;
	cmd.linear.x = 4;
	cmd.linear.y = cmd.linear.z = 0;
	cmd.angular.x = cmd.angular.y = cmd.angular.z = 0;

	while (ros::ok() && !stopDisplacement)
		publisher_.publish(cmd);

	ROS_INFO("...displacement completed");
}

void moveHead()
{
	// define action client
	HeadClient *headClient = new HeadClient("/head_traj_controller/point_head_action", true);

	// wait for the action server to come up
	while (!headClient->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for the point_head_action server to come up");

	ROS_INFO("Moving head");

	// the target point, expressed in the given frame
	geometry_msgs::PointStamped targetPoint;
	targetPoint.header.frame_id = "base_link";
	targetPoint.point.x = 0.9;
	targetPoint.point.y = 0;
	targetPoint.point.z = 0.5;

	// make the kinect x axis point at the desired position
	pr2_controllers_msgs::PointHeadGoal goal;
	goal.target = targetPoint;
	goal.pointing_frame = "head_mount_kinect_rgb_link";
	goal.pointing_axis.x = 1;
	goal.pointing_axis.y = 0;
	goal.pointing_axis.z = 0;

	// displacement limits (at least 1 sec and no faster than 1 rad/s)
	goal.min_duration = ros::Duration(1);
	goal.max_velocity = 1.0;

	ROS_INFO("...sending head goal");
	headClient->sendGoal(goal);
	headClient->waitForResult(ros::Duration(2));

	ROS_INFO("...head moved");
}

void addCollisionBox(ros::Publisher &collisionPub_,
					 const std::string &frame_,
					 const std::string &objectName_,
					 const geometry_msgs::Pose &objectPose_,
					 const float dimx_,
					 const float dimy_,
					 const float dimz_)
{
	moveit_msgs::CollisionObject collision;

	collision.id = objectName_;
	collision.header.stamp = ros::Time::now();
	collision.header.frame_id = frame_;

	// Add the bounding box of the object for collisions and its pose
	shape_msgs::SolidPrimitive primitive;
	primitive.type = shape_msgs::SolidPrimitive::BOX;
	primitive.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = dimx_;
	primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dimy_;
	primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dimz_;

	collision.primitives.push_back(primitive);
	collision.primitive_poses.push_back(objectPose_);

	// Remove and detach the object first (just in case)
	ROS_INFO("Removing object");
	collision.operation = moveit_msgs::CollisionObject::REMOVE;
	collisionPub_.publish(collision);
	ros::WallDuration(1.0).sleep();

	// Then add it to the scene
	ROS_INFO("Adding object");
	collision.operation = moveit_msgs::CollisionObject::ADD;
	collisionPub_.publish(collision);
	ros::WallDuration(1.0).sleep();
}

void addObjects(ros::Publisher &collisionPub_)
{
	ROS_INFO("Adding table to scene");
	float dimH = 0.92;
	float dimV = 0.78;
	geometry_msgs::Pose tablePose = GraspingUtils::genPose(0.37 + dimH / 2, 0 , 0 + dimV / 2);
	addCollisionBox(collisionPub_, "base_link", "table", tablePose, dimH, dimH, dimV);

	ROS_INFO("Adding object to scene");
	dimH = 0.06 * 2;
	dimV = 0.25;
	geometry_msgs::Pose objectPose = GraspingUtils::genPose(0.58, 0, 0.87);
	addCollisionBox(collisionPub_, "base_link", "target_object", objectPose, dimH, dimH, dimV);
}

int main(int argc, char** argv)
{
	// setup node
	ros::init(argc, argv, "test_node");
	ros::NodeHandle handler;
	tfListener = new tf::TransformListener(ros::Duration(10.0));

	// define publishers and subscribers
	ros::Publisher displacementPub = handler.advertise<geometry_msgs::Twist>("/base_controller/command", 10);
	ros::Publisher collisionPub = handler.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
	// ros::Publisher attachPub = handler.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
	ros::Subscriber basePoseSub = handler.subscribe("/base_pose_ground_truth", 1, displacementCallback);

	// Set spinning
	ROS_INFO("Beginning setup routine");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	addObjects(collisionPub);

	// liftUpTorso();
	// moveBothArms();
	// moveArmToPose(GraspingUtils::genPose(0.3, -0.5, 1.3), "base_link");
	// moveBase(displacementPub);
	// moveHead();
	// moveArmToPose(GraspingUtils::genPose(0.45, -0.188152, 1, DEG2RAD(60), 1, 0, 1), "base_link");
	// ros::Duration(10).sleep();
	// moveArmToPose(GraspingUtils::genPose(0.546, 0.011, 0.77, DEG2RAD(45), 1, 1, 0), "base_link");
	moveArmToPose(GraspingUtils::genPose(0.607, 0.006, 1.2, 0, 0, 1, 0), "base_link");



	ROS_INFO("Setup routine completed");
	// ros::waitForShutdown();
	spinner.stop();
	ros::shutdown();

	return EXIT_SUCCESS;
}
