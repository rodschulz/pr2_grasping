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

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> HeadClient;

// flag to coordinate the displacement detention
bool stopDisplacement = false;



#include <tf/transform_listener.h>
#include "GraspingUtils.hpp"
tf::TransformListener *tfListener;



void displacementCallback(const nav_msgs::Odometry::ConstPtr &msg_)
{
	// if (!stopDisplacement && msg_->pose.pose.position.x >= 1.1)
	if (!stopDisplacement && msg_->pose.pose.position.x >= 0.8)
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

void moveArms(const int _idx)
{
	ROS_INFO("Preparing arms setup");

	// group to plane movement for both arms
	moveit::planning_interface::MoveGroup arms("right_arm");
	arms.setEndEffector("right_eef");


	// right arm pose
	geometry_msgs::Pose rightPose;
	if (_idx == 1)
	{
		rightPose.orientation.w = 1.0;
		rightPose.position.x = 0.3;
		rightPose.position.y = -0.5;
		rightPose.position.z = 1;
	}
	else
	{
		rightPose.position.x = 0.45;
		rightPose.position.y = -0.188152;
		rightPose.position.z = 1;

		Eigen::Vector3f p = Eigen::Vector3f(1, 0, 1).normalized();
		float angle = DEG2RAD(60);
		rightPose.orientation.w = cos(angle/2);
		rightPose.orientation.x = p.x() * sin(angle/2);
		rightPose.orientation.y = p.y() * sin(angle/2);
		rightPose.orientation.z = p.z() * sin(angle/2);
	}


	// set the pose for each arm
	std::string targetRef = "base_link";
	arms.setPoseReferenceFrame(targetRef);
	arms.setPoseTarget(rightPose);



	ROS_INFO("...plan ref: %s - pose ref: %s", arms.getPlanningFrame().c_str(), arms.getPoseReferenceFrame().c_str());
	std::string origRef = arms.getCurrentPose().header.frame_id;
	geometry_msgs::Pose curr = arms.getCurrentPose().pose;
	ROS_INFO("+++curr:(%.3f, %.3f, %.3f) // ref:%s", curr.position.x, curr.position.y, curr.position.z, origRef.c_str());

	tf::StampedTransform toTarget;
	while (!GraspingUtils::getTransformation(toTarget, tfListener, targetRef, origRef));
	tf::Vector3 conv = toTarget * tf::Vector3(curr.position.x, curr.position.y, curr.position.z);
	ROS_INFO("+++curr:(%.3f, %.3f, %.3f) // ref:%s", conv.x(), conv.y(), conv.z(), toTarget.frame_id_.c_str());

	tf::StampedTransform toOrig;
	while (!GraspingUtils::getTransformation(toOrig, tfListener, origRef, targetRef));
	conv = toOrig * tf::Vector3(rightPose.position.x, rightPose.position.y, rightPose.position.z);
	ROS_INFO("+++target:(%.3f, %.3f, %.3f) // ref:%s", conv.x(), conv.y(), conv.z(), toOrig.frame_id_.c_str());



	// plan the trajectory
	moveit::planning_interface::MoveGroup::Plan armsPlan;
	ROS_INFO("...planing arms trajectory");
	bool planningOk = arms.plan(armsPlan);
	ROS_INFO("...trajectory plan %s", planningOk ? "SUCCESSFUL" : "FAILED");

	// move the arms
	if (planningOk)
	{
		ROS_INFO("...moving arms");
		arms.move();


		/**********/
		curr = arms.getCurrentPose().pose;
		ROS_INFO("***after:(%.3f, %.3f, %.3f) // ref:%s", curr.position.x, curr.position.y, curr.position.z, origRef.c_str());

		while (!GraspingUtils::getTransformation(toTarget, tfListener, targetRef, origRef));
		conv = toTarget * tf::Vector3(curr.position.x, curr.position.y, curr.position.z);
		ROS_INFO("***after:(%.3f, %.3f, %.3f) // ref:%s", conv.x(), conv.y(), conv.z(), toTarget.frame_id_.c_str());
		/**********/
	}

	ROS_INFO("...arms movement completed");
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

int main(int argc, char** argv)
{
	// setup node
	ros::init(argc, argv, "test_node");

	// define publisher and subscriber for the base's movement
	ros::NodeHandle handler;
	ros::Publisher publisher = handler.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
	ros::Subscriber subscriber = handler.subscribe("/base_pose_ground_truth", 1, displacementCallback);


	// Tf listener
	tfListener = new tf::TransformListener(ros::Duration(10.0));


	ROS_INFO("Beginning setup routine");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	liftUpTorso();
	moveArms(1);
	moveBase(publisher);
	moveHead();
	moveArms(2);

	ROS_INFO("Setup routine completed");
	spinner.stop();
	// ros::shutdown();

	return EXIT_SUCCESS;
}
