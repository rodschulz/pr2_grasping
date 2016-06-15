/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

// flag to coordinate the displacement detention
bool stopDisplacement = false;

void displacementCallback(const nav_msgs::Odometry::ConstPtr &msg_)
{
	if (msg_->pose.pose.position.x >= 1.1)
	{
		ROS_INFO("Destination reached");
		stopDisplacement = true;
	}
}

void liftUpTorso()
{
	// define action clients
	TorsoClient *torsoClient = new TorsoClient("torso_controller/position_joint_action", true);

	// wait for the action server to come up
	while (!torsoClient->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for action server to come up");

	ROS_INFO("Lifting up torso");

	pr2_controllers_msgs::SingleJointPositionGoal up;
	up.position = 0.195;
	up.min_duration = ros::Duration(2.0);
	up.max_velocity = 1.0;

	ROS_INFO("...sending torso goal");
	torsoClient->sendGoal(up);
	torsoClient->waitForResult();

	ROS_INFO("...torso lifted up");
}

void moveArms()
{
	ROS_INFO("Preparing arms setup");

	// group to plane movement for both arms
	moveit::planning_interface::MoveGroup armsGroup("arms");

	// right arm pose
	geometry_msgs::Pose rightArmPose;
	rightArmPose.orientation.w = 1.0;
	rightArmPose.position.x = 0.3;
	rightArmPose.position.y = -0.5;
	rightArmPose.position.z = 1.4;

	// left arm pose
	geometry_msgs::Pose leftArmPose;
	leftArmPose.orientation.w = 1.0;
	leftArmPose.position.x = 0.3;
	leftArmPose.position.y = 0.5;
	leftArmPose.position.z = 1.4;

	// set the pose for each arm
	armsGroup.setPoseTarget(rightArmPose, "r_wrist_roll_link");
	armsGroup.setPoseTarget(leftArmPose, "l_wrist_roll_link");

	// plan the trajectory
	moveit::planning_interface::MoveGroup::Plan armsPlan;
	ROS_INFO("...planing arms trajectory");
	bool planningOk = armsGroup.plan(armsPlan);
	ROS_INFO("...trajectory plan %s", planningOk ? "SUCCESSFUL" : "FAILED");

	// move the arms
	if (planningOk)
	{
		ROS_INFO("...moving arms");
		armsGroup.move();
	}

	ROS_INFO("...arms movement completed");
}

void moveBase(ros::Publisher &publisher_)
{
	ROS_INFO("Moving robot base");

	geometry_msgs::Twist cmd;
	cmd.linear.x = 2;
	cmd.linear.y = cmd.linear.z = 0;
	cmd.angular.x = cmd.angular.y = cmd.angular.z = 0;

	while (ros::ok() && !stopDisplacement)
		publisher_.publish(cmd);

	ROS_INFO("...displacement completed");
}

void sx()
{
}

int main(int argc, char** argv)
{
	// setup node
	ros::init(argc, argv, "arms_setup");

	// define publisher and subscriber for the base's movement
	ros::NodeHandle handler;
	ros::Publisher publisher = handler.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
	ros::Subscriber subscriber = handler.subscribe("/base_pose_ground_truth", 10, displacementCallback);

	ROS_INFO("Beginning setup routine");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	liftUpTorso();
	moveArms();
	moveBase(publisher);

	ROS_INFO("Setup routine completed");
	ros::shutdown();

	return EXIT_SUCCESS;
}
