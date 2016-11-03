/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>
#include "GraspingUtils.hpp"
#include "RobotUtils.hpp"


#define TARGET_OBJECT 		"target_object"
#define SUPPORT_OBJECT 		"support_object"


bool stopDisplacement = false;


/**************************************************/
void displacementCallback(const nav_msgs::Odometry::ConstPtr &msg_)
{
	if (!stopDisplacement && msg_->pose.pose.position.x >= 0.88)
		stopDisplacement = true;
}


/**************************************************/
void liftUpTorso()
{
	// define action client
	TorsoClient *torsoClient = new TorsoClient("torso_controller/position_joint_action", true);

	// wait for the action server to come up
	while (!torsoClient->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for torso action server");

	ROS_INFO("Lifting up torso");

	control_msgs::SingleJointPositionGoal torsoGoal;
	torsoGoal.position = 0.18;
	// torsoGoal.min_duration = ros::Duration(1.0);
	// torsoGoal.max_velocity = 5.0;

	ROS_INFO("...sending torso goal");
	torsoClient->sendGoal(torsoGoal);
	torsoClient->waitForResult();

	ROS_INFO("...torso lifted up");
}


/**************************************************/
void moveArmToPose(const EffectorSide arm_,
				   const geometry_msgs::Pose &targetPose_,
				   const std::string &targetRef_)
{
	std::pair<std::string, std::string> effector = RobotUtils::getEffectorNames(arm_);

	// group to plane arm movement
	moveit::planning_interface::MoveGroup arm(effector.first);
	arm.setEndEffector(effector.second);

	// Set the pose for the arm
	arm.setPoseReferenceFrame(targetRef_);
	arm.setPoseTarget(targetPose_);
	arm.setPlanningTime(30);

	// plan the trajectory
	ROS_INFO("...planing arm trajectory");
	moveit::planning_interface::MoveGroup::Plan armPlan;
	bool planningOk = arm.plan(armPlan);
	ROS_INFO("...plan %s", planningOk ? "SUCCESSFUL" : "FAILED");

	// move the arm
	if (planningOk)
		arm.move();

	ROS_INFO("...arm movement completed");
}


/**************************************************/
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


/**************************************************/
void moveHead()
{
	// define action client
	HeadClient *headClient = new HeadClient("/head_traj_controller/point_head_action", true);

	// wait for the action server to come up
	while (!headClient->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for head action server");

	ROS_INFO("Moving head");

	// the target point, expressed in the given frame
	geometry_msgs::PointStamped targetPoint;
	targetPoint.header.frame_id = "base_footprint";
	targetPoint.point.x = 0.9;
	targetPoint.point.y = 0;
	targetPoint.point.z = 0.55;

	// make the kinect x axis point at the desired position
	control_msgs::PointHeadGoal goal;
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


/**********************************************************************/
moveit_msgs::CollisionObject genCollisionObject(const std::string objectId_,
		const geometry_msgs::PoseStamped &objectPose_,
		const float dimX_,
		const float dimY_,
		const float dimZ_)
{
	moveit_msgs::CollisionObject collision;

	collision.id = objectId_;
	collision.header.stamp = ros::Time(0);
	collision.header.frame_id = objectPose_.header.frame_id;

	// Add the bounding box of the object for collisions and its pose
	shape_msgs::SolidPrimitive primitive;
	primitive.type = shape_msgs::SolidPrimitive::BOX;
	primitive.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = dimX_;
	primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dimY_;
	primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dimZ_;

	collision.primitives.push_back(primitive);
	collision.primitive_poses.push_back(objectPose_.pose);

	collision.operation = moveit_msgs::CollisionObject::ADD;

	return collision;
}


/**************************************************/
void groupPick(moveit::planning_interface::PlanningSceneInterface &planningScene_,
			   ros::Publisher &posePublisher_,
			   const geometry_msgs::PoseStamped &targetPose_,
			   const geometry_msgs::PoseStamped &supportPose_)
{
	ROS_INFO("Starting group pick routine");

	/********** Generate collision objects **********/
	float dimX = 0.13;
	float dimY = 0.13;
	float dimZ = 0.27;
	moveit_msgs::CollisionObject target = genCollisionObject(TARGET_OBJECT, targetPose_, dimX, dimY, dimZ);

	dimX = 1.5; //0.92
	dimY = 0.82; //0.92
	dimZ = 0.62; //0.78
	moveit_msgs::CollisionObject support = genCollisionObject(SUPPORT_OBJECT, supportPose_, dimX, dimY, dimZ);

	std::vector<moveit_msgs::CollisionObject> collisions;
	collisions.push_back(target);
	collisions.push_back(support);

	ROS_INFO("...adding collisions to scene");
	planningScene_.addCollisionObjects(collisions);
	ros::Duration(3.0).sleep();


	/********** Generate grasp **********/
	ROS_INFO("...generating grasp");
	geometry_msgs::PoseStamped graspPose;
	graspPose.header = targetPose_.header;
	graspPose.pose = GraspingUtils::genPose(targetPose_.pose.position.x,
											targetPose_.pose.position.y - 0.18,
											targetPose_.pose.position.z,
											DEG2RAD(90), 0, 0, 1);


	ROS_INFO("...publishing grasping pose");
	posePublisher_.publish(graspPose);


	moveit_msgs::Grasp grasp;
	grasp.id = "TARGET_GRASP";
	grasp.grasp_pose = graspPose;

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


	grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint";
	grasp.post_grasp_retreat.direction.vector.x = 0;
	grasp.post_grasp_retreat.direction.vector.y = 0;
	grasp.post_grasp_retreat.direction.vector.z = 1;
	grasp.post_grasp_retreat.min_distance = 0.08;
	grasp.post_grasp_retreat.desired_distance = 0.3;

	grasp.allowed_touch_objects.clear();
	grasp.allowed_touch_objects.push_back(TARGET_OBJECT);
	grasp.allowed_touch_objects.push_back(SUPPORT_OBJECT);


	/********** Attempt pickup **********/
	ROS_INFO("...attempting pickup");

	moveit::planning_interface::MoveGroup group("right_arm");
	group.setEndEffector("right_eef");
	group.setPlanningTime(45.0);
	// group.setPoseTarget(grasp.grasp_pose);
	group.setPlannerId("RRTConnectkConfigDefault");

	ROS_INFO("...planning frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("...end effector link: %s", group.getEndEffectorLink().c_str());

	group.pick(TARGET_OBJECT, grasp);
	ros::Duration(3).sleep();
}


/**************************************************/
int main(int argn_, char **argv_)
{
	// setup node
	ros::init(argn_, argv_, "example_group_pick");
	ros::NodeHandle handler;

	// define publishers and subscribers
	ros::Publisher displacementPub = handler.advertise<geometry_msgs::Twist>("/base_controller/command", 10);
	ros::Publisher posePublisher = handler.advertise<geometry_msgs::PoseStamped>("/test/grasp_pose", 10, true);
	ros::Subscriber basePoseSub = handler.subscribe("/base_pose_ground_truth", 1, displacementCallback);


	// Set spinning
	ROS_INFO("Beginning setup routine");
	ros::AsyncSpinner spinner(1);
	spinner.start();


	// For some reason this MUST be declared before using the group planning capabilities, otherwise
	// collision objects won't be added to the planning interface (WTF)
	moveit::planning_interface::PlanningSceneInterface planningScene;


	// Prepare initial setup
	moveArmToPose(RIGHT_ARM, GraspingUtils::genPose(0.35, -0.5, 1.1), "base_footprint");
	moveArmToPose(LEFT_ARM, GraspingUtils::genPose(0.35, 0.5, 1.1), "base_footprint");
	moveHead();
	// liftUpTorso();
	// moveBase(displacementPub);
	ROS_INFO("Setup completed");


	// Generate target object's pose
	geometry_msgs::PoseStamped targetPose;
	targetPose.header.stamp = ros::Time(0);
	targetPose.header.frame_id = "odom_combined";
	targetPose.pose = GraspingUtils::genPose(0.75, 0, 0.63 + 0.23 * 0.5, DEG2RAD(0), 0, 0, 1);

	// Generate support's pose
	geometry_msgs::PoseStamped supportPose;
	supportPose.header.stamp = ros::Time(0);
	supportPose.header.frame_id = "odom_combined";
	supportPose.pose = GraspingUtils::genPose(1.4, 0, 0.615 * 0.5, DEG2RAD(0), 0, 0, 1);


	// Pick an object
	groupPick(planningScene, posePublisher, targetPose, supportPose);


	ROS_INFO("Moving arm after picking");
	// moveArmToPose(RIGHT_ARM, GraspingUtils::genPose(0.35, -0.5, 1.1), "base_footprint");
	moveArmToPose(RIGHT_ARM, GraspingUtils::genPose(0.5, 0.0, 1.0, DEG2RAD(-90), 0, 1, 0), "base_footprint");


	ROS_INFO("Routine completed");
	spinner.stop();

	return EXIT_SUCCESS;
}
