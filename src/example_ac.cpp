/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>
#include "GraspingUtils.hpp"
#include "RobotUtils.hpp"


#define TARGET_ID	"target"
#define SUPPORT_ID	"support"

typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> HeadClient;
typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> PickupClient;


/**************************************************/
void moveArmToPose(const Effector arm_,
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
void genCollisionObject(ros::Publisher &collisionPub_,
						const std::string objectId_,
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

	// Publish the collision object
	collisionPub_.publish(collision);
}


/**********************************************************************/
void actionClientPickup(ros::Publisher &collisionPub_,
						ros::Publisher &posePublisher_,
						PickupClient *client_,
						const geometry_msgs::PoseStamped &targetPose_,
						const geometry_msgs::PoseStamped &supportPose_)
{
	/********** Generate collision objects **********/
	ROS_INFO("...adding collisions to the scene");
	float dimX = 0.13;
	float dimY = 0.13;
	float dimZ = 0.27;
	genCollisionObject(collisionPub_, TARGET_ID, targetPose_, dimX, dimY, dimZ);
	ros::Duration(1.0).sleep();

	dimX = 1.5; //0.92
	dimY = 0.82; //0.92
	dimZ = 0.62; //0.78
	genCollisionObject(collisionPub_, SUPPORT_ID, supportPose_, dimX, dimY, dimZ);
	ros::Duration(1.0).sleep();


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
	grasp.pre_grasp_posture.points[0].positions[0] = 0.8;
	grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(45.0);


	grasp.grasp_posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
	grasp.grasp_posture.points.resize(1);
	grasp.grasp_posture.points[0].positions.resize(1);
	grasp.grasp_posture.points[0].positions[0] = 0.01;
	grasp.grasp_posture.points[0].time_from_start = ros::Duration(45.0);


	grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint";
	grasp.post_grasp_retreat.direction.vector.x = 0;
	grasp.post_grasp_retreat.direction.vector.y = 0;
	grasp.post_grasp_retreat.direction.vector.z = 1;
	grasp.post_grasp_retreat.min_distance = 0.08;
	grasp.post_grasp_retreat.desired_distance = 0.3;

	grasp.allowed_touch_objects.clear();
	grasp.allowed_touch_objects.push_back(TARGET_ID);
	grasp.allowed_touch_objects.push_back(SUPPORT_ID);
	grasp.allowed_touch_objects.push_back("r_gripper_palm_link");


	/********** Attempt pickup **********/
	ROS_INFO("...generating pickup goal");
	moveit_msgs::PickupGoal goal;

	goal.group_name = "right_arm";
	goal.end_effector = "right_eef";

	goal.target_name = TARGET_ID;
	goal.support_surface_name = SUPPORT_ID;

	goal.planner_id = "RRTConnectkConfigDefault";
	goal.allowed_planning_time = 120;

	goal.planning_options.plan_only = false;
	goal.planning_options.replan = false;
	goal.planning_options.replan_delay = 3;
	goal.planning_options.replan_attempts = 5;

	goal.planning_options.planning_scene_diff.is_diff = true;
	goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

	goal.possible_grasps.push_back(grasp);


	/********** Attempt pickup **********/
	ROS_INFO("...preparing pickup");
	client_->cancelAllGoals();
	ros::Duration(3.0).sleep();

	ROS_INFO("...sending pickup goal");
	client_->sendGoal(goal);
	client_->waitForResult(ros::Duration(45.0));

	ROS_INFO("Client state: %s", client_->getState().toString().c_str());
	if (client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Pickup SUCCEEDED");
}


/**********************************************************************/
int main(int argn_, char **argv_)
{
	ros::init(argn_, argv_, "example_action_client");
	ros::NodeHandle handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();


	// Set publishers
	ROS_INFO("Setting publishers");
	ros::Publisher posePub = handle.advertise<geometry_msgs::PoseStamped>("/test/grasp_pose", 10, true);
	ros::Publisher collisionPub = handle.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);


	// Setup clients action
	ROS_INFO("Setting action servers");
	PickupClient *pickupClient = new PickupClient(move_group::PICKUP_ACTION, false);
	while (!pickupClient->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for pickup action server");


	// Prepare initial setup
	moveArmToPose(RIGHT_ARM, GraspingUtils::genPose(0.35, -0.5, 1.1), "base_footprint");
	moveArmToPose(LEFT_ARM, GraspingUtils::genPose(0.35, 0.5, 1.1), "base_footprint");
	moveHead();
	ROS_INFO("Setup completed");


	// Generate target's pose
	geometry_msgs::PoseStamped targetPose;
	targetPose.header.stamp = ros::Time(0);
	targetPose.header.frame_id = "odom_combined";
	targetPose.pose = GraspingUtils::genPose(0.75, 0, 0.63 + 0.23 * 0.5, DEG2RAD(0), 0, 0, 1);

	// Generate support's pose
	geometry_msgs::PoseStamped supportPose;
	supportPose.header.stamp = ros::Time(0);
	supportPose.header.frame_id = "odom_combined";
	supportPose.pose = GraspingUtils::genPose(1.4, 0, 0.615 * 0.5, DEG2RAD(0), 0, 0, 1);

	// Attempt the pickup
	actionClientPickup(collisionPub, posePub, pickupClient, targetPose, supportPose);
	pickupClient->cancelAllGoals();
	ros::Duration(3.0).sleep();


	ROS_INFO("Moving arm after pickup attempt");
	moveArmToPose(RIGHT_ARM, GraspingUtils::genPose(0.35, -0.5, 1.1), "base_footprint");


	ROS_INFO("Routine completed");
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
