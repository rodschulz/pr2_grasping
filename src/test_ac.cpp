/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>
#include "GraspingUtils.hpp"


#define TARGET_ID	"target"
#define SUPPORT_ID	"support"


typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> PickupClient;


/**********************************************************************/
void prepareGripper(GripperClient *client_,
					const float position_,
					const float maxEffort_)
{
	control_msgs::GripperCommandGoal cmd;
	cmd.command.position = position_;
	cmd.command.max_effort = maxEffort_;

	ROS_INFO("Sending gripper goal (p:%f, e:%f)", position_, maxEffort_);
	client_->cancelAllGoals();
	client_->sendGoal(cmd);

	ROS_INFO("Waiting to complete the action");

	// Send the goal until is reached (apparently this isn't necessary in the real robot)
	int count = 0;
	while (true)
	{
		count++;

		if (count % 5 == 0)
			ROS_INFO("...state %s", client_->getState().toString().c_str());

		if (client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Gripper action completed");
			break;
		}
		// client_->cancelAllGoals();
		client_->sendGoal(cmd);
		ros::Duration(0.2).sleep();
	}
}


/**********************************************************************/
void addObject(ros::Publisher &collisionPub_,
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
moveit_msgs::PickupGoal genPickGoal(const moveit::planning_interface::MoveGroup &group_,
									const moveit_msgs::Grasp &grasp_,
									const std::string &target_,
									const std::string &support_ = "")
{
	moveit_msgs::PickupGoal goal;

	goal.group_name = group_.getName();
	goal.end_effector = group_.getEndEffector();

	if (group_.getPathConstraints().name != std::string())
		goal.path_constraints = group_.getPathConstraints();

	goal.target_name = target_;
	if (support_ != "")
		goal.support_surface_name = support_;

	goal.planner_id = "RRTConnectkConfigDefault";
	goal.allowed_planning_time = 120;

	goal.planning_options.plan_only = false;
	goal.planning_options.replan = false;
	goal.planning_options.replan_delay = 3;
	goal.planning_options.replan_attempts = 5;

	goal.planning_options.planning_scene_diff.is_diff = true;
	goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

	goal.possible_grasps.push_back(grasp_);

	return goal;
}


/**********************************************************************/
moveit_msgs::Grasp genGrasp(const std::string &target_,
							const geometry_msgs::PoseStamped &graspPose_)
{
	moveit_msgs::Grasp grasp;

	grasp.id = "grasp_attempt";
	grasp.grasp_pose = graspPose_;

	grasp.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
	grasp.pre_grasp_approach.direction.vector.x = 1;
	grasp.pre_grasp_approach.direction.vector.y = 0;
	grasp.pre_grasp_approach.direction.vector.z = 0;
	grasp.pre_grasp_approach.min_distance = 0.05;
	grasp.pre_grasp_approach.desired_distance = 0.1;

	// grasp.pre_grasp_posture = GraspingUtils::generateGraspPosture(1.0, "right_gripper");
	// grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(60.0);

	// grasp.grasp_posture = GraspingUtils::generateGraspPosture(0.03, "right_gripper");
	// grasp.grasp_posture.points[0].time_from_start = ros::Duration(60.0);


	grasp.pre_grasp_posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
	grasp.pre_grasp_posture.points.resize(1);
	grasp.pre_grasp_posture.points[0].positions.resize(1);
	grasp.pre_grasp_posture.points[0].positions[0] = 1;
	grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(45.0);

	grasp.grasp_posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
	grasp.grasp_posture.points.resize(1);
	grasp.grasp_posture.points[0].positions.resize(1);
	grasp.grasp_posture.points[0].positions[0] = 0.02;
	grasp.grasp_posture.points[0].time_from_start = ros::Duration(45.0);




	grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint";
	grasp.post_grasp_retreat.direction.vector.x = 0;
	grasp.post_grasp_retreat.direction.vector.y = 0;
	grasp.post_grasp_retreat.direction.vector.z = 1;
	grasp.post_grasp_retreat.min_distance = 0.05;
	grasp.post_grasp_retreat.desired_distance = 0.3;

	grasp.allowed_touch_objects.clear();
	grasp.allowed_touch_objects.push_back(target_);
	// grasp.allowed_touch_objects.push_back("r_gripper_l_finger_tip_link");
	// grasp.allowed_touch_objects.push_back("r_gripper_r_finger_tip_link");
	// grasp.allowed_touch_objects.push_back("r_gripper_l_finger_link");
	// grasp.allowed_touch_objects.push_back("r_gripper_r_finger_link");
	// grasp.allowed_touch_objects.push_back("r_gripper_motor_screw_link");
	// grasp.allowed_touch_objects.push_back("r_gripper_motor_slider_link");

	return grasp;
}


/**********************************************************************/
void pick(PickupClient *client_,
		  const moveit::planning_interface::MoveGroup &group_,
		  const std::string &target_,
		  const geometry_msgs::PoseStamped &graspPose_)
{
	// Cancel any previous goal
	client_->cancelAllGoals();

	// Generate grasp and pick goal
	moveit_msgs::Grasp grasp = genGrasp(target_, graspPose_);
	moveit_msgs::PickupGoal goal = genPickGoal(group_, grasp, target_);

	// Send goal
	// while (true)
	{
		client_->sendGoal(goal);
		client_->waitForResult();

		ROS_INFO("Pick action state: %s", client_->getState().toString().c_str());

		if (client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Pick SUCCEEDED");
			// break;
		}
	}
}


/**********************************************************************/
int main(int argn_, char **argv_)
{
	ros::init(argn_, argv_, "test_action_client");
	ros::NodeHandle handle;
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// Set publishers
	ROS_INFO("Setting publishers");
	ros::Publisher posePub = handle.advertise<geometry_msgs::PoseStamped>("/test/grasp_pose", 10, true);
	ros::Publisher collisionPub = handle.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);


	// Setup clients action
	ROS_INFO("Setting action servers");
	// GripperClient *gripperClient = new GripperClient("r_gripper_controller/gripper_action", true);
	// while (!gripperClient->waitForServer(ros::Duration(5.0)))
	// ROS_INFO("Waiting for gripper action server");

	PickupClient *pickupClient = new PickupClient(move_group::PICKUP_ACTION, false);
	while (!pickupClient->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for pickup action server");


	// Set initial gripper position
	// ROS_INFO("Setting initial gripper position");
	// prepareGripper(gripperClient, 0.07, -1.0); //open: 0.08 - no limit effort: <0


	// Define the active group
	moveit::planning_interface::MoveGroup group("right_arm");
	group.setEndEffector("right_eef");
	group.setPlanningTime(60.0);
	ROS_INFO("Planning frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("End effector link: %s", group.getEndEffectorLink().c_str());


	// Move the arm to a initial pose
	ROS_INFO("Setting initial arm pose");
	group.setPoseTarget(GraspingUtils::genPose(0.3, -0.8, 0.9, DEG2RAD(0), 1, 0, 0));
	moveit::planning_interface::MoveGroup::Plan plan;
	if (group.plan(plan))
	{
		ROS_INFO("...moving arm");
		group.move();
	}


	// Add objects to the environment
	ROS_INFO("Adding objects to scene");
	geometry_msgs::PoseStamped objPose;
	objPose.header.stamp = ros::Time(0);
	objPose.header.frame_id = "base_footprint";
	objPose.pose = GraspingUtils::genPose(0.75, -0.3, 0.6, DEG2RAD(0), 0, 0, 1);
	addObject(collisionPub, TARGET_ID, objPose, 0.04, 0.04, 0.2);
	ros::Duration(1).sleep();


	ROS_INFO("Publishing grasping pose");
	geometry_msgs::PoseStamped graspPose = objPose;
	graspPose.pose.position.x -= 0.2;
	posePub.publish(graspPose);


	// ROS_INFO("Planning grasping pose");
	// group.setPoseTarget(graspPose);
	// group.plan(plan);


	// Attempt to pick an object
	ROS_INFO("Attempting pick");
	pick(pickupClient, group, TARGET_ID, graspPose);
	ROS_INFO("Pick attempt completed");


	group.setPoseTarget(GraspingUtils::genPose(0.3, -0.8, 0.9, DEG2RAD(0), 1, 0, 0));
	if (group.plan(plan))
	{
		ROS_INFO("...moving arm");
		group.move();
	}


	ROS_INFO("Test completed");

	ros::shutdown();
	return EXIT_SUCCESS;
}
