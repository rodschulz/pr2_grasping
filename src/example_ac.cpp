/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>
#include "GraspingUtils.hpp"

#define TARGET_ID	"target"
#define SUPPORT_ID	"support"

typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> PickupClient;

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
int main(int argn_, char **argv_)
{
	ros::init(argn_, argv_, "example_action_client");
	ros::NodeHandle handle;
	ros::AsyncSpinner spinner(2);
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
		group.move();
	else
		ROS_INFO("Plan failed, unnable to set initial arm's position");



	return EXIT_SUCCESS;
}
