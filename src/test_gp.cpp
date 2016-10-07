/**
 * Author: rodrigo
 * 2016
 */
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <boost/foreach.hpp>
#include <GraspingUtils.hpp>
#include <ros/ros.h>


#define TARGET_OBJECT	"test_box"


int main(int argn_, char **argv_)
{
	ros::init(argn_, argv_, "test_group_pick");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();



	moveit::planning_interface::MoveGroup group("right_arm");
	group.setEndEffector("right_eef");
	group.setPlanningTime(60.0);


	ros::Publisher posePublisher = node_handle.advertise<geometry_msgs::PoseStamped>("/test/grasp_pose", 10, true);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::DisplayTrajectory display_trajectory;


	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());


	// Planning to a Pose goal
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.3;
	target_pose1.position.y = -0.7;
	target_pose1.position.z = 0.7;
	group.setPoseTarget(target_pose1);


	// Now, we call the planner to compute the plan and visualize it.
	moveit::planning_interface::MoveGroup::Plan plan;
	if (group.plan(plan))
		group.move();


	// Adding/Removing Objects and Attaching/Detaching Objects
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = group.getPlanningFrame();

	/* The id of the object is used to identify it. */
	collision_object.id = TARGET_OBJECT;

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.04;
	primitive.dimensions[1] = 0.04;
	primitive.dimensions[2] = 0.2;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x =  0.6;
	box_pose.position.y = -0.3;
	box_pose.position.z =  0.8;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

	// Now, let's add the collision object into the world
	ROS_INFO("Add an object into the world and waiting");
	planning_scene_interface.addCollisionObjects(collision_objects);
	ros::Duration(3.0).sleep();


	geometry_msgs::PoseStamped poseMsg;
	poseMsg.header.stamp = ros::Time::now();
	poseMsg.header.frame_id = collision_object.header.frame_id;
	poseMsg.pose = GraspingUtils::genPose(box_pose.position.x,
										  box_pose.position.y - 0.2,
										  box_pose.position.z,
										  DEG2RAD(90),
										  0,
										  0,
										  1);

	ROS_INFO("Publishing target pose");
	posePublisher.publish(poseMsg);


	/*******************************************************************************/
	ROS_INFO("Preparing grasp");

	// gen the actual grasp
	moveit_msgs::Grasp grasp;
	grasp.id = "test_grasp";
	grasp.grasp_pose = poseMsg;


	grasp.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
	grasp.pre_grasp_approach.direction.vector.x = 1;
	grasp.pre_grasp_approach.direction.vector.y = 0;
	grasp.pre_grasp_approach.direction.vector.z = 0;
	grasp.pre_grasp_approach.min_distance = 0.05;
	grasp.pre_grasp_approach.desired_distance = 0.1;


	// grasp.pre_grasp_posture = GraspingUtils::generateGraspPosture(1.0, "right_gripper");
	grasp.pre_grasp_posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
	grasp.pre_grasp_posture.points.resize(1);
	grasp.pre_grasp_posture.points[0].positions.resize(1);
	grasp.pre_grasp_posture.points[0].positions[0] = 1;
	grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(45.0);


	// grasp.grasp_posture = GraspingUtils::generateGraspPosture(0.01, "right_gripper");
	grasp.grasp_posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
	grasp.grasp_posture.points.resize(1);
	grasp.grasp_posture.points[0].positions.resize(1);
	grasp.grasp_posture.points[0].positions[0] = 0;
	grasp.grasp_posture.points[0].time_from_start = ros::Duration(45.0);


	grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint";
	grasp.post_grasp_retreat.direction.vector.x = 0;
	grasp.post_grasp_retreat.direction.vector.y = 0;
	grasp.post_grasp_retreat.direction.vector.z = 1;
	grasp.post_grasp_retreat.min_distance = 0.05;
	grasp.post_grasp_retreat.desired_distance = 0.2;

	grasp.allowed_touch_objects.clear();
	grasp.allowed_touch_objects.push_back(collision_object.id);


	group.setPoseTarget(grasp.grasp_pose);
	group.setPlannerId("RRTConnectkConfigDefault");
	// group.plan(plan);



	ROS_INFO("pick attempt");
	std::vector<moveit_msgs::Grasp> grasps;
	grasps.push_back(grasp);
	group.pick(collision_object.id, grasps);
	// group.pick(collision_object.id, grasp);
	// group.pick(collision_object.id);
	ros::Duration(1).sleep();


	// Move the object
	group.setPoseTarget(GraspingUtils::genPose(0.15, -0.5, 0.3, DEG2RAD(90), 0, 1, 0));
	if (group.plan(plan))
		group.move();
	ros::Duration(1).sleep();


	ROS_INFO("Detaching");
	group.detachObject(collision_object.id);
	ros::Duration(1.0).sleep();

	ROS_INFO("Removing");
	std::vector<std::string> object_ids;
	object_ids.push_back(collision_object.id);
	planning_scene_interface.removeCollisionObjects(object_ids);
	ros::Duration(1.0).sleep();


	ROS_INFO("Test finished");
	ros::Duration(1.0).sleep();
	ros::shutdown();
	return 0;
}
