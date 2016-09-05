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


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_group_pick");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();



	moveit::planning_interface::MoveGroup group("right_arm");
	group.setEndEffector("right_eef");
	group.setPlanningTime(30.0);

	ros::Publisher posePublisher = node_handle.advertise<geometry_msgs::PoseStamped>("/test_pose", 10);
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::DisplayTrajectory display_trajectory;
	// std::vector<std::string> objs;
	// objs.push_back(TARGET_OBJECT);
	// planning_scene_interface.removeCollisionObjects(objs);


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
	moveit::planning_interface::MoveGroup::Plan my_plan;
	if (group.plan(my_plan))
	{
		group.move();
		sleep(10.0);
	}



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
	box_pose.position.x =  0.7;
	box_pose.position.y = -0.3;
	box_pose.position.z =  0.8;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

	// Now, let's add the collision object into the world
	ROS_INFO("Add an object into the world and waiting 10s");
	planning_scene_interface.addCollisionObjects(collision_objects);


	geometry_msgs::PoseStamped poseMsg;
	poseMsg.header.stamp = ros::Time::now();
	poseMsg.header.frame_id = collision_object.header.frame_id;
	poseMsg.pose = GraspingUtils::genPose(box_pose.position.x - 0.25,
										  box_pose.position.y,
										  box_pose.position.z,
										  DEG2RAD(0),
										  0,
										  0,
										  1);

	ROS_INFO("Publishing target pose and waiting 10s");
	posePublisher.publish(poseMsg);


	/*******************************************************************************/
	ROS_INFO("Preparing grasp");

	// gen the actual grasp
	moveit_msgs::Grasp grasp;
	grasp.id = "test_grasp";
	grasp.grasp_pose = poseMsg;

	// grasp.pre_grasp_approach.direction.header.frame_id = collision_object.header.frame_id;
	grasp.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
	grasp.pre_grasp_approach.direction.vector.x = 1;
	grasp.pre_grasp_approach.direction.vector.y = 0;
	grasp.pre_grasp_approach.direction.vector.z = 0;
	grasp.pre_grasp_approach.min_distance = 0.05;//0.08 is ok, 0.09 fails
	grasp.pre_grasp_approach.desired_distance = 0.15;

	grasp.pre_grasp_posture = GraspingUtils::generateGraspPosture(1.0, "right_gripper");

	ROS_INFO("*** Pre-grasp posture detail");
	for (size_t i = 0; i < grasp.pre_grasp_posture.joint_names.size(); i++)
		ROS_INFO_STREAM(grasp.pre_grasp_posture.joint_names[i]
						<< " - p: " << grasp.pre_grasp_posture.points[0].positions[i]
						<< " e: " << grasp.pre_grasp_posture.points[0].effort[i]);
	ROS_INFO("*****");

	grasp.grasp_posture = GraspingUtils::generateGraspPosture(0.01, "right_gripper");

	ROS_INFO("*** Grasp posture detail");
	for (size_t i = 0; i < grasp.grasp_posture.joint_names.size(); i++)
		ROS_INFO_STREAM(grasp.grasp_posture.joint_names[i]
						<< " - p: " << grasp.grasp_posture.points[0].positions[i]
						<< " e: " << grasp.grasp_posture.points[0].effort[i]);
	ROS_INFO("*****");


	grasp.post_grasp_retreat.direction.header.frame_id = collision_object.header.frame_id;
	grasp.post_grasp_retreat.direction.vector.x = 0;
	grasp.post_grasp_retreat.direction.vector.y = 0;
	grasp.post_grasp_retreat.direction.vector.z = 1;
	grasp.post_grasp_retreat.min_distance = 0.05;
	grasp.post_grasp_retreat.desired_distance = 0.2;

	grasp.allowed_touch_objects.clear();
	grasp.allowed_touch_objects.push_back(collision_object.id);

	ROS_INFO("*** Allowed touches");
	for (size_t i = 0; i < grasp.allowed_touch_objects.size(); i++)
		ROS_INFO_STREAM(grasp.allowed_touch_objects[i]);


	group.setPoseTarget(grasp.grasp_pose);
	if (group.plan(my_plan))
	{
		ROS_INFO("Showing plan");
		display_trajectory.trajectory_start = my_plan.start_state_;
		display_trajectory.trajectory.push_back(my_plan.trajectory_);
		display_publisher.publish(display_trajectory);
	}



	ROS_INFO("pick attempt");
	std::vector<moveit_msgs::Grasp> grasps;
	grasps.resize(20, grasp);
	// group.move();
	group.pick(collision_object.id, grasps);
	// group.pick(collision_object.id, grasp);
	// group.pick(collision_object.id);
	sleep(30);


	ROS_INFO("Detaching");
	group.detachObject(collision_object.id);
	sleep(4.0);
	ROS_INFO("Removing");
	std::vector<std::string> object_ids;
	object_ids.push_back(collision_object.id);
	planning_scene_interface.removeCollisionObjects(object_ids);
	sleep(4.0);


	ros::shutdown();
	return 0;
}
