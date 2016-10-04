/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
// #include <gazebo_msgs/GetWorldProperties.h>
// #include <gazebo_msgs/DeleteModel.h>
// #include <gazebo_msgs/SpawnModel.h>
#include <pr2_grasping/GazeboSetup.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "Config.hpp"
#include "GraspingUtils.hpp"
#include "GazeboUtils.hpp"


/***** Client types definitions *****/
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> HeadClient;


/***** Global variables *****/
bool stopDisplacement = false;
float displacementThreshold = 1.0;
ros::Publisher cmdPublisher;
std::map<std::string, bool> ignoreMap;
std::map<std::string, gazebo_msgs::GetModelStateResponse> initialState


/**************************************************/
void displacementCallback(const nav_msgs::Odometry::ConstPtr &msg_)
{
	if (!stopDisplacement && msg_->pose.pose.position.x >= displacementThreshold)
	{
		ROS_INFO("Destination reached");
		stopDisplacement = true;
	}
}


/**************************************************/
void liftUpTorso()
{
	// define action client
	TorsoClient *torsoClient = new TorsoClient("torso_controller/position_joint_action", true);

	// wait for the action server to come up
	while (!torsoClient->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for action server to come up");

	ROS_INFO("Lifting up torso");

	pr2_controllers_msgs::SingleJointPositionGoal torsoGoal;
	torsoGoal.position = Config::get()["setup"]["torsoPosition"].as<float>(0.18);
	torsoGoal.min_duration = ros::Duration(1.0);
	torsoGoal.max_velocity = 5.0;

	ROS_INFO("...sending torso goal");
	torsoClient->sendGoal(torsoGoal);
	torsoClient->waitForResult();

	ROS_INFO("...torso lifted up");
}


/**************************************************/
bool moveArms()
{
	ROS_INFO("Preparing arms setup");

	// group to plane movement for both arms
	moveit::planning_interface::MoveGroup armsGroup("arms");
	armsGroup.setPoseReferenceFrame("base_link");

	// right arm pose
	geometry_msgs::Pose rightArmPose;
	rightArmPose.orientation.w = 1.0;
	rightArmPose.position.x = Config::get()["setup"]["armsPosition"]["right"]["x"].as<float>(0.3);
	rightArmPose.position.y = Config::get()["setup"]["armsPosition"]["right"]["y"].as<float>(-0.5);
	rightArmPose.position.z = Config::get()["setup"]["armsPosition"]["right"]["z"].as<float>(1.1);

	// left arm pose
	geometry_msgs::Pose leftArmPose;
	leftArmPose.orientation.w = 1.0;
	leftArmPose.position.x = Config::get()["setup"]["armsPosition"]["left"]["x"].as<float>(0.3);
	leftArmPose.position.y = Config::get()["setup"]["armsPosition"]["left"]["y"].as<float>(0.5);
	leftArmPose.position.z = Config::get()["setup"]["armsPosition"]["left"]["z"].as<float>(1.1);

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

	return planningOk;
}


/**************************************************/
void moveBase()
{
	ROS_INFO("Moving robot base");

	geometry_msgs::Twist cmd;
	cmd.linear.x = 4;
	cmd.linear.y = cmd.linear.z = 0;
	cmd.angular.x = cmd.angular.y = cmd.angular.z = 0;

	while (ros::ok() && !stopDisplacement)
		cmdPublisher.publish(cmd);

	ROS_INFO("...displacement completed");
}


/**************************************************/
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
	targetPoint.point.x = Config::get()["setup"]["headTarget"]["x"].as<float>(0.9);
	targetPoint.point.y = Config::get()["setup"]["headTarget"]["y"].as<float>(0.0);
	targetPoint.point.z = Config::get()["setup"]["headTarget"]["z"].as<float>(0.5);

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


/**************************************************/
bool respawnObject()
{
	std::vector<std::string> objects2;
	GazeboUtils::getWorldObjects(objects2, ignoreMap);

	gazebo_msgs::GetWorldProperties props;
	if (ros::service::call("/gazebo/get_world_properties", props))
	{
		// Extract the objects to re-spawn
		std::vector<std::string> objects;
		for (std::vector<std::string>::const_iterator it = props.response.model_names.begin(); it != props.response.model_names.end(); it++)
		{
			if (ignoreMap.find(*it) == ignoreMap.end())
				objects.push_back(*it);
			else
				ROS_DEBUG("Ignoring object %s", it->c_str());
		}

		// Re-spawn the extracted objects
		for (std::vector<std::string>::const_iterator it = objects.begin(); it != objects.end(); it++)
		{
			ROS_INFO("Re-spawning %s", it->c_str());

			// Remove model
			gazebo_msgs::DeleteModel remove;
			remove.request.model_name = *it;
			if (ros::service::call("/gazebo/delete_model", remove))
			{
				ROS_DEBUG("...model delete %s", remove.response.success ? "SUCCESSFUL" : "FAILED");
				if (!remove.response.success)
					return false;
			}

			// Spawn new model
			gazebo_msgs::SpawnModel spawn;
			spawn.request.model_name = *it;
			spawn.request.model_xml = "2";
			if (ros::service::call("/gazebo/delete_model", spawn))
				ROS_DEBUG("...model spawn %s", spawn.response.success ? "SUCCESSFUL" : "FAILED");

		}
	}

	return true;
}


/**************************************************/
bool runSetup(pr2_grasping::GazeboSetup::Request  &request_,
			  pr2_grasping::GazeboSetup::Response &response_)
{
	ROS_INFO("Beginning gazebo setup routine");

	if (Config::get()["setup"]["liftTorso"].as<bool>())
		liftUpTorso();

	bool armsOk = true;
	if (Config::get()["setup"]["moveArms"].as<bool>())
		moveArms();

	if (Config::get()["setup"]["moveBase"].as<bool>())
		moveBase();

	if (Config::get()["setup"]["moveHead"].as<bool>())
		moveHead();

	bool respawnOk = true;
	if (request_.respawnObject)
		respawnOk = respawnObject();

	ROS_INFO("Gazebo setup routine finished");
	response_.result = respawnOk && armsOk;

	return true;
}


/**************************************************/
void storeInitialWorldState()
{
	
}


/**************************************************/
int main(int argn_, char** argv_)
{
	// setup node
	ros::init(argn_, argv_, "gazebo_pr2_setup");

	// Load the node's configuration
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(GraspingUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + GraspingUtils::getConfigPath());

	// Load config data
	displacementThreshold = Config::get()["setup"]["displacementThreshold"].as<float>(1.0);
	std::vector<std::string> list = Config::get()["setup"]["ignoreList"].as<std::vector<std::string> >();
	for (std::vector<std::string>::const_iterator it = list.begin(); it != list.end(); it++)
		ignoreMap[*it] = true;

	// Retrieve the world's initial state
	if (!GazeboUtils::getWorldState(initialState, ignoreMap))
	{
		ROS_ERROR("Unnable to retrieve initial world state");
		throw std::runtime_error("Unnable to retrieve initial world state");
	}

	// Publisher and subscriber for base's movement
	ros::NodeHandle handler;
	cmdPublisher = handler.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
	ros::Subscriber poseSubscriber = handler.subscribe("/base_pose_ground_truth", 1, displacementCallback);

	// Service for setup configuration
	ros::ServiceServer setupService = handler.advertiseService("/pr2_grasping/gazebo_setup", runSetup);

	ROS_INFO("Starting setup service");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
