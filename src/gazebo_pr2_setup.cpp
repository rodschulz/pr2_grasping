/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <pr2_grasping/GazeboSetup.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/node/impl.h>
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
std::pair<std::string, geometry_msgs::Pose> state;
YAML::Node models;


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
	armsGroup.setPoseReferenceFrame(FRAME_BASE);

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

	// Stop any previous movement
	armsGroup.stop();
	ros::Duration(1.0).sleep();

	// plan the trajectory
	moveit::planning_interface::MoveGroup::Plan armsPlan;
	ROS_INFO("...planing arms trajectory");

	int counter = 0;
	while (!armsGroup.plan(armsPlan))
	{
		ROS_INFO(".....planning failed, retrying");
		ros::Duration(0.5).sleep();
		armsGroup.setPoseTarget(armsGroup.getRandomPose());
		armsGroup.move();

		ros::Duration(0.5).sleep();
		armsGroup.setPoseTarget(rightArmPose, "r_wrist_roll_link");
		armsGroup.setPoseTarget(leftArmPose, "l_wrist_roll_link");

		if (++counter > 25)
		{
			ROS_INFO("...too many retries, stopping");
			break;
		}
	}
	ROS_INFO("...trajectory plan %s", planningOk ? "" : "FAILED");

	// move the arms
	if (planningOk)
	{
		ROS_INFO("...moving arms");
		armsGroup.move();
		ROS_INFO("...arms movement completed");
	}

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
	targetPoint.header.frame_id = FRAME_BASE;
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
void retrieveWorldState()
{
	std::vector<std::string> list = Config::get()["setup"]["ignore"].as<std::vector<std::string> >();
	std::map<std::string, bool> ignoreMap;
	for (std::vector<std::string>::const_iterator it = list.begin(); it != list.end(); it++)
		ignoreMap[*it] = true;

	// Retrieve the world's initial state
	std::map<std::string, geometry_msgs::Pose> initState;
	if (!GazeboUtils::getWorldState(initState, ignoreMap))
	{
		std::string msg = "Unable to retrieve initial world state";
		ROS_ERROR_STREAM(msg);
		throw std::runtime_error(msg.c_str());
	}
	ROS_INFO("Retrieved state for: ");
	for (std::map<std::string, geometry_msgs::Pose>::const_iterator it = initState.begin(); it != initState.end(); it++)
	{
		ROS_INFO_STREAM("..." << it->first);
		state.first = it->first;
		state.second = it->second;
		break;
	}
}


/**************************************************/
bool resetObject()
{
	bool resetOk = true;

	size_t n = state.first.find_last_of(':');
	std::string root = state.first.substr(0, n);
	int spawns = n == std::string::npos ? 0 : boost::lexical_cast<int>(state.first.substr(n + 1));

	ROS_INFO_STREAM("Deleting model " << state.first);
	if (!GazeboUtils::deleteModel(state.first))
	{
		ROS_WARN_STREAM("Can't delete model '" << state.first << "'");
		resetOk = false;
	}

	state.first = root + ":" + boost::lexical_cast<std::string>(spawns + 1);
	ROS_INFO_STREAM("Spawning model " << state.first);
	if (!GazeboUtils::spawnModel(state.first, models["models"][root].as<std::string>(), state.second))
	{
		ROS_WARN_STREAM("Can't spawn model '" << state.first << "'");
		resetOk = false;
	}

	return resetOk;
}


/**************************************************/
bool runSetup(pr2_grasping::GazeboSetup::Request  & request_,
			  pr2_grasping::GazeboSetup::Response & response_)
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
	if (request_.resetObject)
		respawnOk = resetObject();

	ROS_INFO("Gazebo setup routine finished");
	response_.result = respawnOk && armsOk;

	return true;
}


/**************************************************/
int main(int argn_, char** argv_)
{
	// Setup node
	ros::init(argn_, argv_, "gazebo_pr2_setup");
	ros::NodeHandle handler;

	// Load the node's configuration
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(GraspingUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + GraspingUtils::getConfigPath());

	// Load config data
	displacementThreshold = Config::get()["setup"]["displacementThreshold"].as<float>(1.0);
	models = YAML::LoadFile(ros::package::getPath(PACKAGE_NAME) + "/config/" + Config::get()["setup"]["modelsFile"].as<std::string>());
	retrieveWorldState();

	// Publisher and subscriber for base's movement
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
