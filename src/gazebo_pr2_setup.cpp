/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <pr2_grasping/GazeboSetup.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/node/impl.h>
#include "Config.hpp"
#include "GraspingUtils.hpp"
#include "GazeboUtils.hpp"
#include "RobotUtils.hpp"


/***** Global variables *****/
bool stopDisplacement = false;
float displacementThreshold = 1.0;
ros::Publisher cmdPublisher;
std::pair<std::string, geometry_msgs::Pose> state;
MoveGroupPtr arms;


/**************************************************/
void displacementCallback(const nav_msgs::Odometry::ConstPtr &msg_)
{
	if (!stopDisplacement && msg_->pose.pose.position.x >= displacementThreshold)
	{
		ROS_DEBUG("...base destination reached");
		stopDisplacement = true;
	}
}


/**************************************************/
void liftUpTorso()
{
	ROS_DEBUG("...moving torso");

	// define action client
	TorsoClient *torsoClient = new TorsoClient("torso_controller/position_joint_action", true);

	// wait for the action server to come up
	while (!torsoClient->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for action server to come up");

	ROS_INFO("...lifting torso");
	control_msgs::SingleJointPositionGoal torsoGoal;
	torsoGoal.position = Config::get()["setup"]["torsoPosition"].as<float>(0.18);
	torsoGoal.min_duration = ros::Duration(1.0);
	torsoGoal.max_velocity = 5.0;

	ROS_DEBUG("...sending goal");
	torsoClient->sendGoalAndWait(torsoGoal);

	ROS_INFO("...torso lifted up");
}


/**************************************************/
bool moveArms()
{
	ROS_DEBUG("...moving arms");

	// Stop any previous movement
	arms->stop();
	ros::Duration(1.0).sleep();

	Eigen::Quaternionf orientation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(1, 0, 0), Eigen::Vector3f(0, 0, 1));

	// right arm pose
	geometry_msgs::PoseStamped rightArm;
	rightArm.header.frame_id = FRAME_BASE;
	// rightArm.pose.orientation.w = 1.0;
	rightArm.pose.position.x = Config::get()["setup"]["armsPosition"]["right"]["x"].as<float>(0.3);
	rightArm.pose.position.y = Config::get()["setup"]["armsPosition"]["right"]["y"].as<float>(-0.5);
	rightArm.pose.position.z = Config::get()["setup"]["armsPosition"]["right"]["z"].as<float>(1.1);
	rightArm.pose.orientation.x = orientation.x();
	rightArm.pose.orientation.y = orientation.y();
	rightArm.pose.orientation.z = orientation.z();
	rightArm.pose.orientation.w = orientation.w();

	// left arm pose
	geometry_msgs::PoseStamped leftArm;
	leftArm.header.frame_id = FRAME_BASE;
	// leftArm.pose.orientation.w = 1.0;
	leftArm.pose.position.x = Config::get()["setup"]["armsPosition"]["left"]["x"].as<float>(0.3);
	leftArm.pose.position.y = Config::get()["setup"]["armsPosition"]["left"]["y"].as<float>(0.5);
	leftArm.pose.position.z = Config::get()["setup"]["armsPosition"]["left"]["z"].as<float>(1.1);
	leftArm.pose.orientation.x = orientation.x();
	leftArm.pose.orientation.y = orientation.y();
	leftArm.pose.orientation.z = orientation.z();
	leftArm.pose.orientation.w = orientation.w();


	// set the pose for each arm
	arms->setPoseTarget(rightArm, "r_wrist_roll_link");
	arms->setPoseTarget(leftArm, "l_wrist_roll_link");

	ROS_DEBUG("...attempting move");
	return RobotUtils::move(arms, 30);
}


/**************************************************/
void moveBase()
{
	ROS_DEBUG("...moving robot base");

	geometry_msgs::Twist cmd;
	cmd.linear.x = 4;
	cmd.linear.y = cmd.linear.z = 0;
	cmd.angular.x = cmd.angular.y = cmd.angular.z = 0;

	ROS_INFO("...sending displacement messages");
	while (ros::ok() && !stopDisplacement)
		cmdPublisher.publish(cmd);

	ROS_INFO("...displacement completed");
}


/**************************************************/
void moveHead()
{
	ROS_DEBUG("...moving head");

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME ".actionlib", ros::console::levels::Info))
		ros::console::notifyLoggerLevelsChanged();

	RobotUtils::moveHead(Config::get()["setup"]["headTarget"]["x"].as<float>(0.9),
						 Config::get()["setup"]["headTarget"]["y"].as<float>(0),
						 Config::get()["setup"]["headTarget"]["z"].as<float>(0.5),
						 FRAME_BASE);

	ROS_INFO("...head moved");
}


/**************************************************/
void retrieveWorldState()
{
	ROS_DEBUG("...retrieving world state");

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
	ROS_INFO("Reseting target object state");

	bool resetOk = true;
	if (!GazeboUtils::setModelState(state.first, state.second, "world"))
	{
		ROS_WARN_STREAM("Can't reset state of model '" << state.first << "'");
		resetOk = false;
	}

	return resetOk;
}


/**************************************************/
bool runSetup(pr2_grasping::GazeboSetup::Request  & request_,
			  pr2_grasping::GazeboSetup::Response & response_)
{
	ROS_INFO("Beginning setup routine...");

	if (Config::get()["setup"]["liftTorso"].as<bool>())
		liftUpTorso();

	bool armsOk = true;
	if (Config::get()["setup"]["moveArms"].as<bool>())
		moveArms();

	if (Config::get()["setup"]["moveBase"].as<bool>())
		moveBase();

	if (Config::get()["setup"]["moveHead"].as<bool>())
		moveHead();

	bool resetOk = resetObject();

	response_.result = resetOk && armsOk;
	response_.trackedObject = state.first;
	ROS_INFO("Setup routine %s...", response_.result ? "SUCCESSFUL" : "FAILED");

	return true;
}


/**************************************************/
int main(int argn_, char** argv_)
{
	ros::init(argn_, argv_, "gazebo_pr2_setup");
	ros::NodeHandle handler;


	/********** Start spinning **********/
	ros::AsyncSpinner spinner(3);
	spinner.start();


	/********** Load the node's configuration **********/
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(GraspingUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + GraspingUtils::getConfigPath());

	bool debugEnabled = Config::get()["setupDebug"].as<bool>();
	displacementThreshold = Config::get()["setup"]["displacementThreshold"].as<float>();

	if (debugEnabled)
	{
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
			ros::console::notifyLoggerLevelsChanged();
	}


	/********** Retrieve the initial world state **********/
	retrieveWorldState();


	/********** Set subscriptions/publishers **********/
	cmdPublisher = handler.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
	ros::Subscriber poseSubscriber = handler.subscribe("/base_pose_ground_truth", 1, displacementCallback);


	/********** Set services **********/
	ROS_INFO("Starting setup service");
	arms = MoveGroupPtr(new moveit::planning_interface::MoveGroup("arms"));
	arms->setPoseReferenceFrame(FRAME_BASE);
	ros::ServiceServer setupService = handler.advertiseService("/pr2_grasping/gazebo_setup", runSetup);


	ros::waitForShutdown();
	return EXIT_SUCCESS;
}
