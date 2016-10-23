/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <utility>
#include <string>
#include <boost/algorithm/string.hpp>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/SingleJointPositionAction.h>
#include <control_msgs/PointHeadAction.h>
#include <moveit_msgs/PickupAction.h>
#include <control_msgs/GripperCommandAction.h>

enum Effector
{
	RIGHT_ARM,
	LEFT_ARM
};


/***** Robot interesting frames *****/
#define FRAME_KINNECT		"head_mount_kinect_ir_optical_frame"
#define FRAME_BASE			"base_footprint"
#define FRAME_R_GRIPPER		"r_gripper_tool_frame"
#define FRAME_L_GRIPPER		"l_gripper_tool_frame"

/***** Gripper related definitions *****/
#define GRIPPER_OPEN			0.086
#define GRIPPER_CLOSED			0.0
#define GAP_CONVERSION_RATIO	0.1714
#define R_GRIPPER_JOINT			"r_gripper_motor_screw_joint"
#define L_GRIPPER_JOINT			"l_gripper_motor_screw_joint"



/***** Move group pointer *****/
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

/***** Client types definitions *****/
typedef actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> HeadClient;
typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> PickupClient;
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;


// Class implementing utilities for robot interaction
class RobotUtils
{
public:
	/**************************************************/
	static std::pair<std::string, std::string> getEffectorNames(const Effector &arm_);

	/**************************************************/
	static std::pair<std::string, std::string> getEffectorNames(const std::string &arm_);

	/**************************************************/
	static std::string getEffectorFrame(const std::string &arm_);

	/**************************************************/
	static std::string getGripperTopic(const std::string &arm_);

	/**************************************************/
	static std::string getArmTopic(const std::string &arm_);

	/**************************************************/
	static bool planAndMove(MoveGroupPtr &group_,
							const int maxRetries_ = -1);

	/**************************************************/
	static bool move(MoveGroupPtr &group_,
					 const geometry_msgs::PoseStamped &pose_,
					 const int maxRetries_ = -1);

	/**************************************************/
	static void moveHead(const float x_,
						 const float y_,
						 const float z_);

	/**************************************************/
	static float getPR2GripperJointOpening(const float gap_);

	/**************************************************/
	static actionlib::SimpleClientGoalState moveGripper(const std::string arm_,
			const float position_,
			const float maxEffort_ = -1);

private:
	// Constructor
	RobotUtils();
	// Destructor
	~RobotUtils();
};
