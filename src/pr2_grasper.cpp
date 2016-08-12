/**
 * Author: rodrigo
 * 2016     
 */
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <boost/signals2/mutex.hpp>
#include <queue>

// Pointer to a move group interface
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

// Queue of grasping points
std::queue<int> pointsQueue;
// Mutex for concurrent access to the queue
boost::mutex mutex;
// Manipulation group
MoveGroupPtr rightArm;


void graspingPointsCallback(const std_msgs::Float32MultiArrayConstPtr &msg_)
{
	ROS_INFO("Points received");
//
//	int npoints = msg_->layout.dim[0].size;
//	int step = msg_->layout.dim[0].stride;

	static int kk = 0;


	mutex.lock();
	pointsQueue.push(kk++);
	mutex.unlock();

//	use average normal direction to set the gripper orientation???






//	// right arm pose
//	geometry_msgs::Pose rightArmPose;
//	rightArmPose.orientation.w = 1.0;
//	rightArmPose.position.x = 0.3;
//	rightArmPose.position.y = -0.5;
//	rightArmPose.position.z = 1.4;
//
//	// left arm pose
//	geometry_msgs::Pose leftArmPose;
//	leftArmPose.orientation.w = 1.0;
//	leftArmPose.position.x = 0.3;
//	leftArmPose.position.y = 0.5;
//	leftArmPose.position.z = 1.4;
//
//	// set the pose for each arm
//	armsGroup.setPoseTarget(rightArmPose, "r_wrist_roll_link");
//	armsGroup.setPoseTarget(leftArmPose, "l_wrist_roll_link");
//
//	// plan the trajectory
//	moveit::planning_interface::MoveGroup::Plan armsPlan;
//	ROS_INFO("...planing arms trajectory");
//	bool planningOk = armsGroup.plan(armsPlan);
//	ROS_INFO("...trajectory plan %s", planningOk ? "SUCCESSFUL" : "FAILED");
//
//	// move the arms
//	if (planningOk)
//	{
//		ROS_INFO("...moving arms");
//		armsGroup.move();
//	}
//
//	ROS_INFO("...arms movement completed");
}

void timerCallback(const ros::TimerEvent& event)
{
	while(!pointsQueue.empty())
	{
		mutex.lock();

		ROS_INFO("Popping stuff!");
		pointsQueue.pop();

		mutex.unlock();
	}
}

int main(int _argn, char **_argv)
{
	ros::init(_argn, _argv, "pr2_grasper");
	ros::NodeHandle nodeHandler;

	// group to plane movement for both arms
	ROS_INFO("Seting up arms control");
	rightArm = MoveGroupPtr(new moveit::planning_interface::MoveGroup("right_arm"));


	ros::Timer timer = nodeHandler.createTimer(ros::Duration(1), timerCallback);

	// Set the subscription to get the point clouds
	ROS_INFO("Subscriber set");
	ros::Subscriber sub = nodeHandler.subscribe("/pr2_grasping/grasping_points", 1, graspingPointsCallback);

	// Keep looping
	ROS_INFO("Grasper node looping");
	ros::spin();

//	ros::AsyncSpinner spinner(1);
//	spinner.start();

//	while(true)
//	{
//		while(!pointsQueue.empty())
//		{
//			ROS_INFO("Popping stuff!");
//			pointsQueue.pop();
//		}
//
//		ROS_INFO("Sleeping");
//		sleep(5);
//	}

//	ROS_INFO("Setup routine completed");
//	ros::shutdown();

	return EXIT_SUCCESS;
}
