/**
 * Author: rodrigo
 * 2016     
 */
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <boost/signals2/mutex.hpp>
#include <deque>

// Pointer to a move group interface
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

// Structure to store a grasping point
struct GraspPt
{
	float x;
	float y;
	float z;
	int label;

	GraspPt(const float x_, const float y_, const float z_, const int label_)
	{
		x = x_;
		y = y_;
		z = z_;
		label = label_;
	}
};

// Queue of grasping points
std::deque<GraspPt> ptsQueue;
// Mutex for concurrent access to the queue
boost::mutex mutex;
// Manipulation group
MoveGroupPtr rightArm;

void graspingPointsCallback(const std_msgs::Float32MultiArrayConstPtr &msg_)
{
	ROS_INFO("Points received");

	// Prevent queue from growing too much
	if (ptsQueue.size() <= 20)
	{
		int npoints = msg_->layout.dim[0].size;
		int step = msg_->layout.dim[0].stride;

		for (int i = 0; i < npoints; i++)
		{
			int index = i * step;\
			// Add the new point to the queue
			mutex.lock();
			ptsQueue.push_back(GraspPt(msg_->data[index], msg_->data[index + 1], msg_->data[index + 2], msg_->data[index + 3]));
			mutex.unlock();
		}
	}
	else
		ROS_INFO("Queue full, discarding...");
}

void timerCallback(const ros::TimerEvent& event)
{
	while (!ptsQueue.empty())
	{
		ROS_INFO("...processing point (%.3f, %.3f, %.3f, %d)", ptsQueue.front().x, ptsQueue.front().y, ptsQueue.front().z, ptsQueue.front().label);

		rightArm->setPoseReferenceFrame("head_mount_kinect_ir_optical_frame");
//		rightArm->setPoseReferenceFrame("base_link");

		ROS_INFO("...ref frame: %s", rightArm->getPlanningFrame().c_str());
		ROS_INFO("...efector: %s - efector link: %s", rightArm->getEndEffector().c_str(), rightArm->getEndEffectorLink().c_str());



		// right arm pose
		geometry_msgs::Pose armPose;
		armPose.orientation.w = 1.0;
		armPose.position.x = ptsQueue.front().x;
		armPose.position.y = ptsQueue.front().y;
		armPose.position.z = ptsQueue.front().z;
//		armPose.position.x = 0.75;
//		armPose.position.y = 0;
//		armPose.position.z = 0.77;

		// Pop a point from the queue
		mutex.lock();
		ptsQueue.pop_front();
		mutex.unlock();

		// Set arm's pose
//		rightArm->setPoseTarget(armPose, "r_wrist_roll_link");
		rightArm->setPoseTarget(armPose);
//		rightArm->setPoseTarget(armPose, "r_gripper_l_finger_link");

		// Plan the trajectory
		moveit::planning_interface::MoveGroup::Plan armPlan;
		ROS_INFO("...planing arm trajectory");
		bool planningOk = rightArm->plan(armPlan);
		ROS_INFO("...trajectory plan %s", planningOk ? "SUCCESSFUL" : "FAILED");

		// Move the arm
		if (planningOk)
		{
			ROS_INFO("...moving right arm");
			rightArm->move();
		}

		ROS_INFO("...arm movement completed");
	}
}

int main(int _argn, char **_argv)
{
	ros::init(_argn, _argv, "pr2_grasper");
	ros::NodeHandle nodeHandler;

	// group to plane movement for both arms
	ROS_INFO("Setting right arm control");
	rightArm = MoveGroupPtr(new moveit::planning_interface::MoveGroup("right_arm"));
//	rightArm = MoveGroupPtr(new moveit::planning_interface::MoveGroup("right_eef"));
//	rightArm = MoveGroupPtr(new moveit::planning_interface::MoveGroup("right_gripper"));

	// Set a timer to process queued grasping points
	ros::Timer timer = nodeHandler.createTimer(ros::Duration(1), timerCallback);

	// Set the subscription to get the point clouds
	ROS_INFO("Subscriber set");
	ros::Subscriber sub = nodeHandler.subscribe("/pr2_grasping/grasping_points", 1, graspingPointsCallback);

	// Keep looping
	ROS_INFO("Grasper node looping");
	ros::spin();

	return EXIT_SUCCESS;
}
