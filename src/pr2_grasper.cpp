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
#include "GraspingUtils.hpp"

#define PRINT_POSE(msg, point, frame)	ROS_INFO("..." msg "= (%.2f, %.2f, %.2f) - ref: %s", (point).x, (point).y, (point).z, (frame).c_str())


// Pointer to a move group interface
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

// Structure to store a grasping point
struct GraspPt
{
	geometry_msgs::Point point;
	int label;

	GraspPt(const float x_, const float y_, const float z_, const int label_)
	{
		point.x = x_;
		point.y = y_;
		point.z = z_;
		label = label_;
	}
};

// Queue of grasping points
std::deque<GraspPt> queue;
// Mutex for concurrent access to the queue
boost::mutex mutex;
// Manipulation group
MoveGroupPtr arm;
//
ros::Publisher posePublisher;
//
tf::TransformListener *tfListener;


void graspingPointsCallback(const std_msgs::Float32MultiArrayConstPtr &msg_)
{
	ROS_INFO("Points received");

	// Prevent queue from growing endlessly
	if (queue.size() <= 20)
	{
		int npoints = msg_->layout.dim[0].size;
		int step = msg_->layout.dim[0].stride;

		for (int i = 0; i < npoints; i++)
		{
			int index = i * step;
			// Add the new point to the queue
			mutex.lock();
			queue.push_back(GraspPt(msg_->data[index], msg_->data[index + 1], msg_->data[index + 2], msg_->data[index + 3]));
			mutex.unlock();
		}
	}
	else
		ROS_INFO("Queue full, discarding...");
}

void timerCallback(const ros::TimerEvent& event)
{
	while (!queue.empty())
	{
		ROS_INFO("***** p=(%.2f, %.2f, %.2f) + l=%d *****", queue.front().point.x, queue.front().point.y, queue.front().point.z, queue.front().label);

		std::string targetFrame = FRAME_BASE;
		arm->setEndEffector("right_eef");
		arm->setPoseReferenceFrame(targetFrame);
		ROS_INFO("...plan frame: %s - pose frame: %s", arm->getPlanningFrame().c_str(), arm->getPoseReferenceFrame().c_str());

		// Generate the target pose
		geometry_msgs::Pose target = GraspingUtils::genPose(0.546, 0.011, 0.77, DEG2RAD(45), 1, 1, 0);
//		geometry_msgs::Pose armPose = GraspingUtils::genPose(queue.front().x, queue.front().y, queue.front().z, DEG2RAD(225), 0, 1, 1);
//		armPose.position.x = queue.front().x;
//		armPose.position.y = queue.front().y;
//		armPose.position.z = queue.front().z;
//		armPose.orientation.w = 1.0;

		// Print positions
		geometry_msgs::PoseStamped current = arm->getCurrentPose();
		PRINT_POSE("curr", current.pose.position, current.header.frame_id);
		geometry_msgs::Point transformed = GraspingUtils::transformPose(tfListener, targetFrame, current.header.frame_id, current.pose.position);
		PRINT_POSE("curr", transformed, targetFrame);
		PRINT_POSE("tget", target.position, targetFrame);
		transformed = GraspingUtils::transformPose(tfListener, current.header.frame_id, targetFrame, target.position);
		PRINT_POSE("tget", transformed, current.header.frame_id);

		// Publish the target pose so it can be visualized using RViz
		geometry_msgs::PoseStamped msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = targetFrame;
		msg.pose = target;
		posePublisher.publish(msg);

		// Pop a point from the queue
		mutex.lock();
		queue.pop_front();
		mutex.unlock();

		arm->setPoseTarget(msg);

		// Plan the trajectory
		moveit::planning_interface::MoveGroup::Plan armPlan;
		ROS_INFO("...planing arm trajectory");
		bool planningOk = arm->plan(armPlan);
		ROS_INFO("...trajectory plan %s", planningOk ? "SUCCESSFUL" : "FAILED");

		// Move the arm
		if (planningOk)
		{
			ROS_INFO("...moving right arm");
			arm->move();
			ROS_INFO("...arm movement completed");

			current = arm->getCurrentPose();
			PRINT_POSE("curr", current.pose.position, current.header.frame_id);
			geometry_msgs::Point transformed = GraspingUtils::transformPose(tfListener, targetFrame, current.header.frame_id, current.pose.position);
			PRINT_POSE("curr", transformed, targetFrame);
		}
		else
			ROS_INFO("...movement aborted");
	}
}

int main(int _argn, char **_argv)
{
	ros::init(_argn, _argv, "pr2_grasper");
	ros::NodeHandle nodeHandler;
	tfListener = new tf::TransformListener(ros::Duration(10.0));

	// group to plane movement for both arms
	ROS_INFO("Setting right arm control");
	arm = MoveGroupPtr(new moveit::planning_interface::MoveGroup("right_arm"));

	// Set a timer to process queued grasping points
	ros::Timer timer = nodeHandler.createTimer(ros::Duration(1), timerCallback);

	// Debug publisher
	posePublisher = nodeHandler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/grasping_pose", 1);

	// Set the subscription to get the point clouds
	ROS_INFO("Subscriber set");
	ros::Subscriber sub = nodeHandler.subscribe("/pr2_grasping/grasping_points", 1, graspingPointsCallback);

	// Keep looping
	ROS_INFO("Grasper node looping");
	ros::AsyncSpinner spinner(2); // Use 2 threads
	spinner.start();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
