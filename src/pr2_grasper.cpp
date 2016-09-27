/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <pr2_grasping/GraspingPoint.h>
#include <pr2_grasping/GraspingPointArray.h>
#include <boost/signals2/mutex.hpp>
#include "boost/bind.hpp"
#include <deque>
#include "Config.hpp"
#include "GraspingUtils.hpp"
#include "RobotUtils.hpp"


#define PRINT_POSE(msg, point, frame)	ROS_INFO("..." msg "= (%.2f, %.2f, %.2f) - ref: %s", (point).x, (point).y, (point).z, (frame).c_str())


// Pointer to a move group interface
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

// // Structure to store a grasping point
// struct GraspPt
// {
// 	geometry_msgs::Point point;
// 	int label;

// 	GraspPt(const float x_, const float y_, const float z_, const int label_)
// 	{
// 		point.x = x_;
// 		point.y = y_;
// 		point.z = z_;
// 		label = label_;
// 	}
// };


// Queue of grasping points
// std::deque<GraspPt> queue;
std::deque<pr2_grasping::GraspingPointArray> queue;
// Max length allowed for the message queue
unsigned int queueMaxsize = 5;
// Mutex for concurrent access to the queue
boost::mutex mutex;
// // Manipulation group
// MoveGroupPtr effector;
// // Publisher for debug/inspection purposes
// ros::Publisher posePublisher;
// // TF listener for debug/inspection purposes
// tf::TransformListener *tfListener;


/**************************************************/
void graspingPointsCallback(const pr2_grasping::GraspingPointArrayConstPtr &msg_)
{
	// Prevent queue from growing endlessly
	if (queue.size() < queueMaxsize)
	{
		ROS_INFO("New points received");

		// Add the new point to the queue
		mutex.lock();
		queue.push_back(*msg_);
		mutex.unlock();
	}
	else
		ROS_INFO_ONCE("Message queue full at size %zu, discarding...", queue.size());
}


/**************************************************/
void tt(const ros::TimerEvent &event_, std::string _caca
		//const MoveGroupPtr &effector_
		//,
		//tf::TransformListener *tfListener_
		//,
		//const ros::Publisher &posePublisher_
	   )
{
	while (!queue.empty())
	{
		/**ROS_INFO("***** p=(%.2f, %.2f, %.2f) + l=%d *****", queue.front().point.x, queue.front().point.y, queue.front().point.z, queue.front().label);

		std::string targetFrame = FRAME_BASE;
		// effector->setEndEffector("right_eef");
		effector->setPoseReferenceFrame(targetFrame);
		ROS_INFO("...plan frame: %s - pose frame: %s", effector->getPlanningFrame().c_str(), effector->getPoseReferenceFrame().c_str());

		// Generate the target pose
		geometry_msgs::Pose target = GraspingUtils::genPose(0.546, 0.011, 0.77, DEG2RAD(45), 1, 1, 0);
		//		geometry_msgs::Pose armPose = GraspingUtils::genPose(queue.front().x, queue.front().y, queue.front().z, DEG2RAD(225), 0, 1, 1);
		//		armPose.position.x = queue.front().x;
		//		armPose.position.y = queue.front().y;
		//		armPose.position.z = queue.front().z;
		//		armPose.orientation.w = 1.0;

		// Print positions
		geometry_msgs::PoseStamped current = effector->getCurrentPose();
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

		effector->setPoseTarget(msg);

		// Plan the trajectory
		moveit::planning_interface::MoveGroup::Plan armPlan;
		ROS_INFO("...planing effector trajectory");
		bool planningOk = effector->plan(armPlan);
		ROS_INFO("...trajectory plan %s", planningOk ? "SUCCESSFUL" : "FAILED");

		// Move the effector
		if (planningOk)
		{
			ROS_INFO("...moving right effector");
			effector->move();
			ROS_INFO("...effector movement completed");

			current = effector->getCurrentPose();
			PRINT_POSE("curr", current.pose.position, current.header.frame_id);
			geometry_msgs::Point transformed = GraspingUtils::transformPose(tfListener, targetFrame, current.header.frame_id, current.pose.position);
			PRINT_POSE("curr", transformed, targetFrame);
		}
		else
			ROS_INFO("...movement aborted");**/
	}
}



void test2(const pr2_grasping::GraspingPointArrayConstPtr &msg_, const std::string xx)
{
	ROS_INFO("test callback %s", xx.c_str());

	// ROS_INFO("Received array size %zu", msg_->data.size());
	// for (size_t i = 0; i < msg_->data.size(); i++)
	// {
	// 	ROS_INFO("id=%s -- label:%d -- p=(%f, %f, %f) -- n=(%f, %f, %f)",
	// 			 msg_->data[i].header.frame_id.c_str(),
	// 			 msg_->data[i].label,
	// 			 msg_->data[i].position.x,
	// 			 msg_->data[i].position.y,
	// 			 msg_->data[i].position.z,
	// 			 msg_->data[i].normal.x,
	// 			 msg_->data[i].normal.y,
	// 			 msg_->data[i].normal.z);
	// }
}

void timerCallback(const ros::TimerEvent &event_,
				   const MoveGroupPtr &effector_,
				   const tf::TransformListener *tfListener_,
				   const ros::Publisher &posePubslisher_)
{}

int main(int _argn, char **_argv)
{
	ros::init(_argn, _argv, "pr2_grasper");
	ros::NodeHandle handler;
	tf::TransformListener *tfListener = new tf::TransformListener(ros::Duration(10.0));

	// Load the node's configuration
	ROS_INFO("Loading %s config", ros::this_node::getName().c_str());
	if (!Config::load(GraspingUtils::getConfigPath()))
		throw std::runtime_error((std::string) "Error reading config at " + GraspingUtils::getConfigPath());

	// Get the max allowed size for the message queue
	queueMaxsize = Config::get()["grasper"]["queueMaxsize"].as<int>();

	// Group to plane movement for both arms
	ROS_INFO("Setting effector control");
	std::pair<std::string, std::string> effectorNames = RobotUtils::getEffectorNames(Config::get()["grasper"]["arm"].as<std::string>());
	MoveGroupPtr effector = MoveGroupPtr(new moveit::planning_interface::MoveGroup(effectorNames.first));
	effector->setEndEffector(effectorNames.second);

	// Set subscriptions
	ros::Publisher posePublisher = handler.advertise<geometry_msgs::PoseStamped>("/pr2_grasping/grasping_pose", 1);
	ros::Subscriber sub = handler.subscribe("/pr2_grasping/grasping_points", 10, graspingPointsCallback);
	ros::Timer timer = handler.createTimer(ros::Duration(1), boost::bind(timerCallback, _1, effector, tfListener, posePublisher));

	// Keep looping
	ROS_INFO("Grasper node looping");
	ros::AsyncSpinner spinner(2); // Use 2 threads
	spinner.start();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
