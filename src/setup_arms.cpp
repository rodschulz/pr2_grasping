/**
 * Author: rodrigo
 * 2016
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction > TrajectoryClient;

class ArmsSetup
{
private:
	TorsoClient *torsoClient;
	TrajectoryClient *rightArmClient;
	TrajectoryClient *leftArmClient;

public:
	ArmsSetup()
	{
		// define action clients
		torsoClient = new TorsoClient("torso_controller/position_joint_action", true);
		rightArmClient = new TrajectoryClient("r_arm_controller/joint_trajectory_action", true);
		leftArmClient = new TrajectoryClient("l_arm_controller/joint_trajectory_action", true);

		// wait for the action server to come up
		while (!torsoClient->waitForServer(ros::Duration(5.0)))
			ROS_INFO("Waiting for action server to come up");

		while (!rightArmClient->waitForServer(ros::Duration(5.0)))
			ROS_INFO("Waiting for joint trajectory action server to come up");

		while (!leftArmClient->waitForServer(ros::Duration(5.0)))
			ROS_INFO("Waiting for joint trajectory action server to come up");
	}

	~ArmsSetup()
	{
		delete torsoClient;
	}

	void liftUpTorso()
	{
		ROS_INFO("Lifting up torso");

		pr2_controllers_msgs::SingleJointPositionGoal up;
		up.position = 0.195;
		up.min_duration = ros::Duration(2.0);
		up.max_velocity = 1.0;

		ROS_INFO("...sending torso goal");
		torsoClient->sendGoal(up);
		torsoClient->waitForResult();

		ROS_INFO("...torso lifted up");
	}

	void moveArms()
	{
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arms_setup");

	ROS_INFO("Beginning setup routine");

	ArmsSetup setup;
	setup.liftUpTorso();
	setup.moveArms();

	return EXIT_SUCCESS;
}