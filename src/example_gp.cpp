/**
 * Author: rodrigo
 * 2016
 */
#include <ros/ros.h>

int main(int _argn, char **_argv)
{
	ros::init(_argn, _argv, "pr2_grasper");
	ros::NodeHandle nodeHandler;

	// Keep looping
	ROS_INFO("Grasper node looping");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::waitForShutdown();

	return EXIT_SUCCESS;
}
