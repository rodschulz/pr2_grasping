/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

void graspingCallback(const std_msgs::String::ConstPtr &_msg)
{
	ROS_INFO("I heard: [%s]", _msg->data.c_str());
}

int main(int _argn, char **_argv)
{
	// Initialize the node as "graping_node"
	ros::init(_argn, _argv, "graping_node");

	// Set the node handler
	ros::NodeHandle nodeHandler;
	// Subscribe to the topic ""
	ros::Subscriber sub = nodeHandler.subscribe("chatter", 1000, graspingCallback);

	return EXIT_SUCCESS;
}
