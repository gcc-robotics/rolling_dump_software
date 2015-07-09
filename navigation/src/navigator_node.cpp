#include <iostream>
#include <stdio.h>
#include <string.h>
#include "Navigator.cpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
	// Start Node
	ROS_INFO("Starting navigator_node");

	ros::init(argc, argv, "navigator_node");
	ros::NodeHandle node;

	// Start Navigator
	Navigator nav = Navigator(node);
	ROS_INFO("Press Ctrl-C to kill node.");

	// Spin
	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();

	return 0;
}



