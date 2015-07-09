#include <iostream>
#include <stdio.h>
#include <string.h>
#include "Pathplanner.h"
#include "Pathplanner.cpp"
#include "ros/ros.h"
using namespace std;

int main(int argc, char **argv)
{
	// Start Node
	ROS_INFO("Starting pathplanner_node");

	ros::init(argc, argv, "pathplanner");
	ros::NodeHandle node;

	// Start dump_pathplanner
	Pathplanner path = Pathplanner(node);
	ROS_INFO("Press Ctrl-C to kill node.");

	// Spin
	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();

	return 0;
}



