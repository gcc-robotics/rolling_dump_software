#include <iostream>
#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "SelfCheck.cpp"
using namespace std;

int main(int argc, char **argv)
{
	// Start Node
	ROS_INFO("Starting self_check_node");

	ros::init(argc, argv, "self_check");
	ros::NodeHandle node;
	SelfCheck check = SelfCheck(node);
	int count = 0;
	std_msgs::Bool healthMsg;


	// Start dump_pathplanner
	//Pathplanner path = Pathplanner(node);
	ROS_INFO("Press Ctrl-C to kill node.");

	// Spin
	ros::Rate loopRate(10); // 10 hz

	while(ros::ok())
	{
		//geometry_msgs::PointStamped bob = path.getTargetPoint(path.currentPose.pose.position.x, path.currentPose.pose.position.y, true);

		//ROS_INFO("current location: %f %f", path.currentPose.pose.position.x, path.currentPose.pose.position.y);
		//ROS_INFO("Target point: %f %f", bob.point.x, bob.point.y);
		//path.targetPointPublisher.publish(bob);
		// ROS Spin & Sleep

		healthMsg.data = true;
		
		check.healthPublisher.publish(healthMsg);
		count++;

		ros::spinOnce();
		loopRate.sleep();
	}

	ros::shutdown();

	return 0;
}


