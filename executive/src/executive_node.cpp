#include <iostream>
#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "Executive.cpp"
using namespace std;

int main(int argc, char **argv)
{
	// Start Node
	ROS_INFO("Starting executive_node");

	ros::init(argc, argv, "executive");
	ros::NodeHandle node;
	ros::ServiceClient client = node.serviceClient<navigation::navigateToPointStamped>("/navigation/navigateToPointStamped");
	ros::ServiceClient poseClient = node.serviceClient<navigation::navigateToPoseStamped>("navigation/navigateToPoseStamped");
	ros::ServiceClient targetClient = node.serviceClient<navigation::getTargetPoint>("navigation/getTargetPoint");
	Executive exec = Executive(node);
	int currentCount = 0;
	geometry_msgs::Quaternion odom_quat;

	ROS_INFO("Press Ctrl-C to kill node.");

	// Spin
	ros::Rate loopRate(10); // 10 hz

	while(ros::ok())
	{
		if(exec.healthCheckOK)
		{
			navigation::navigateToPoseStamped::Request navPose;
			navigation::navigateToPoseStamped::Response respPose;
			if(currentCount < 4)
			{
				navPose.targetPose.pose.position.x = exec.currentPose.pose.position.x;
				navPose.targetPose.pose.position.y = exec.currentPose.pose.position.y;
				float theta =  2 * acos(exec.currentPose.pose.orientation.z) + 3.14/2.0;
				odom_quat = tf::createQuaternionMsgFromYaw(theta);
				navPose.targetPose.pose.orientation = odom_quat; 
				if(poseClient.call(navPose, respPose))
					ROS_INFO("Turned 90 degrees");
				else
					ROS_INFO("Failed");
				currentCount++;
			}
		// }
		// else
		// {
		// 	ROS_ERROR("Health check failed");
		// }
		// if(exec.healthCheckOK)
		// {
			navigation::getTargetPoint::Request targetRequest;
			navigation::getTargetPoint::Response targetResponse;

			if(targetClient.call(targetRequest, targetResponse))
			{
				navigation::navigateToPointStamped::Request nav;
				navigation::navigateToPointStamped::Response resp;

				nav.targetPoint.point.x = targetResponse.targetPoint.point.x;
				nav.targetPoint.point.y = targetResponse.targetPoint.point.y;

				ROS_INFO("Current Pos (x,y) : (%f,%f)" , exec.currentPose.pose.position.x, exec.currentPose.pose.position.y);
				ROS_INFO("Target (x,y) : (%f,%f)" , nav.targetPoint.point.x, nav.targetPoint.point.y);

				if(client.call(nav, resp))
				{
					ROS_INFO("Success!!");

				}
				else
				{
					ROS_INFO("Failed");
				}
			}
			else
			{
				ROS_ERROR("TargetPoint service call failed");
			}
		}
		else
		{	
			ROS_ERROR("Health check failed!!  :(");
		}

		ros::spinOnce();
		loopRate.sleep();
	}

	ros::shutdown();

	return 0;
}


