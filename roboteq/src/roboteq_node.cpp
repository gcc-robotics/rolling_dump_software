#include <iostream>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include "roboteq/MotorCommands.h"

#include "include/RoboteqDevice.h"
#include "include/ErrorCodes.h"
#include "include/Constants.h"

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

void motorSetCallback(boost::shared_ptr<roboteq::MotorCommands> msg)
{
	ROS_INFO("Setting Motor Speeds: %i %i", msg->leftMotor, msg->rightMotor);
	leftMotorSpeed = (int)((msg->leftMotor / 100.0) * 1024);
	rightMotorSpeed = (int)((msg->rightMotor / 100.0) * 1024);
}

int main(int argc, char **argv)
{
	// Connect to the motor controller
	string mcPort = "/dev/ttyACM1";

	ROS_INFO("Connecting to Motor Controller on: %s", mcPort.c_str());

	RoboteqDevice motorController;
	int status = motorController.Connect(mcPort);

	if(status != RQ_SUCCESS)
	{
		ROS_INFO("Error connecting to Motor Controller! Error #: %i", status);
		return 1;
	}
	else
	{
		ROS_INFO("Connected to Motor Controller! %i", RQ_SUCCESS);
	}

	// Start up ROS
	ROS_INFO("Starting roboteq_node");

	ros::init(argc, argv, "roboteq_node");
	ros::NodeHandle node;

	// Subscribe to motorSpeeds topic 
	string topicName = "motorSpeeds";

	ROS_INFO("Subscribing to topic: %s", topicName.c_str());
	ros::Subscriber sub = node.subscribe(topicName, 1, motorSetCallback);

	ROS_INFO("Press Ctrl-C to kill node.");

	// Spin
	ros::Rate loopRate(10); // 10 hz

	while(ros::ok())
	{
		// Set motor speeds
		motorController.SetCommand(_GO, 1, leftMotorSpeed);
		motorController.SetCommand(_GO, 2, rightMotorSpeed);

		// ROS Spin & Sleep
		ros::spinOnce();
		loopRate.sleep();
	}

	motorController.Disconnect();
	ros::shutdown();

	return 0;
}
