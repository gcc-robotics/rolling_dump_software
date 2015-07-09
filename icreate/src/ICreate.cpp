#include "ICreate.h"

/**
	Constructor for ICreate class

	@param	rosNode	ros::NodeHandle
	@return			void
*/
ICreate::ICreate(ros::NodeHandle rosNode)
{
	this->node = rosNode;
	this->speedSubscriber = this->node.subscribe("/motorSpeeds", 1, &ICreate::speedCallback, this);
	this->velocityPublisher = this->node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

}

/**
	Callback function for processing motor speeds

	@param	msg	boost::shared_ptr<roboteq::MotorCommands>
	@return		void
*/
void ICreate::speedCallback(boost::shared_ptr<roboteq::MotorCommands> msg)
{
	ROS_INFO("Motor Speeds; Left: %i, Right: %i", msg->leftMotor, msg->rightMotor);
	
	velocityCommand.linear.x = (msg->rightMotor + msg->leftMotor) / 2;
	velocityCommand.angular.z = (msg->rightMotor - msg->leftMotor) / 100;
	
	ROS_INFO("Twist Message; linear.x: %f, angular.z: %f", velocityCommand.linear.x, velocityCommand.angular.z);
	
	velocityPublisher.publish(velocityCommand);

}

