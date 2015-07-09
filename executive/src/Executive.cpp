#include "Executive.h"

/**
	Constructor for Executive class

	@param	rosNode	ros::NodeHandle
	@return			void
*/
Executive::Executive(ros::NodeHandle rosNode)
{
	this->node = rosNode;
	this->healthCheckOK = false;
	this->poseSubscriber = this->node.subscribe("/slam_out_pose", 1, &Executive::poseCallback, this);
	this->targetSubscriber = this->node.subscribe("/targetPoint", 1, &Executive::navigationCallback, this);
	this->healthSubscriber = this->node.subscribe("/healthCheck", 1, &Executive::healthCallback, this);
}

/**
	Callback function for processing current pose

	@param	msg	geometry_msgs::PoseStamped
	@return		void
*/
void Executive::poseCallback(geometry_msgs::PoseStamped msg)
{
	this->currentPose.pose.position.x = msg.pose.position.x;
	this->currentPose.pose.position.y = msg.pose.position.y;
	this->currentPose.pose.orientation.z = msg.pose.orientation.z;
}

/**
	Callback function for processing targetPoint for navigation
	
	@param	msg	geometry_msgs::Point
	@return		void
*/
void Executive::navigationCallback(geometry_msgs::PointStamped msg)
{
	this->targetPoint.point.x = msg.point.x;
	this->targetPoint.point.y = msg.point.y;
}

/**
	Callback function for processing health message for housekeeping

	@param	msg	std_msgs::bool
	@return		void
*/
void Executive::healthCallback(std_msgs::Bool msg)
{
	this->healthCheckOK = msg.data;
}
