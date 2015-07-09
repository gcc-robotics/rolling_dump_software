#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "roboteq/MotorCommands.h"

#ifndef __ICreate_H_
#define __ICreate_H_


class ICreate
{
	private:
		ros::NodeHandle node;
		geometry_msgs::Twist velocityCommand;
		ros::Subscriber speedSubscriber;
		ros::Publisher velocityPublisher;
	public:
		ICreate(ros::NodeHandle rosNode);
		void speedCallback(boost::shared_ptr<roboteq::MotorCommands> msg);	
};

#endif
