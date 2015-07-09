#include <vector>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

#ifndef __SelfCheck_H_
#define __SelfCheck_H_


class SelfCheck
{
	private:
		ros::NodeHandle node;
		std::vector<ros::Subscriber> subscriberList;
	public:
		SelfCheck(ros::NodeHandle rosNode);
		ros::Publisher healthPublisher;
		void checkCallback();
		void imageCallback();
};

#endif
