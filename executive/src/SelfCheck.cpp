#include "SelfCheck.h"

SelfCheck::SelfCheck(ros::NodeHandle rosNode)
{
	this->node = rosNode;
	this->healthPublisher = node.advertise<std_msgs::Bool>("/healthCheck", 1);
}

void SelfCheck::checkCallback()
{

	//ros::Subscriber subscriber;
	//this->subscriberList.push_back(subscriber = this->node.subscribe("/image_raw",1,&SelfCheck::imageCallback));
}
void SelfCheck::imageCallback()
{

}