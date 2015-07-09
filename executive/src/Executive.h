#include "ros/ros.h"
#include <cmath>
#include "navigation/navigateToPointStamped.h"
#include "navigation/navigateToPoseStamped.h"
#include "navigation/getTargetPoint.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_datatypes.h>

#ifndef __Executive_H_
#define __Executive_H_


class Executive
{
	private:
		ros::NodeHandle node;
		ros::Subscriber targetSubscriber;
		ros::Subscriber poseSubscriber;
		ros::Subscriber healthSubscriber;
		ros::ServiceClient client;
		ros::ServiceClient poseClient;
		ros::ServiceClient targetClient;

	public:
		Executive(ros::NodeHandle rosNode);
		void poseCallback(geometry_msgs::PoseStamped msg);
		void navigationCallback(geometry_msgs::PointStamped msg);
		void healthCallback(std_msgs::Bool msg);
		geometry_msgs::PoseStamped currentPose;
		geometry_msgs::PointStamped targetPoint;		
		bool healthCheckOK;
};

#endif
