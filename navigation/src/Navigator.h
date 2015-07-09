#include "ros/ros.h"
#include "roboteq/MotorCommands.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "navigation/navigateToPointStamped.h"
#include "navigation/navigateToPoseStamped.h"
#include <cmath>

#ifndef __Navigator_H_
#define __Navigator_H_


class Navigator
{
	private:
		// Constant goToTarget data
		
		// Once the robot is closer than this threshold value (Meters) it will
		// consider the target point reached
		static const double distanceToTargetThreshold = 0.25;
		
		// Once the difference between our current heading and the target 
		// heading is less than this threshold we assume we are facing the 
		// correct direction
		static const double angleToTargetThreshold = 5.0;
		
		// The maximum motor speed that navigator will pass to the roboteq node
		static const int maxMotorSpeed = 100;
		
		// The minimum non-zero speed the navigator will pass to the roboteq
		// node
		static const int minMotorSpeed = 10;
		
		// ROS stuff
		ros::NodeHandle node;
		ros::Subscriber poseSubscriber;
		ros::ServiceServer navigateToPointStampedService;
		ros::ServiceServer navigateToPoseStampedService;
		ros::Publisher motorCommandPublisher;
		
		// Current robot position
		geometry_msgs::PoseStamped currentPose;
		bool curentPoseUpdated;

	public:
		Navigator(ros::NodeHandle rosNode);
		
		// Message Callbacks
		void processPose(geometry_msgs::PoseStamped msg);
		
		// Service Callbacks
		bool navigateToPointStamped(navigation::navigateToPointStamped::Request  &req, 
		                            navigation::navigateToPointStamped::Response &res);
		bool navigateToPoseStamped(navigation::navigateToPoseStamped::Request  &req, 
		                           navigation::navigateToPoseStamped::Response &res);
		
		// Helper Functions
		double calculateAngleToTargetPoint(geometry_msgs::Point target);
		double calculateDistanceToTargetPoint(geometry_msgs::Point target);
};

#endif
