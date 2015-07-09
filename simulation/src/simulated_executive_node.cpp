#include <iostream>
#include <stdio.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "navigation/getTargetPoint.h"
#include <tf/transform_datatypes.h>

#include <cmath>

#define PI (3.141592653589793238)

ros::Publisher simulatedPosePublisher;
ros::Publisher simulatedTargetPointPubliser;
geometry_msgs::PointStamped targetPoint;

geometry_msgs::PoseStamped simulatedPoseMessage;

geometry_msgs::Quaternion odom_quat;

int seq = 0;
int counter = 1;
double orientationZ = 0.0;

//bool targetPointUpdated = false;

/*void processTargetPoint(const geometry_msgs::PointStamped msg)
{
	//ROS_INFO("TargetPoint got updated");
	targetPoint = msg;
	targetPointUpdated = true;
}*/

int main(int argc, char **argv)
{
	ROS_INFO("Starting simulated_executive_node");

	ros::init(argc, argv, "simulated_executive_node");
	ros::NodeHandle node;

	ros::Publisher simulatedPosePubliser = node.advertise<geometry_msgs::PoseStamped>("/simulatedPose", 0);
	ros::Publisher simulatedTargetPointPubliser = node.advertise<geometry_msgs::PointStamped>("/simulatedTargetPoint", 0);

	//ROS_INFO("Subscribing to topic: %s", "/targetPoint");
	//ros::Subscriber subScan = node.subscribe("/targetPoint", 1, processTargetPoint);

	ros::ServiceClient targetPointRequest = node.serviceClient<navigation::getTargetPoint>("/navigation/getTargetPoint");

	simulatedPoseMessage.pose.position.x = 0.0;
	simulatedPoseMessage.pose.position.y = 0.0;
	simulatedPoseMessage.pose.position.z = 0.0;

	//1Hz
	ros::Rate loopRate(5);

	while(ros::ok())
	{
		geometry_msgs::PointStamped simulatedPointMessage;

		navigation::getTargetPoint::Request reqTargetPoint;
		navigation::getTargetPoint::Response respTargetPoint;

		//targetPointRequest.call(reqTargetPoint, respTargetPoint);

		simulatedPointMessage.header.stamp = ros::Time::now();
		simulatedPointMessage.header.seq;

		simulatedPoseMessage.header.stamp = ros::Time::now();
		simulatedPoseMessage.header.seq++;
		simulatedPoseMessage.header.frame_id = "map";

		if(targetPointRequest.call(reqTargetPoint, respTargetPoint))
		{
			simulatedPointMessage = respTargetPoint.targetPoint;

			double targetX = respTargetPoint.targetPoint.point.x;
			double targetY = respTargetPoint.targetPoint.point.y;

			double deltaX = targetX - simulatedPoseMessage.pose.position.x;
			double deltaY = targetY - simulatedPoseMessage.pose.position.y;

			double hypotenuse = sqrt(pow(deltaY, 2) + pow(deltaX, 2));

			if(hypotenuse > 0.05)
			{

				double theta = acos(deltaX / hypotenuse);

				if(deltaY < 0)
				{
					theta *= -1;
				}

				odom_quat = tf::createQuaternionMsgFromYaw(theta);

				//ROS_INFO("Pose x %f", simulatedPoseMessage.pose.position.x);
				//ROS_INFO("Pose y %f", simulatedPoseMessage.pose.position.y);

				//ROS_INFO("Point x %f", simulatedPointMessage.point.x);
				//ROS_INFO("Point y %f", simulatedPointMessage.point.y);

				ROS_INFO("deltaX %f", deltaX);
				ROS_INFO("deltaY %f", deltaY);
				ROS_INFO("theta %f", theta);

				//ROS_INFO("tangent of y/x %f", tan((deltaY) / (deltaX)));
				//ROS_INFO("cosine of whole %f", cos(tan((deltaY) / (deltaX)) / 2));

				// if(orientationZ == std::numeric_limits<float>::quiet_NaN())
				// {
				// 	ROS_INFO("I am inside the NaN if");
				// 	orientationZ = 0.999;
				// }

				if(counter > 1)
				{
					//ROS_INFO("Counter %i", counter);

					simulatedPoseMessage.pose.position.x = targetX;
					simulatedPoseMessage.pose.position.y = targetY;
					simulatedPoseMessage.pose.position.z = 0.0;

					//simulatedPoseMessage.pose.orientation.x = 0.0;
					//simulatedPoseMessage.pose.orientation.y = 0.0;
					simulatedPoseMessage.pose.orientation = odom_quat;
					//simulatedPoseMessage.pose.orientation.w = 1.0;

					simulatedTargetPointPubliser.publish(simulatedPointMessage);
					simulatedPosePubliser.publish(simulatedPoseMessage);

					//targetPointUpdated = false;
				}
				else
				{
					//ROS_INFO("First time in the loop");
					counter++;

					simulatedPoseMessage.pose.position.x = -5.0;
					simulatedPoseMessage.pose.position.y = 5.0;
					simulatedPoseMessage.pose.position.z = 0.0;

					//simulatedPoseMessage.pose.orientation.x = 0.0;
					//simulatedPoseMessage.pose.orientation.y = 0.0;
					simulatedPoseMessage.pose.orientation = odom_quat;
					//simulatedPoseMessage.pose.orientation.w = 1.0;

					simulatedTargetPointPubliser.publish(simulatedPointMessage);
					simulatedPosePubliser.publish(simulatedPoseMessage);

					//targetPointUpdated = false;
				}
			}
		}
		else
		{
			ROS_WARN("Cannot connect pathplanner");
		}

		ros::spinOnce();
		loopRate.sleep();
	}

	ros::shutdown();

	return 0;
}