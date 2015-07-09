#include <iostream>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"


#define START (180)					//This constant indicates the starting index of the filtered LRF from raw LRF data
#define END (900)					//This constant indicates the ending index of the filtered LRF from raw LRF data 
#define MID_POINT (540)				//This constant indicates the middle index from raw LRF data
#define ARRAY_SIZE (1080)			//This constant indicates the array size of the raw data
#define FILTERED_ARRAY_SIZE (720)	//This constant indicates the array size of the filtered data

#define myAngleMin (-1.57079633)	//This constant indicates the minimum angle for filtered scan
#define myAngleMax (1.57079633)		//This constant indicates the maximum angle for filtered scan

sensor_msgs::LaserScanPtr filterMsg(new sensor_msgs::LaserScan());

bool buffering = false;

double angleMin;
double angleMax;
double angleInc;
double timeInc;
double scanTime;
double rangeMin;
double rangeMax;

ros::Publisher lrfFilterDataPublisher;

void processRangefinderData(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//static bool run_once = false;

	filterMsg->header = msg->header;

	filterMsg->angle_min = myAngleMin;
	filterMsg->angle_max = myAngleMax;
	filterMsg->angle_increment = (double)(msg->angle_increment);
	filterMsg->time_increment = (double)(msg->time_increment);
	filterMsg->scan_time = (double)(msg->scan_time);
	filterMsg->range_min = (double)(msg->range_min);
	filterMsg->range_max = (double)(msg->range_max);

	if(!buffering)
	{
		filterMsg->ranges.resize(FILTERED_ARRAY_SIZE);
		filterMsg->intensities.resize(FILTERED_ARRAY_SIZE);

		for(int i = START; i < END; i++)
		{
			filterMsg->ranges[i - START] = (double)(msg->ranges[i]);
			filterMsg->intensities[i - START] = (double)(msg->intensities[i]);
		}
	}


	// Debugging tool
	// if(!run_once) 
	// {
	// 	run_once = true;
	// 	// PRINT STUFF
	// 	for(int i = 0; i < ARRAY_SIZE; i++)
	// 	{
	// 		ROS_INFO("Raw Data: [%d] %f", i, msg->ranges[i]);
	// 	}
	// 	for(int i = 0; i < FILTERED_ARRAY_SIZE; i++)
	// 	{
	// 		ROS_INFO("Filted Data: [%i] %f", i, filterMsg->ranges[i]);
	// 	}
	// }	

}

void processBufferRequest(const std_msgs::Bool msg)
{
	buffering =  msg.data;
}

int main(int argc, char **argv)
{
	ROS_INFO("Starting LRF_filter node");

	ros::init(argc, argv, "LRF_filter");
	ros::NodeHandle node;

	ros::Publisher lrfFilterDataPublisher = node.advertise<sensor_msgs::LaserScan>("/filteredScan", 1);

	ros::Rate loopRate(40);

	ROS_INFO("Subscribing to topic: %s", "/rawScan");
	ros::Subscriber subScan = node.subscribe("/rawScan", 1, processRangefinderData);

	ROS_INFO("Subscribing to topic: %s", "/enableLRBuffer");
	ros::Subscriber subBuffer = node.subscribe("/enableLRBuffer", 1, processBufferRequest);

	ROS_INFO("Press Ctrl-C to kill node.");

	while(ros::ok())
	{
		lrfFilterDataPublisher.publish(filterMsg);

		ROS_INFO("ROS is alive");

		ROS_INFO("seq [%i]", filterMsg->header.seq);

		ros::spinOnce();

		loopRate.sleep();
	}

	ROS_INFO("ROS is not okay");

	ros::shutdown();

	return 0;
}