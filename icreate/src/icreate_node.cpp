#include <iostream>
#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ICreate.cpp"
using namespace std;

int main(int argc, char **argv)
{
	// Start Node
	ROS_INFO("Starting icreate_node");

	ros::init(argc, argv, "icreate");
	ros::NodeHandle node;
	geometry_msgs::Twist cmd_vel;

	ICreate create = ICreate(node);

	ROS_INFO("Press Ctrl-C to kill node.");

	// Spin
	ros::Rate loopRate(10); // 10 hz

	while(ros::ok())
	{


		ros::spinOnce();
		loopRate.sleep();
	}

	ros::shutdown();

	return 0;
}


/*
def mover():
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('robot_mover')

    twist = Twist()
    twist.linear.x = 0.1; # move forward at 0.1 m/s   

    rospy.loginfo("Moving the robot forward.")
    pub.publish(twist)
    rospy.sleep(1)

    rospy.loginfo("Moving the robot backward.")
    twist.linear.x = -0.1; # move backward at 0.1 m/s
    pub.publish(twist)
    rospy.sleep(1);

    rospy.loginfo("Turning the robot left.");
    twist = Twist();
    twist.angular.z = 0.5
    pub.publish(twist)
    rospy.sleep(1);
    
    rospy.loginfo("Turning the robot right.");
    twist.angular.z = -0.5
    pub.publish(twist)
    rospy.sleep(1);

    rospy.loginfo("Stopping!")
    twist = Twist()
    pub.publish(twist)

    rospy.loginfo("Node exiting.");
    */