#include <termios.h>

#include "ros/ros.h"
#include "roboteq/MotorCommands.h"

int getKey()
{
	static struct termios oldt, newt;
	tcgetattr(STDIN_FILENO, &oldt); // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);  // disable buffering & echo  
	newt.c_cc[VMIN] = 0;
	newt.c_cc[VTIME] = 0;    
	tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

	int c = getchar(); // read character (non-blocking)

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
	return c;
}

int main(int argc, char **argv)
{
	ROS_INFO("Starting keyboard_node");
	ROS_INFO("Press Ctrl-C to kill node.");

	// Start up ROS
	ros::init(argc, argv, "keyboard_node");
	ros::NodeHandle node;
	ros::Publisher publisher = node.advertise<roboteq::MotorCommands>("motorSpeeds", 1);

	ros::Rate loopRate(100);

	int leftMotor = 0;
	int rightMotor = 0;
	int loopsSinceInput = 0;

	while(ros::ok())
	{
		int key = getKey();
		roboteq::MotorCommands msg;

		/*
			Up > 65
			Down > 66
			Right > 67
			Left > 68
		*/

		if(key == 65 || key == 66 || key == 67 || key == 68)
		{
			if(key == 65)
			{
				msg.leftMotor  += 100;
				msg.rightMotor += 100;
			}

			if(key == 66)
			{
				msg.leftMotor  -= 100;
				msg.rightMotor -= 100;
			}

			if(key == 67)
			{
				msg.leftMotor  += 100;
				msg.rightMotor -= 100;
			}

			if(key == 68)
			{
				msg.leftMotor  -= 100;
				msg.rightMotor += 100;
			}

			// Clamp to -100 & 100
			if(msg.leftMotor > 100)
			{
				msg.leftMotor = 100;
			}
			else if(msg.leftMotor < -100)
			{
				msg.leftMotor = -100;
			}

			if(msg.rightMotor > 100)
			{
				msg.rightMotor = 100;
			}
			else if(msg.rightMotor < -100)
			{
				msg.rightMotor = -100;
			}

			ROS_INFO("Sending MotorCommands: %i %i", msg.leftMotor, msg.rightMotor);
			publisher.publish(msg);
			loopsSinceInput = 0;
		}
		else if(key == -1)
		{
			// Nothing was pressed

			if(loopsSinceInput > 25)
			{
				msg.leftMotor  = 0;
				msg.rightMotor = 0;

				ROS_INFO("No user input. Stopping motors.");
				publisher.publish(msg);
				loopsSinceInput = 0;
			}

			loopsSinceInput++;
		}

		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}