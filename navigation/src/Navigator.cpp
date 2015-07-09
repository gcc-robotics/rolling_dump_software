#include "Navigator.h"

Navigator::Navigator(ros::NodeHandle rosNode)
{
	this->node = rosNode;
	
	// Init current pose data
	this->currentPose = geometry_msgs::PoseStamped();
	this->curentPoseUpdated = false;
	
	// Subscribe to pose data
	this->poseSubscriber = this->node.subscribe("/slam_out_pose", 1, &Navigator::processPose, this);
	
	// Register the navigation services
	this->navigateToPointStampedService = this->node.advertiseService("/navigation/navigateToPointStamped", &Navigator::navigateToPointStamped, this);
	this->navigateToPoseStampedService = this->node.advertiseService("/navigation/navigateToPoseStamped", &Navigator::navigateToPoseStamped, this);

	// Init Motor Command Publisher
	this->motorCommandPublisher = this->node.advertise<roboteq::MotorCommands>("/motorSpeeds", 1);
}

/**
	Callback for the current pose provided by SLAM.
	@param		msg		geometry_msgs::PoseStamped
	@return		void
*/
void Navigator::processPose(geometry_msgs::PoseStamped msg)
{
	// Copy position
	this->currentPose.pose.position.x = msg.pose.position.x;
	this->currentPose.pose.position.y = msg.pose.position.y;
	this->currentPose.pose.position.z = msg.pose.position.z;
	
	// Copy orientation
	this->currentPose.pose.orientation.x = msg.pose.orientation.x;
	this->currentPose.pose.orientation.y = msg.pose.orientation.y;
	this->currentPose.pose.orientation.z = msg.pose.orientation.z;
	this->currentPose.pose.orientation.w = msg.pose.orientation.w;
	
	this->curentPoseUpdated = true;
}

/**
	Navigate to a provided point

	Loop at 10 hz until we get to the point and call ros::spinOnce() every 
	iteration to make sure we continue getting current robot pose updates.

	TODO:
	Should probably add a timeout or something so we don't get stuck in an 
	infinite loop. If it trips we can send back false in the response.
	
	@param		&req	navigation::navigateToPointStamped::Request
	@param		&res	navigation::navigateToPointStamped::Response
	@return		success	bool
*/	
bool Navigator::navigateToPointStamped(navigation::navigateToPointStamped::Request  &req, 
		                               navigation::navigateToPointStamped::Response &res)
{
	ROS_INFO("Navigating to Point: X: %f, Y: %f", req.targetPoint.point.x, req.targetPoint.point.y);
	
	bool targetPointReached = false;
	
	// Set the loop rate to 10 hz
	ros::Rate navRate(10);

	while(!targetPointReached && ros::ok())
	{
		// Init motorCommands message
		roboteq::MotorCommands motorMsg;

		// Only move if the pose has been updated
		if(this->curentPoseUpdated)
		{
			// Get the angle and distance to the target point
			double distanceToTarget = this->calculateDistanceToTargetPoint(req.targetPoint.point);
			double angleToTarget = this->calculateAngleToTargetPoint(req.targetPoint.point);
			
			if(distanceToTarget < this->distanceToTargetThreshold)
			{
				// We have reached the target
				targetPointReached = true;

				ROS_INFO("Target point reached, Stopping.");

				// Stop
				motorMsg.leftMotor = 0;
				motorMsg.rightMotor = 0;
			}
			else
			{
				// Check if we need to turn
				if(abs(angleToTarget) < this->angleToTargetThreshold)
				{
					// The target angle is close enough
					ROS_INFO("Moving forward");
					
					// Calculate the motor speed
					int motorSpeed = (distanceToTarget > 1.0) ? this->maxMotorSpeed : distanceToTarget * this->maxMotorSpeed;
					motorSpeed = (motorSpeed < this->minMotorSpeed) ? this->minMotorSpeed : motorSpeed;
					motorSpeed = (motorSpeed > this->maxMotorSpeed) ? this->maxMotorSpeed : motorSpeed;

					// Move forward
					motorMsg.leftMotor = motorSpeed;
					motorMsg.rightMotor = motorSpeed;
				}
				else
				{
					// We need to turn
					
					// Calculate the motor speed
					int motorSpeed = (abs(angleToTarget) > this->angleToTargetThreshold * 2) 
						? this->maxMotorSpeed 
						: ((angleToTarget / (360 - this->angleToTargetThreshold)) * this->maxMotorSpeed) + minMotorSpeed;
					motorSpeed = (motorSpeed < this->minMotorSpeed) ? this->minMotorSpeed : motorSpeed;
					motorSpeed = (motorSpeed > this->maxMotorSpeed) ? this->maxMotorSpeed : motorSpeed;

					if(angleToTarget > 0)
					{
						// Turn right (Clockwise)
						ROS_INFO("Turning right");

						// Turn Right
						motorMsg.leftMotor = motorSpeed;
						motorMsg.rightMotor = -motorSpeed;
					}
					else
					{
						// Turn left (Counter-Clockwise)
						ROS_INFO("Turning left");

						// Turn Left
						motorMsg.leftMotor = -motorSpeed;
						motorMsg.rightMotor = motorSpeed;
					}
				}
			}
			
			// Mark the current pose as old
			this->curentPoseUpdated = false;
		}
		else
		{
			ROS_INFO("Waiting for the robot pose to be updated.");

			// Stop
			motorMsg.leftMotor = 0;
			motorMsg.rightMotor = 0;
		}
		
		// Publish the motor commands
		this->motorCommandPublisher.publish(motorMsg);

		// Sleep
		navRate.sleep();
	}
	
	// Set the response
	res.pointReached = targetPointReached;
	return res.pointReached;
}

/**
	Navigate to a provided pose

	First call the navigateToPoint with the position in the targetPose to 
	take care of moving to the the correct positions.

	Next, loop at 10 hz until we rotate to the orientation specified in 
	targetPose and call ros::spinOnce() every iteration to make sure we 
	continue getting current robot pose updates.

	TODO:
	Should probably add a timeout or something so we don't get stuck in an 
	infinite loop. If it trips we can send back false in the response.

    @param		&req	navigation::navigateToPoseStamped::Request
    @param		&res	navigation::navigateToPoseStamped::Response
    @return		success	bool
*/
bool Navigator::navigateToPoseStamped(navigation::navigateToPoseStamped::Request  &req, 
		                       navigation::navigateToPoseStamped::Response &res)
{
	ROS_INFO("Navigating to Pose: Position X: %f, Y: %f, Orientation: Z: %f", 
	         req.targetPose.pose.position.x, 
	         req.targetPose.pose.position.y, 
	         req.targetPose.pose.orientation.z);
	
	navigation::navigateToPointStamped moveToPoint;
	moveToPoint.request.targetPoint.point.x = req.targetPose.pose.position.x;
	moveToPoint.request.targetPoint.point.y = req.targetPose.pose.position.y;
	moveToPoint.request.targetPoint.point.z = req.targetPose.pose.position.z;
	
	this->navigateToPointStamped(moveToPoint.request, moveToPoint.response);
	
	if(moveToPoint.response.pointReached)
	{
		// We are at the correct position, now we need to rotate to the
		// correct orientation
		bool targetAngleReached = false;
		
		// Set the loop rate to 10 hz
		ros::Rate navRate(10);

		while(!targetAngleReached && ros::ok())
		{
			// Only move if the pose has been updated
			if(this->curentPoseUpdated)
			{
				// Get the angle  to the target pose
				double angleToTarget = this->calculateAngleToTargetPoint(moveToPoint.request.targetPoint.point);
				
				// Init motorCommands message
				roboteq::MotorCommands motorMsg;
				
				// Check if we need to turn
				if(abs(angleToTarget) < this->angleToTargetThreshold)
				{
					// The target angle is close enough
					ROS_INFO("Target pose reached!");
					targetAngleReached = true;

					// Stop
					motorMsg.leftMotor = 0;
					motorMsg.rightMotor = 0;
				}
				else
				{
					// We need to turn
					
					// Calculate the motor speed
					int motorSpeed = (abs(angleToTarget) > this->angleToTargetThreshold * 2) 
						? this->maxMotorSpeed 
						: ((angleToTarget / (360 - this->angleToTargetThreshold)) * this->maxMotorSpeed) + minMotorSpeed;
					motorSpeed = (motorSpeed < this->minMotorSpeed) ? this->minMotorSpeed : motorSpeed;
					motorSpeed = (motorSpeed > this->maxMotorSpeed) ? this->maxMotorSpeed : motorSpeed;

					if(angleToTarget > 0)
					{
						// Turn right (Clockwise)
						ROS_INFO("Turning right");

						// Turn Right
						motorMsg.leftMotor = motorSpeed;
						motorMsg.rightMotor = -motorSpeed;
					}
					else
					{
						// Turn left (Counter-Clockwise)
						ROS_INFO("Turning left");

						// Turn Left
						motorMsg.leftMotor = -motorSpeed;
						motorMsg.rightMotor = motorSpeed;
					}
				}
				
				// Mark the current pose as old
				this->curentPoseUpdated = false;
			}
			else
			{
				ROS_INFO("Waiting for the robot pose to be updated.");
			}
			
			// Sleep
			navRate.sleep();
		}
		
		res.poseReached = targetAngleReached;
	}
	else
	{
		ROS_INFO("Failed to navigate to the position of the target pose!");
		res.poseReached = false;
	}
	
	return res.poseReached;
}

/**
	Calculate the number of degrees we need to turn in order to face towards the target point
	@param	target	geometry_msgs::Point
	@return	angle	double
*/
double Navigator::calculateAngleToTargetPoint(geometry_msgs::Point target)
{
	double currentAngle = 2 * acos(this->currentPose.pose.orientation.z) * (180 / 3.14);
	double deltaX = target.x - this->currentPose.pose.position.x;
	double deltaY = target.y - this->currentPose.pose.position.y;
	double triangleAngle = atan(deltaY / deltaX) * (180 / 3.14);
	
	if(deltaX > 0 && deltaY > 0)
	{
		triangleAngle = 180 - triangleAngle;
	}
	else if(deltaX < 0 && deltaY > 0)
	{
		triangleAngle = 0 - triangleAngle;
	}
	else if(deltaX > 0 && deltaY < 0)
	{
		triangleAngle = 180 - triangleAngle ;
	}
	else
	{
		triangleAngle = (360 - triangleAngle);
	}
	
	return (-1.0)*(currentAngle - triangleAngle);
}

/**
	Calculate how far we need to move in meters to reach the target point
	@param	target		geometry_msgs::Point
	@return	distance	double
*/
double Navigator::calculateDistanceToTargetPoint(geometry_msgs::Point target)
{
	double currentZQuart = this->currentPose.pose.orientation.z;
	double deltaX = target.x - this->currentPose.pose.position.x;
	double deltaY = target.y - this->currentPose.pose.position.y;
	
	return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}
