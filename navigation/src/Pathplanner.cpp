#include "Pathplanner.h"

/**
	Constructor for Pathplanner class

	@param	rosNode	ros::NodeHandle
	@return	void
*/

Pathplanner::Pathplanner(ros::NodeHandle rosNode)
{
	this->node = rosNode;

	// Subscribe to map data
	this->map_subscriber = this->node.subscribe("/map", 1, &Pathplanner::processMap, this);

	this->poseUpdated = false;

	// Subscribe to pose data, use /initialpose for testing
	this->pose_subscriber = this->node.subscribe("/simulatedPose", 1, &Pathplanner::processPose, this);

	// Publisher for the target point
	this->targetPointPublisher = node.advertise<geometry_msgs::PointStamped>("/targetPoint", 1);
	
	// Start sweeping to the right
	this->sweepingRight = true;

	// Register the navigation services
	this->getTargetPointService = this->node.advertiseService("/navigation/getTargetPoint", &Pathplanner::getTargetPoint, this);
}

/**
	Callback function for processing current pose

	@param	msg	geometry_msgs::PoseStamped
	@return		void
*/
void Pathplanner::processPose(geometry_msgs::PoseStamped msg)
{
	this->currentPose.pose.position.x = msg.pose.position.x;
	this->currentPose.pose.position.y = msg.pose.position.y;
	this->currentPose.pose.orientation.z = msg.pose.orientation.z;
	this->poseUpdated = true;
}

/**
	Callback function for processing OccupancyGrid array and storing into 2D array

	@param	msg	boost::shared_ptr<nav_msgs::OccupancyGrid>
	@return		void
*/
void Pathplanner::processMap(boost::shared_ptr<nav_msgs::OccupancyGrid> msg)
{
	this->gridLength = msg->info.width;	// Length from mapMetaData
	this->gridResolution = msg->info.resolution;
	this->gridCenter = gridLength / 2;
	this->boundaryBoxGrid = BOUNDARY_BOX_SIZE / this->gridResolution;
	this->gridLowerBound = this->gridCenter - (this->boundaryBoxGrid / 2);
	this->gridUpperBound = this->gridCenter + (this->boundaryBoxGrid / 2);

	int index = 0;
	for (int i = 0; i < this->gridLength; i++)
	{
		for (int j = 0; j < this->gridLength; j++)
		{
			occupancyGrid[i][j] = msg->data[index];//note: change to memset
			index++;
		}
	}
}

/**
	Function that calculates the corresponding grid indices of an x,y coordinate

	@param	x			float
	@param	y			float   
	@return	gridIndices	std::vector<int>
*/
std::vector<int> Pathplanner::getGrid(float x, float y)
{
	std::vector<int> gridIndices;
	float row, column;
	column = (x / this->gridResolution) + this->gridCenter;
	row = (-y / this->gridResolution) + this->gridCenter;

	if(row == this->gridLength)
	{
		row = this->gridLength - 1;
	}

	if(column == this->gridLength)
	{
		column = this->gridLength - 1;
	}

	//ROS_INFO(" FROM GRID: %i, %i", row, column);
	gridIndices.push_back((int)column);
	gridIndices.push_back((int)row);
	//ROS_INFO("Row: %i", gridIndices[0]);
	//ROS_INFO("Column: %i", gridIndices[1]);

	return gridIndices;
}

/**
	Function that calculates the column in which an obstacle (i.e. grid indices 
	corresponding to a value greater than 50 (obstacle) or equal to -1 (unknown))

	@param	x	float
	@param	y	float   
	@return	row	int
*/
int Pathplanner::getXObstacleInPath()
{
	// ROS_INFO("%i, %i", indices[0], indices[1]);

	float currentX = this->currentPose.pose.position.x;
	float currentY = this->currentPose.pose.position.y;
	float currentZ = this->currentPose.pose.orientation.z;
	//float currentAngle = 2 * acos(currentZ) * (180/3.14);

	std::vector<int> indices = getGrid(currentX, currentY);

	ROS_INFO("X: %f, Y: %f", currentX, currentY);

	int column = indices[1];
	int row = indices[0];

	poseUpdated = false;

	if(this->sweepingRight)
	{
		for(; column < this->gridLength - 1; ++column)
		{
			int row = (indices[0] - 1 < 0) ? 0 : indices[0] - 10;
			int endI = (indices[0] + 1 >= this->gridLength) ? this->gridLength - 1 : indices[0] + 1;

			for(; row < endI; row++)
			{
				if(occupancyGrid[row][column] == 100) // || occupancyGrid[row][column] == -1)
				{
					ROS_INFO("Obst Column: %i", column);
					ROS_INFO("Current indices: %i, %i", row, column);
					return column;
				}
			}
		}
	}
	else
	{
		for(; column >= 1; --column)
		{
			int row = (indices[0] - 1 < 0) ? 0 : indices[0] - 1;
			int endI = (indices[0] + 1 >= this->gridLength) ? this->gridLength - 1 : indices[0] + 1;
			
			for(; row < endI; row++)
			{
				if(occupancyGrid[row][column] == 100) //|| occupancyGrid[row][column] == -1)
				{
					ROS_INFO("Obst Column: %i", column);
					ROS_INFO("Current indices: %i, %i", row, column);
					return column;
				}
			}
		}
	}

	//ROS_INFO("sweeping right? %i, obstacle column: %i, indices[0]: %i, indices[1]: %i", this->sweepingRight, column, indices[0], indices[1]);
}

int Pathplanner::getYObstacleInPath()
{
	float currentX = this->currentPose.pose.position.x;
	float currentY = this->currentPose.pose.position.y;
	float currentZ = this->currentPose.pose.orientation.z;
	//float currentAngle = 2 * acos(currentZ) * (180/3.14);

	std::vector<int> indices = getGrid(currentX, currentY);

	int column = indices[1];
	int row = indices[0];

	if(this->sweepingRight)
	{
		for( row; row < this->gridLength - 1; ++row)
		{
			int column = (indices[1] - 10 < 0) ? 0 : indices[1] - 10;
			int endI = (indices[1] + 10 >= this->gridLength) ? this->gridLength - 1 : indices[1] + 10;

			for(column; column < endI; column++)
			{
				if(occupancyGrid[row][column] >= 50 || occupancyGrid[row][column] == -1)
				{
					//ROS_INFO("Obst Row: %i", row);
					return row;
				}
			}
		}
	}
	else
	{
		for(row; row >= 1; --row)
		{
			int column = (indices[1] - 10 < 0) ? 0 : indices[1] - 10;
			int endI = (indices[1] + 10 >= this->gridLength) ? this->gridLength - 1 : indices[1] + 10;
			
			for(column; column < endI; column++)
			{
				if(occupancyGrid[row][column] >= 50 || occupancyGrid[row][column] == -1)
				{
					//ROS_INFO("Obst Row: %i", row);
					return row;
				}
			}
		}
	}
}

/**
	Function that calculates the corresponding x coordinate from the
	occupancy grid index

	@param	gridColumn	int
	@return	xCoordinate	float
*/
float Pathplanner::getXCoordinate(int gridColumn)
{
	float xCoordinate = (gridColumn - this->gridCenter - 1) * this->gridResolution;
	return xCoordinate;
}

/**
	Function that calculates the corresponding y coordinate from the
	occupancy grid index

	@param	gridRow		int
	@return	yCoordinate	float
*/
float Pathplanner::getYCoordinate(int gridRow)
{
	float yCoordinate = -(gridRow - this->gridCenter - 1) * this->gridResolution;
	return yCoordinate;
}

/**
	Function that calculates the target point based on current position
	and nearest obstacle in path

	@return	targetPoint	geometry_msgs::Point
*/
geometry_msgs::PointStamped Pathplanner::getTargetPoint()
{
	float currentX = this->currentPose.pose.position.x;
	float currentY = this->currentPose.pose.position.y;
	float currentZ = this->currentPose.pose.orientation.z;
	//float currentAngle = 2 * acos(currentZ) * (180 / 3.14);


	geometry_msgs::PointStamped targetPoint;

	targetPoint.header.stamp = ros::Time::now();
	targetPoint.header.frame_id = "map";

	int nearestObstacleColumn = this->getXObstacleInPath();
	int nearestObstacleRow = this->getYObstacleInPath();
	float xCoordinateOfObstacleColumn = 0;
	float yCoordinateOfObstacleRow = this->getYCoordinate(nearestObstacleRow);
	float temp = fabs(this->getXCoordinate(nearestObstacleColumn));

	if(this->sweepingRight)
	{

		//xCoordinateOfObstacleColumn = (fabs(this->getXCoordinate(nearestObstacleColumn)) > 5.0) ? 5.0 : this->getXCoordinate(nearestObstacleColumn);
		if(temp > 5.0)
		{
			xCoordinateOfObstacleColumn = 5.0;
			ROS_INFO ("Wall. Setting 5.0");
		}
		else
		{
			xCoordinateOfObstacleColumn = this->getXCoordinate(nearestObstacleColumn);
			ROS_INFO("We see obstacle");
		}
	}
	else
	{
		//xCoordinateOfObstacleColumn = (fabs(this->getXCoordinate(nearestObstacleColumn)) > 5.0) ? -5.0 : this->getXCoordinate(nearestObstacleColumn);
		if(temp > 5.0)
		{
			xCoordinateOfObstacleColumn = -5.0;
			ROS_INFO ("Wall. Setting -5.0");
		}
		else
		{
			xCoordinateOfObstacleColumn = this->getXCoordinate(nearestObstacleColumn);
			ROS_INFO("We see obstacle");
		}
	}
	ROS_INFO("xCoordinateOfObstacleColumn: %f", xCoordinateOfObstacleColumn);
	//float yCoordinateOfObstacleRow = this->getYCoordinate(nearestObstacleRow);

	double distanceToXObstacle = fabs(currentX - xCoordinateOfObstacleColumn);
	double distanceToYObstacle = fabs(currentY - yCoordinateOfObstacleRow);

	if(this->sweepingRight)
	{
		// Target Point X Coordinate
		if(distanceToXObstacle >= MAX_MOVE_DISTANCE)
		{
			targetPoint.point.x = currentX + MAX_MOVE_DISTANCE;
			targetPoint.point.y = currentY;
		}
		else
		{
			// ROS_INFO("Hit right side, obstacle column: %f, %i", distanceToXObstacle, (distanceToXObstacle >= MAX_MOVE_DISTANCE));
			targetPoint.point.x = xCoordinateOfObstacleColumn;

			if(distanceToYObstacle <= MAX_DROP_DISTANCE)
			{
				targetPoint.point.y = currentY - distanceToYObstacle;
				this->sweepingRight = !this->sweepingRight;
			}
			else
			{
				targetPoint.point.y = currentY - MAX_DROP_DISTANCE;
				this->sweepingRight = !this->sweepingRight;				
			}
		}
	}

	else //else if(!this->sweepingRight)
	{
		// Target Point X Coordinate
		if(distanceToXObstacle >= MAX_MOVE_DISTANCE)
		{
			targetPoint.point.x = currentX - MAX_MOVE_DISTANCE;
			targetPoint.point.y = currentY;
		}

		else
		{
			// ROS_INFO("Hit left side, obstacle column: %f, %i", distanceToXObstacle, (distanceToXObstacle >= MAX_MOVE_DISTANCE));
			targetPoint.point.x = xCoordinateOfObstacleColumn;

			if(distanceToYObstacle <= MAX_DROP_DISTANCE)
			{
				targetPoint.point.y = currentY - distanceToYObstacle;
				this->sweepingRight = !this->sweepingRight;
			}
			else
			{
				targetPoint.point.y = currentY - MAX_DROP_DISTANCE;
				this->sweepingRight = !this->sweepingRight;				
			}
		}
	}

	//removing for testing
	if(targetPoint.point.y < -5)
	{
		this->sweepingRight = true;
		targetPoint.point.x = -5;
		targetPoint.point.y = 5;
	}

	return targetPoint;
}
/**
	Get targetPoint service
	
	@param		&req	navigation::getTargetPoint::Request
	@param		&res	navigation::getTargetPoint::Response
	@return		success	bool
*/	
bool Pathplanner::getTargetPoint(navigation::getTargetPoint::Request  &req,
	                             navigation::getTargetPoint::Response &res)
{
	// Set the loop rate to 1 hz
	ros::Rate marekNapRate(10);
	int currentInteration = 0;
	int maxLoopCount = 10;

	while(!this->poseUpdated && currentInteration <= maxLoopCount && ros::ok())
	{
		ROS_INFO("Pose hasn't been updated! Marek says take a nap!");

		// Sleep
		marekNapRate.sleep();

		currentInteration++;
	}

	geometry_msgs::PointStamped currentTarget;
	currentTarget = getTargetPoint();

	// Set the response
	res.targetPoint = currentTarget;
	ROS_INFO("Current target: X: %f, Y: %f", res.targetPoint.point.x, res.targetPoint.point.y);
	return true;
}



