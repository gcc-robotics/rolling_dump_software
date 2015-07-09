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
	this->map_subscriber = this->node.subscribe("/map", 0, &Pathplanner::processMap, this);

	this->poseUpdated = false;

	// Subscribe to pose data, use /initialpose for testing
	this->pose_subscriber = this->node.subscribe("/simulatedPose", 0, &Pathplanner::processPose, this);

	// Publisher for the target point
	this->targetPointPublisher = node.advertise<geometry_msgs::PointStamped>("/targetPoint", 1);
	
	// Start sweeping to the right
	this->sweepingRight = true;
	
	this->dropWhenPossible = false;

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

	@param	msg	const nav_msgs::OccupancyGrid::ConstPtr&
	@return		void
*/
void Pathplanner::processMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	this->gridLength = msg->info.width;	// Length from mapMetaData
	this->gridResolution = msg->info.resolution;
	this->gridCenter = gridLength / 2;
	this->boundaryBoxGridLength = BOUNDARY_BOX_SIZE / this->gridResolution;
	this->boundaryBoxGridLowerBound = this->gridCenter - (this->boundaryBoxGridLength / 2);
	this->boundaryBoxGridUpperBound = this->gridCenter + (this->boundaryBoxGridLength / 2);
	
	// Clamp the boundary box parameters
	// Boundary Box Grid Length
	if(this->boundaryBoxGridLength < 0)
	{
		this->boundaryBoxGridLength = 0;
	}
	
	if(this->boundaryBoxGridLength > this->gridLength)
	{
		this->boundaryBoxGridLength = this->gridLength;
	}
	
	// Boundary Box Grid Lower Bound
	if(this->boundaryBoxGridLowerBound < 0)
	{
		this->boundaryBoxGridLowerBound = 0;
	}
	
	if(this->boundaryBoxGridLowerBound > this->gridCenter)
	{
		this->boundaryBoxGridLowerBound = this->gridCenter;
	}
	
	// Boundary Box Grid Upper Bound
	if(this->boundaryBoxGridUpperBound < this->gridCenter)
	{
		this->boundaryBoxGridUpperBound = this->gridCenter;
	}
	
	if(this->boundaryBoxGridUpperBound >= gridLength)
	{
		this->boundaryBoxGridUpperBound = gridLength - 1;
	}
	

	//memcpy(this->occupancyGrid, msg->data, (this->gridLength * this->gridLength) * sizeof(int));

	for(int i = 0; i < this->gridLength; i++)
	{
		for(int j = 0; j < this->gridLength; j++)
		{
			this->occupancyGrid[i][j] = msg->data[(i * 1200) + j]; //note: change to memset
		}
	}
}

/**
	Function that calculates the corresponding grid index of an coordinate

	@param	coord	float
	@return	grid	int
*/
int Pathplanner::getGrid(float coord)
{
	int grid = (coord / this->gridResolution) + this->gridCenter;
	
	if(grid < 0)
	{
		grid = 0;
	}

	if(grid >= this->gridLength)
	{
		grid = this->gridLength - 1;
	}

	return grid;
}

/**
	Function that calculates the corresponding coordinate from the
	occupancy grid index

	@param	grid		int
	@return	coordinate	float
*/
float Pathplanner::getCoordinate(int grid)
{
	float coordinate = (grid - this->gridCenter) * this->gridResolution;
	return coordinate;
}

/**
	Function that calculates the column in which an obstacle (i.e. grid indices 
	corresponding to a value greater than 75 (obstacle))

	@return	column	int
*/
int Pathplanner::getXObstacleInPath()
{
	float currentX = this->currentPose.pose.position.x;
	float currentY = this->currentPose.pose.position.y;
	
	int currentRow = this->getGrid(currentY);
	int currentColumn = this->getGrid(currentX);
	
	int rowCheckHeight = 20,
		columnIncrement,
		startColumn = currentColumn,
		endColumn,
		currentCheckColumn = startColumn,
		columnsToCheck,
		startRow = currentRow + (rowCheckHeight / 2),
		endRow = startRow - rowCheckHeight,
		currentCheckRow = startRow,
		rowsToCheck,
		obstacleCells = 0,
		obstacleCellsTreshold = 1;
		
	// Clamp startRow
	if(startRow > this->boundaryBoxGridUpperBound)
	{
		startRow = currentCheckRow = this->boundaryBoxGridUpperBound;
	}
	
	// Clamp endRow
	if(endRow < this->boundaryBoxGridLowerBound)
	{
		endRow = this->boundaryBoxGridLowerBound;
	}
	
	// Direction dependent variables
	if(this->sweepingRight)
	{
		columnIncrement = 1;
		endColumn = this->boundaryBoxGridUpperBound;
	}
	else
	{
		columnIncrement = -1;
		endColumn = this->boundaryBoxGridLowerBound;
	}
	
	ROS_INFO("Current X: %f, Y: %f", currentX, currentY);
	ROS_INFO("Current Row: %i, Column: %i", currentRow, currentColumn);
	ROS_INFO("Sweeping Right? %i", this->sweepingRight);
	
	columnsToCheck = abs(startColumn - endColumn);
	
	while(columnsToCheck > 0)
	{
		obstacleCells = 0;
		currentCheckRow = startRow;
		rowsToCheck = abs(currentCheckRow - endRow);

		while(rowsToCheck > 0)
		{
			if(this->occupancyGrid[currentCheckRow][currentCheckColumn] >= 75)
			{
				obstacleCells++;
				
				ROS_INFO("Obstacle! row: %i, column: %i, value: %i", currentCheckRow, currentCheckColumn, occupancyGrid[currentCheckRow][currentCheckColumn]);
				
				if(obstacleCells > obstacleCellsTreshold)
				{
					ROS_INFO("Returning XObst Column: %i", currentCheckColumn);
					return currentCheckColumn;
				}
			}
			
			currentCheckRow--;
			rowsToCheck = abs(currentCheckRow - endRow);
		}
		
		currentCheckColumn += columnIncrement;
		columnsToCheck = (currentCheckColumn > 0) ? abs(startColumn - endColumn) : 0;
	}
	
	return endColumn;
}

int Pathplanner::getYObstacleInPath()
{
	float currentX = this->currentPose.pose.position.x;
	float currentY = this->currentPose.pose.position.y;
	
	int currentRow = this->getGrid(currentY);
	int currentColumn = this->getGrid(currentX);
	
	int columnCheckWidth = 20,
		startColumn = currentColumn + (columnCheckWidth / 2),
		endColumn = startColumn - columnCheckWidth,
		currentCheckColumn = startColumn,
		columnsToCheck,
		startRow = currentRow,
		endRow = this->boundaryBoxGridLowerBound,
		currentCheckRow = startRow,
		rowsToCheck,
		obstacleCells = 0,
		obstacleCellsTreshold = 1;
		
	// Clamp start column
	if(startColumn > this->boundaryBoxGridUpperBound)
	{
		startColumn = currentCheckRow = this->boundaryBoxGridUpperBound;
	}
	
	// Clamp endColumn
	if(endColumn < this->boundaryBoxGridLowerBound)
	{
		endColumn = this->boundaryBoxGridLowerBound;
	}

	rowsToCheck = abs(currentCheckRow - endRow);
	
	while(rowsToCheck > 0)
	{
		obstacleCells = 0;
		currentCheckColumn = startColumn;
		columnsToCheck = abs(startColumn - endColumn);

		while(columnsToCheck > 0)
		{
			if(this->occupancyGrid[currentCheckRow][currentCheckColumn] >= 75)
			{
				obstacleCells++;
				
				if(obstacleCells > obstacleCellsTreshold)
				{
					ROS_INFO("Returning YObst Row: %i", currentCheckRow);
					return currentCheckRow;
				}
			}
			
			currentCheckColumn--;
			columnsToCheck = abs(currentCheckColumn - endColumn);
		}
		
		currentCheckRow--;
		rowsToCheck = (currentCheckRow > 0) ? abs(currentCheckRow - endRow) : 0;
	}
	
	return endRow;
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
	//float currentZ = this->currentPose.pose.orientation.z;
	//float currentAngle = 2 * acos(currentZ) * (180 / 3.14);

	geometry_msgs::PointStamped targetPoint;

	targetPoint.header.stamp = ros::Time::now();
	targetPoint.header.frame_id = "map";

	int nearestObstacleColumn = this->getXObstacleInPath();
	float xCoordinateOfObstacleColumn = this->getCoordinate(nearestObstacleColumn);
	
	int nearestObstacleRow = this->getYObstacleInPath();
	float yCoordinateOfObstacleRow = this->getCoordinate(nearestObstacleRow);
	
	ROS_INFO("xCoordinateOfObstacleColumn: %f", xCoordinateOfObstacleColumn);

	double distanceToXObstacle = fabs(currentX - xCoordinateOfObstacleColumn);
	double distanceToYObstacle = fabs(currentY - yCoordinateOfObstacleRow);

	if(this->sweepingRight)
	{
		if(this->dropWhenPossible && distanceToYObstacle > MAX_DROP_DISTANCE)
		{
			targetPoint.point.x = currentX;
			targetPoint.point.y = currentY - MAX_DROP_DISTANCE;
			this->dropWhenPossible = false;
			
			ROS_WARN("DROPING 2! Dist to YObst: %f", distanceToYObstacle);
		}
		else if(distanceToXObstacle > MAX_MOVE_DISTANCE)
		{
			targetPoint.point.x = currentX + MAX_MOVE_DISTANCE;
			targetPoint.point.y = currentY;
		}
		else
		{
			// ROS_INFO("Hit right side, obstacle column: %f, %i", distanceToXObstacle, (distanceToXObstacle >= MAX_MOVE_DISTANCE));
			targetPoint.point.x = currentX; //xCoordinateOfObstacleColumn;
			
			this->sweepingRight = !this->sweepingRight;

			if(distanceToYObstacle < MAX_DROP_DISTANCE)
			{
				this->dropWhenPossible = true;
				targetPoint.point.y = currentY;
			}
			else
			{
				targetPoint.point.y = currentY - MAX_DROP_DISTANCE;
				ROS_WARN("DROPING! Dist to YObst: %f", distanceToYObstacle);
			}
		}
	}
	else //else if(!this->sweepingRight)
	{
		if(this->dropWhenPossible && distanceToYObstacle > MAX_DROP_DISTANCE)
		{
			targetPoint.point.x = currentX;
			targetPoint.point.y = currentY - MAX_DROP_DISTANCE;
			this->dropWhenPossible = false;
			
			ROS_WARN("DROPING 2! Dist to YObst: %f", distanceToYObstacle);
		}
		else if(distanceToXObstacle > MAX_MOVE_DISTANCE)
		{
			targetPoint.point.x = currentX - MAX_MOVE_DISTANCE;
			targetPoint.point.y = currentY;
		}
		else
		{
			// ROS_INFO("Hit left side, obstacle column: %f, %i", distanceToXObstacle, (distanceToXObstacle >= MAX_MOVE_DISTANCE));
			targetPoint.point.x = currentX; //xCoordinateOfObstacleColumn;
			this->sweepingRight = !this->sweepingRight;	

			if(distanceToYObstacle < MAX_DROP_DISTANCE)
			{
				this->dropWhenPossible = true;
				targetPoint.point.y = currentY;
			}
			else
			{
				targetPoint.point.y = currentY - MAX_DROP_DISTANCE;
				ROS_WARN("DROPING! Dist to YObst: %f", distanceToYObstacle);		
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
	
	this->poseUpdated = false;

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
	ROS_INFO("------------------------------------");
	ROS_INFO("-- getTargetPoint Service Called! --");
	ROS_INFO("------------------------------------");
	
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
	
	if(ros::ok())
	{
		// Set the response
		res.targetPoint = this->getTargetPoint();
		ROS_INFO("Current target: X: %f, Y: %f", res.targetPoint.point.x, res.targetPoint.point.y);
		return true;
	}
	else
	{
		return false;
	}
}



