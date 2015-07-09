#include "ros/ros.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "roboteq/MotorCommands.h"
#include "navigation/getTargetPoint.h"
#include <cmath>
#include <vector>

#define MAX_MOVE_DISTANCE 0.5		// maximum distance traveled during intermediate steps in meters
#define MAX_DROP_DISTANCE 1.0		// maximum distance dropped in the y direction during sweep
#define BOUNDARY_BOX_SIZE 10		// size of boundary map in meters
#define MAP_SIZE 30
//#define 

#ifndef __pathplanner_h_
#define __pathplanner_h_

class Pathplanner
{
	private:
		ros::NodeHandle node;
		ros::Subscriber map_subscriber;
		ros::Subscriber pose_subscriber;
		ros::ServiceServer getTargetPointService;
		bool sweepingRight;
		bool dropWhenPossible;
		bool poseUpdated;

		int gridLength;
		float gridResolution;
		int gridCenter;
		int boundaryBoxGridLength;
		int boundaryBoxGridLowerBound;
		int boundaryBoxGridUpperBound;

		// 2D array copy of occupancy grid
		int occupancyGrid[1200][1200];		// want to dynamically resize array later

	public:
		Pathplanner(ros::NodeHandle rosNode);
		void processMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void processPose(geometry_msgs::PoseStamped msg);
		int getXObstacleInPath();
		int getYObstacleInPath();
		int getGrid(float coord);
		float getCoordinate(int grid);
		geometry_msgs::PointStamped getTargetPoint();


		ros::Publisher targetPointPublisher;
		geometry_msgs::PoseStamped currentPose;

		// Service Callbacks
		bool getTargetPoint(navigation::getTargetPoint::Request  &req, 
		                     navigation::getTargetPoint::Response &res);
};

#endif
