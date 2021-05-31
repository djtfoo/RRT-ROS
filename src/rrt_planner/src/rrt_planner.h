#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/RrtNode.h>
#include <nav_msgs/PathRequest.h>

#include <vector>

#include "rrt/rrt.h"
#include "rrt/coord.h"

class RrtPlanner
{
public:
    // Constructor
    RrtPlanner(ros::NodeHandle& nh);

protected:
    // Function to plan path
    static void planPath(const Coord& start, const Coord& goal);

    // RRT helper functions to do stuff like compute Voronoi region, get config, collision detection, etc
    static Rrt* buildRrt(Rrt* rrt, int iters, const Coord& goal);
    static void randomState(Coord* state, Coord* currState, const Coord& goal, int radius);
    static int extend(Rrt* rrt, const Coord& state, Rrt** xNew, const Coord& goal);

    static Rrt* nearestNeighbour(Rrt* rrt, const Coord& state);
    static Rrt* nearestNeighbourSearch(Rrt* rrt, const Coord& state, float* dist);
    static float neighbourDistanceMetric(Rrt* rrt, const Coord& state);

    static bool newState(const Coord& state, Rrt* xNear, float input, Coord* nState, const Coord& goal);
    static bool noCollision(const Coord& startState, const Coord& newState);

    // publish path
    static void publishPath(Rrt* goalNode);

private:
    // Subscriber
    static ros::Subscriber map_sub_;
    static ros::Subscriber pathreq_sub_;

    // Subscriber callback
    static void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    static void pathreqCallback(const nav_msgs::PathRequest::ConstPtr& pathreq);
    static nav_msgs::OccupancyGrid::ConstPtr map_;

    static bool isObstacle(const Coord& coord);
    static bool isObstacle(int gridX, int gridY);

    // Publisher
    static ros::Publisher path_pub_;
    static ros::Publisher rrt_pub_;

    // Publisher function
    static void publishRrtNode(Rrt* node);

    // RRT variables
    // TODO: maybe make start and end static variables instead of constantly passing it as a variable
    //static Coord start;
    //static Coord end;
    static float _incrementalStep;  // represents input to next state (TBD: change to a class to accommodate nonholonomy/more complicated inputs)

    // overall state of map for randomState sampling
    const static int numRegionsX = 20;
    const static int numRegionsY = 20;
    const static int regionSizeX = 25;
    const static int regionSizeY = 25;
    static std::vector<int> unpopulatedRegions;
};

#endif	// RRT_PLANNER_H_
