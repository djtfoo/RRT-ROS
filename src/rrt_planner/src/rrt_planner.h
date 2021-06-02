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

    // Function to plan path
    void planPath(nav_msgs::OccupancyGrid::ConstPtr map, const Coord& start, const Coord& goal);

protected:

    // RRT helper functions
    Rrt* buildRrt(Rrt* rrt, int iters, const Coord& goal);
    void randomState(Coord* state, Coord* currState, const Coord& goal, int radius);
    int extend(Rrt* rrt, const Coord& state, Rrt** xNew);
    virtual void addToRrt(Rrt* rrt, Rrt* xNear, Rrt* xNew);

    Rrt* nearestNeighbour(Rrt* rrt, const Coord& state);
    Rrt* nearestNeighbourSearch(Rrt* rrt, const Coord& state, float* dist);
    float neighbourDistanceMetric(Rrt* rrt, const Coord& state);

    bool newState(const Coord& state, Rrt* xNear, float input, Coord* nState);
    bool noCollision(const Coord& startState, const Coord& newState);

    // Publisher functions
    void publishRrtNode(Rrt* node);

    // RRT variables
    float _incrementalStep;  // represents input to next state

private:
    nav_msgs::OccupancyGrid::ConstPtr map_;

    // Publisher
    ros::Publisher path_pub_;
    ros::Publisher rrt_pub_;
    // Publisher functions
    void publishPath(Rrt* goalNode);

    // overall state of map for randomState sampling
    const int numRegionsX = 50;//20;
    const int numRegionsY = 50;//20;
    const int regionSizeX = 10;//25;
    const int regionSizeY = 10;//25;
    std::vector<int> unpopulatedRegions;

    // RRT helper functions
    bool isObstacle(const Coord& coord);
    bool isObstacle(int gridX, int gridY);
};

#endif	// RRT_PLANNER_H_
