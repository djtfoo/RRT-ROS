#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_

#include <ros/ros.h>
#include <nav_msgs/PathNode.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <vector>

#include "rrt/rrt.h"
#include "rrt/coord.h"

class RrtPlanner
{
public:
    // Constructor
    RrtPlanner(ros::NodeHandle& nh);

protected:
    // plan path
    void planPath(const Coord& start, const Coord& goal);

    // RRT stuff
    //probably helper funcs to do stuff like compute Voronoi region, get config, collision detection, etc
    void randomState();
    void extend(Rrt* rrt);

    // publish path
    void publishPath(std::vector<nav_msgs::PathNode>& path);

private:
    // Subscriber
    ros::Subscriber map_sub_;

    // Subscriber callback
    static void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    static nav_msgs::OccupancyGrid::ConstPtr map_;

    // Publisher
    ros::Publisher path_pub_;

    // RRT stuff
    float incrementalStep;  // represents input to next state (TBD: change to a class to accommodate nonholonomy)
};

#endif	// RRT_PLANNER_H_
