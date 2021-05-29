#include <ros/ros.h>

#include "rrt_planner.h"
#include "rrt/rrt.h"

// Constructor
RrtPlanner::RrtPlanner(ros::NodeHandle& nh) {
    // TODO: sub/advertise options
    map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, mapCallback);
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 1);
}

// Publish path to ROS topic
void RrtPlanner::publishPath(std::vector<nav_msgs::PathNode>& path) {

    nav_msgs::Path pathMsg;

    // TODO: populate pathMsg

    // publish pathMsg to topic
    path_pub_.publish(pathMsg);
}

// Subscriber callback
void RrtPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    ROS_INFO("WEW");
}

// Path planner algorithm (RRT)
void RrtPlanner::planPath(const Coord& start, const Coord& end) {

    // initialise RRT at start position
    Rrt node(start, nullptr);

    // compute RRT
    

    // publish the path
    //publishPath();
}

// RRT helper functions
