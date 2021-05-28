#include "planner.h"

// Constructor
Planner::Planner(ros::NodeHandle& nh) {
    // TODO: sub/advertise options
    map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map");
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 1);
}

// publish path to topic
void Planner::publish_path(std::vector<Coord2D>& path) {

    nav_msgs::Path pathMsg;

    // TODO: populate pathMsg

    // publish pathMsg to topic
    path_pub_.publish(pathMsg);
}
