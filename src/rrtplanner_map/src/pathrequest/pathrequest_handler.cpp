#include <opencv2/imgproc.hpp>

#include <iostream>

#include "pathrequest_handler.h"
#include "../msg_handler.h"

using namespace cv;

bool PathRequestHandler::isWithinBounds(const nav_msgs::OccupancyGrid::ConstPtr& map, int gridX, int gridY) {
    return (gridX < 0 || gridX >= (map->width / map->gridsize) || gridY < 0 || gridY >= (map->height / map->gridsize));
}

bool PathRequestHandler::isObstacle(const nav_msgs::OccupancyGrid::ConstPtr& map, int gridX, int gridY) {

    // do a bounds check first
    if (!isWithinBounds(map, gridX, gridY))
        return false;

    // get grid index
    int idx = gridY * (map->width/map->gridsize) + gridX;
std::cout << std::endl;

    // check if grid is obstacle
    if (map->occupancy[idx])  // is obstacle
        return true;
}
