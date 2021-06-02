#include <opencv2/imgproc.hpp>

#include "pathrequest_handler.h"
#include "../msg_handler.h"

using namespace cv;

bool PathRequestHandler::isWithinBounds(const nav_msgs::OccupancyGrid::ConstPtr& map, int gridX, int gridY) {
    return (gridX < 0 || gridX >= (map->width / map->gridsize) || gridY < 0 || gridY >= (map->height / map->gridsize));
}

bool PathRequestHandler::isObstacle(const nav_msgs::OccupancyGrid::ConstPtr& map, int gridX, int gridY) {

    // get grid index
    int idx = gridY * (map->width/map->gridsize) + gridX;

    // check if grid is obstacle
    if (map->occupancy[idx])  // is obstacle
        return true;

    // also do a bounds check
    return isWithinBounds(map, gridX, gridY);
}
