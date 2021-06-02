#ifndef PATHREQUEST_HANDLER_H_
#define PATHREQUEST_HANDLER_H_

#include <nav_msgs/OccupancyGrid.h>

class PathRequestHandler {
public:
    // Initialization
    void init();

    // Map obstacle checker
    static bool isWithinBounds(const nav_msgs::OccupancyGrid::ConstPtr& map, int gridX, int gridY);
    static bool isObstacle(const nav_msgs::OccupancyGrid::ConstPtr& map, int gridX, int gridY);
};


#endif  // PATHREQUEST_HANDLER_H_
