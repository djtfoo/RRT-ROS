#ifndef PATHREQUEST_HANDLER_H_
#define PATHREQUEST_HANDLER_H_

#include <nav_msgs/OccupancyGrid.h>
#include "../visualizer_window.h"

class PathRequestHandler {
public:
    // Initialization
    void init();

    // Map obstacle checker
    static bool isObstacle(const nav_msgs::OccupancyGrid::ConstPtr& map, int gridX, int gridY);

    // grid fill helper functions
    static void clearGrid(VisualizerWindow* window, int gridX, int gridY, int gridsize);
    static void fillStartGrid(VisualizerWindow* window, int gridX, int gridY, int gridsize);
    static void fillGoalGrid(VisualizerWindow* window, int gridX, int gridY, int gridsize);
};


#endif  // PATHREQUEST_HANDLER_H_
