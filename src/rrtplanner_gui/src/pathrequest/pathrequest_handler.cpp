#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "pathrequest_handler.h"
#include "../msg_visualizer.h"

using namespace cv;

bool PathRequestHandler::isObstacle(const nav_msgs::OccupancyGrid::ConstPtr& map, int gridX, int gridY) {

    // get grid index
    int idx = gridY * (map->width/map->gridsize) + gridX;

    // check if grid is obstacle
    return map->occupancy[idx];
}

void PathRequestHandler::clearGrid(VisualizerWindow* window, int gridX, int gridY, int gridsize) {
    MsgVisualizer::fillGrid(window, gridX, gridY, gridsize, Scalar(0, 0, 0));
}

void PathRequestHandler::fillStartGrid(VisualizerWindow* window, int gridX, int gridY, int gridsize) {
    MsgVisualizer::fillGrid(window, gridX, gridY, gridsize, Scalar(0, 255, 0));
}

void PathRequestHandler::fillGoalGrid(VisualizerWindow* window, int gridX, int gridY, int gridsize) {
    MsgVisualizer::fillGrid(window, gridX, gridY, gridsize, Scalar(0, 0, 255));
}

