#include "msg_visualizer.h"
#include <iostream>  // for testing only
#include <ros/ros.h>  // for testing only

std::map<int, rrt_planner::RrtNode::ConstPtr> MsgVisualizer::rrtNodes;

void MsgVisualizer::parseMap(VisualizerWindow** window, const nav_msgs::OccupancyGrid::ConstPtr& map) {
    std::cout << "Parse Map" << std::endl;
    std::cout << map->name << std::endl;
    std::cout << map->width << ", " << map->height << ", " << (int)map->gridsize << "," << std::endl;

    // create window
    if (*window != nullptr)
        delete *window;
    *window = new VisualizerWindow(map->name.c_str(), map->width, map->height);

    // no. grids along width
    int gridWidth = map->width / map->gridsize;

    // draw obstacles
    for (int i = 0; i < map->occupancy.size(); ++i) {
        //std::cout << (map->occupancy[i] ? 1 : 0);
        if (map->occupancy[i]) {  // is obstacle
            // compute grid coordinate
            int grid_x = i % gridWidth;
            int grid_y = i / gridWidth;
            // compute min pixel coordinate
            int pixel_x = grid_x*map->gridsize;
            int pixel_y = grid_y*map->gridsize;
            // fill the grid with white
            (*window)->drawRectangle(
                Point(pixel_x, pixel_y),
                Point(pixel_x+map->gridsize, pixel_y+map->gridsize),
                Scalar(255,255,255),
                -1   // FILLED
             );
        }
    }

    // TODO: draw grid lines

}

void MsgVisualizer::parseRrtNode(VisualizerWindow** window, const rrt_planner::RrtNode::ConstPtr& rrtNode) {

    // check if RRT Node is a root node
    if (rrtNode->parent == -1)  // node has no parent
        rrtNodes.clear();

    // add rrtNode to map
    rrtNodes[rrtNode->id] = rrtNode;
    std::cout << rrtNode->parent << " | " << rrtNode->id << ": " << rrtNode->x << "," << rrtNode->y << std::endl;

    // draw edge from rrtNode to parent
    if (rrtNode->parent != -1) {
        std::cout << "Trying to draw ..." << std::endl;
        rrt_planner::RrtNode::ConstPtr parent = rrtNodes[rrtNode->parent];
        std::cout << rrtNode->parent << ": " << parent->x << "," << parent->y << " | " << rrtNode->id << ": " << rrtNode->x << "," << rrtNode->y << std::endl;
        (*window)->drawLine(
            Point(parent->x, parent->y),
            Point(rrtNode->x, rrtNode->y),
            Scalar(255, 255, 0),
            1
         );
    }
}

void MsgVisualizer::parsePath(VisualizerWindow** window, const nav_msgs::Path::ConstPtr& path) {

    // TODO: parse Path msg

}
