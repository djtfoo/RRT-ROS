#include "msg_visualizer.h"
#include <iostream>  // for testing only
#include <ros/ros.h>  // for testing only

std::map<int, nav_msgs::RrtNode::ConstPtr> MsgVisualizer::rrtNodes;

void MsgVisualizer::parseMap(VisualizerWindow** window, const nav_msgs::OccupancyGrid::ConstPtr& map) {
    std::cout << "Parse Map" << std::endl;
    std::cout << map->name << std::endl;
    std::cout << map->width << ", " << map->height << ", " << (int)map->gridsize << "," << std::endl;

    // create window
    // TODO: mutex
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
}

void MsgVisualizer::parseRrtNode(VisualizerWindow** window, const nav_msgs::RrtNode::ConstPtr& rrtNode) {

    // check if RRT Node is a root node
    if (rrtNode->parent == -1)  // node has no parent
        rrtNodes.clear();

    // add rrtNode to map
    rrtNodes[rrtNode->id] = rrtNode;

    // draw edge from rrtNode to parent
    if (rrtNode->parent != -1) {
        std::cout << "Trying to draw ..." << std::endl;
        nav_msgs::RrtNode::ConstPtr parent = rrtNodes[rrtNode->parent];
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

    // sleep for a short time to allow map to remain visible
    ros::Duration(1.f).sleep();

    std::cout << "Path: " << std::endl;
    // draw path with a different color and thickness
    for (int i = path->path.size() - 1; i >= 0; --i) {
        // print id of current node
        std::cout << path->path[i].id << ", ";
        if (path->path[i].parent == -1)
            continue;
        nav_msgs::RrtNode::ConstPtr thisNode = rrtNodes[path->path[i].id];
        nav_msgs::RrtNode::ConstPtr parent = rrtNodes[path->path[i].parent];

        // draw line
        (*window)->drawLine(
            Point(parent->x, parent->y),
            Point(thisNode->x, thisNode->y),
            Scalar(0, 255, 255),
            2
         );
    }

    // TODO: draw start and end nodes as circles

}
