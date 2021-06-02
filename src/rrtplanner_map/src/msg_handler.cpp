#include "msg_handler.h"

#include <iostream>
#include <ros/ros.h>

std::map<int, nav_msgs::RrtNode::ConstPtr> MsgHandler::rrtNodes;

void MsgHandler::parseMap(VisualizerWindow** window, const nav_msgs::OccupancyGrid::ConstPtr& map, bool showGridLines) {
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
    drawMap(*window, map, false);

    // draw grid lines if enabled
    if (showGridLines) {
        // draw vertical lines
        for (int i = 1; i < gridWidth; ++i) {
            (*window)->drawLine(
                Point(i*map->gridsize - 1, 0),
                Point(i*map->gridsize - 1, map->height),
                Scalar(255,255,255),
                1
             );
        }
        // draw horizontal lines
        int gridHeight = map->height / map->gridsize;  // no. grids along height
        for (int i = 1; i < gridHeight; ++i) {
            (*window)->drawLine(
                Point(0, i*map->gridsize - 1),
                Point(map->width, i*map->gridsize - 1),
                Scalar(255,255,255),
                1
             );
        }
    }
}

void MsgHandler::drawMap(VisualizerWindow* window, const nav_msgs::OccupancyGrid::ConstPtr& map, bool fillEmpty) {
    // no. grids along width
    int gridWidth = map->width / map->gridsize;

    // draw obstacles
    for (int i = 0; i < map->occupancy.size(); ++i) {
        //std::cout << (map->occupancy[i] ? 1 : 0);
        // compute grid coordinate
        int grid_x = i % gridWidth;
        int grid_y = i / gridWidth;
        if (map->occupancy[i]) {  // is obstacle
            fillGrid(window, grid_x, grid_y, map->gridsize, Scalar(255, 255, 255));
        }
        else if (fillEmpty) {  // empty cell
            fillGrid(window, grid_x, grid_y, map->gridsize, Scalar(0, 0, 0));
        }
    }
}

void MsgHandler::fillGrid(VisualizerWindow* window, int gridX, int gridY, int gridsize, const Scalar& color) {
    // compute min pixel coordinate
    int pixel_x = gridX * gridsize;
    int pixel_y = gridY * gridsize;
    // fill the grid with white
    window->drawRectangle(
        Point(pixel_x, pixel_y),
        Point(pixel_x+gridsize, pixel_y+gridsize),
        color,
        -1   // FILLED
    );

}

void MsgHandler::parsePathRequest(VisualizerWindow* window, const nav_msgs::PathRequest::ConstPtr& pathreq) {

    std::cout << "Start: " << pathreq->start_x << "," << pathreq->start_y << std::endl;
    // draw start node
    window->drawCircle(
        Point(pathreq->start_x, pathreq->start_y),
        3,
        Scalar(255, 150, 0),
        -1   // FILLED
    );

    std::cout << "Goal: " << pathreq->goal_x << "," << pathreq->goal_y << std::endl;
    // draw goal node
    window->drawCircle(
        Point(pathreq->goal_x, pathreq->goal_y),
        2,
        Scalar(255, 150, 0),
        -1   // FILLED
    );
}

void MsgHandler::parseRrtNode(VisualizerWindow* window, const nav_msgs::RrtNode::ConstPtr& rrtNode, const nav_msgs::OccupancyGrid::ConstPtr& map, const nav_msgs::PathRequest::ConstPtr& pathreq) {

    // check if RRT Node is a root node
    if (rrtNode->parent == -1)  // node has no parent
        rrtNodes.clear();

    //std::cout << rrtNode->id << ", " << rrtNode->parent << std::endl;

    // if rrtNode already in map, means it got updated; erase the old edge and redraw
    std::map<int, nav_msgs::RrtNode::ConstPtr>::iterator it;
    it = rrtNodes.find(rrtNode->id);
    if (it != rrtNodes.end()) {  // rrtNode already in map
        // Erase the removed edge
        nav_msgs::RrtNode::ConstPtr parent = rrtNodes[it->second->parent];
        window->drawLine(
            Point(parent->x, parent->y),
            Point(it->second->x, it->second->y),
            Scalar(0, 0, 0),
            1
        );
        // Redraw all edges in case they got erased
        std::map<int, nav_msgs::RrtNode::ConstPtr>::iterator it2;
        for(it2 = rrtNodes.begin(); it2 != rrtNodes.end(); ++it2) {
            // skip the erased edge
            if (it == it2) continue;
            // draw edge
            nav_msgs::RrtNode::ConstPtr node = it2->second;
            if (node->parent != -1) {
                nav_msgs::RrtNode::ConstPtr parent = rrtNodes[node->parent];
                window->drawLine(
                    Point(parent->x, parent->y),
                    Point(node->x, node->y),
                    Scalar(255, 150, 0),
                    1
                );
            }
        }
        // Redraw start and goal nodes in case they got erased
        parsePathRequest(window, pathreq);
    }
    rrtNodes[rrtNode->id] = rrtNode;  // add updated rrtNode to map

    // draw new edge from rrtNode to parent
    if (rrtNode->parent != -1) {
    //std::cout << "Draw" << std::endl;
        //std::cout << "Trying to draw ..." << std::endl;
        nav_msgs::RrtNode::ConstPtr parent = rrtNodes[rrtNode->parent];
        //std::cout << rrtNode->parent << ": " << parent->x << "," << parent->y << " | " << rrtNode->id << ": " << rrtNode->x << "," << rrtNode->y << std::endl;
        window->drawLine(
            Point(parent->x, parent->y),
            Point(rrtNode->x, rrtNode->y),
            Scalar(255, 150, 0),
            1
         );
    //std::cout << "Drawn" << std::endl;
    }
}

void MsgHandler::parsePath(VisualizerWindow* window, const nav_msgs::Path::ConstPtr& path) {

    // allow map to remain visible for a slightly longer time
    ros::Duration(0.5f).sleep();

    std::cout << "Path: " << std::endl;
    // draw path with a different color and thickness
    for (int i = path->path.size() - 1; i >= 0; --i) {
        // print id of current node
        std::cout << path->path[i].id << ", ";

        // root node
        if (path->path[i].parent == -1)
            continue;

        // there is an edge connecting nodes
        nav_msgs::RrtNode::ConstPtr thisNode = rrtNodes[path->path[i].id];
        nav_msgs::RrtNode::ConstPtr parent = rrtNodes[path->path[i].parent];
        // draw line
        window->drawLine(
            Point(parent->x, parent->y),
            Point(thisNode->x, thisNode->y),
            Scalar(255, 0, 255),  // magenta
            2
         );
    }
    std::cout << std::endl;
    // draw start node
    int startIdx = path->path.size() - 1;
    window->drawCircle(
        Point(path->path[startIdx].x, path->path[startIdx].y),
        4,
        Scalar(0, 255, 255),  // yellow
        -1   // FILLED
    );

    // draw goal node
    window->drawCircle(
        Point(path->path[0].x, path->path[0].y),
        4,
        Scalar(0, 0, 255),  // red
        -1   // FILLED
    );
}
