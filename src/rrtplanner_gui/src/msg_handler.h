#ifndef MSG_HANDLER_H_
#define MSG_HANDLER_H_

#include <map>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/PathRequest.h>
#include <nav_msgs/RrtNode.h>
#include <nav_msgs/Path.h>

#include "gui/visualizer_window.h"

class MsgHandler {

    // list of RRT Nodes
    static std::map<int, nav_msgs::RrtNode::ConstPtr> rrtNodes;

public:
    // Parse topic messages
    static void parseMap(VisualizerWindow** window, const nav_msgs::OccupancyGrid::ConstPtr& map, bool showGridLines);
    static void parsePathRequest(VisualizerWindow* window, const nav_msgs::PathRequest::ConstPtr& pathreq);
    static void parseRrtNode(VisualizerWindow* window, const nav_msgs::RrtNode::ConstPtr& rrtNode);
    static void parsePath(VisualizerWindow* window, const nav_msgs::Path::ConstPtr& path);

    // helper functions for visualization
    static void fillGrid(VisualizerWindow* window, int gridX, int gridY, int gridsize, const Scalar& color);
};

#endif  // MSG_HANDLER_H_
