#ifndef MSG_VISUALIZER_H_
#define MSG_VISUALIZER_H_

#include <map>

#include <nav_msgs/OccupancyGrid.h>
#include <rrt_planner/RrtNode.h>
#include <nav_msgs/Path.h>

#include "visualizer_window.h"

class MsgVisualizer {

    // list of RRT Nodes
    static std::map<int, rrt_planner::RrtNode::ConstPtr> rrtNodes;

public:
    static void parseMap(VisualizerWindow** window, const nav_msgs::OccupancyGrid::ConstPtr& map);
    static void parseRrtNode(VisualizerWindow** window, const rrt_planner::RrtNode::ConstPtr& rrtNode);
    static void parsePath(VisualizerWindow** window, const nav_msgs::Path::ConstPtr& path);

};

#endif  // MSG_VISUALIZER_H_