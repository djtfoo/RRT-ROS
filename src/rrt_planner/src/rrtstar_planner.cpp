#include "rrtstar_planner.h"

RrtStarPlanner::RrtStarPlanner(ros::NodeHandle& nh)
 : RrtPlanner(nh) {}


void RrtStarPlanner::addToRrt(Rrt* rrt, Rrt* xNear, Rrt* xNew) {
    // wrapper call to RRT* node insertion and path optimisation
    addAndOptimiseNodes(rrt, xNew);
}


void RrtStarPlanner::addAndOptimiseNodes(Rrt* rrt, Rrt* xNew) {
    // get nearby nodes to xNew
    std::vector<Rrt*> nearbyNodes;
    float neighbourhoodRadius = (_incrementalStep * 10.f);
    getNearbyNodes(rrt, xNew, neighbourhoodRadius*neighbourhoodRadius, nearbyNodes);

    if (nearbyNodes.size() == 0) {  // should not happen
        return;
    }

    // remove parent first
    xNew->setParent(nullptr);

    // compute distances between xNew and nearby nodes (will be used in the following steps)
    std::vector<float> distances;
    for (int i = 0; i < nearbyNodes.size(); ++i) {
        distances.push_back(xNew->distanceToNode(nearbyNodes[i]));
    }

    // find optimal path from root to xNew
    float shortestPath = computePathLength(nearbyNodes[0]) + distances[0];
    int nodeIdx = 0;
    std::cout << "Leggo" << std::endl;
    for (int i = 1; i < nearbyNodes.size(); ++i) {
        float pathLength = computePathLength(nearbyNodes[i]);
        pathLength += distances[i];
        // check if it is new shortest path length
        if (pathLength < shortestPath) {
            shortestPath = pathLength;
            nodeIdx = i;
        }
    }
    // add xNew to nearbyNode with the shortest path length
    nearbyNodes[nodeIdx]->addChild(xNew);
    // set new parent
    xNew->setParent(nearbyNodes[nodeIdx]);

    // publish xNew
    publishRrtNode(xNew);

    // Rewire step
    std::cout << "Rewire step" << std::endl;
    // check all nearby nodes if their path can be optimised with xNew
    // (check if (path of root to nearby node) < (path of root to xNew + xNew to nearby node))
    for (int i = 1; i < nearbyNodes.size(); ++i) {
        // skip checking xNew's parent
        /// (by right, can skip all of the nodes on the path from root to xNew)
        if (i == nodeIdx) {  // xNew's parent node
            continue;
        }

        // compute length of path from root to node to be checked (i.e. nearby node)
        float pathLength = computePathLength(nearbyNodes[i]);
        if (shortestPath + distances[i] < pathLength) {  // a shorter path is found
            // rewire only if it does not pass through obstacles
            // get vector
            float vecX = xNew->getCoord()->_x - nearbyNodes[i]->getCoord()->_x;
            float vecY = xNew->getCoord()->_y - nearbyNodes[i]->getCoord()->_y ;
            // find unit vector
            float unitVecX = vecX / distances[i];
            float unitVecY = vecY / distances[i];
            bool rewire = true;
            for (float dist = 0; dist < distances[i]; dist += 0.5f*_incrementalStep) {
                // check there is an obstacle at every incremental step
                Coord point(nearbyNodes[i]->getCoord()->_x + unitVecX * dist, nearbyNodes[i]->getCoord()->_y + unitVecY * dist);

                if (!noCollision(*(nearbyNodes[i]->getCoord()), point)) {
                    rewire = false;
                    break;
                }
            }
            // rewire nearby node if no obstacles
            if (rewire) {
                // rewire by setting new parent
                nearbyNodes[i]->setParent(xNew);
                // publish changed node
                publishRrtNode(nearbyNodes[i]);
            }
        }
    }
}


void RrtStarPlanner::getNearbyNodes(Rrt* rrt, Rrt* xNew, float radiusSqr, std::vector<Rrt*>& nearbyNodes) {

    // check if this node is nearby
    if (rrt->distanceSquaredToNode(xNew) <= radiusSqr) {
        nearbyNodes.push_back(rrt);
    }

    // recursively visit all children nodes in tree
    Rrt* nearestNeighbour = rrt;
    for (int i = 0; i < rrt->getNumChildren(); ++i) {
        getNearbyNodes(rrt->getChild(i), xNew, radiusSqr, nearbyNodes);
    }
}


float RrtStarPlanner::computePathLength(Rrt* node) {
    float pathLength = 0.f;
    while (node->getParent() != nullptr) {
        // add length of current edge
        pathLength += node->distanceToNode(node->getParent());
        // go to next node
        node = node->getParent();
    }
    return pathLength;
}
