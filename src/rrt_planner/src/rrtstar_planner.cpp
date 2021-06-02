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

    // TODO: remove parent first, but currently buggy
    xNew->getParent()->removeChild(xNew);
    xNew->setParent(nullptr);

    // compute distances between xNew and nearby nodes (will be used in the following steps)
    std::vector<float> distances;
    for (int i = 0; i < nearbyNodes.size(); ++i) {
        distances.push_back(xNew->distanceToNode(nearbyNodes[i]));
    }

    // find optimal path from root to xNew
    float shortestPath = neighbourhoodRadius*neighbourhoodRadius + 1.f;
    int nodeIdx = -1;
    std::cout << "Leggo" << std::endl;
    for (int i = 0; i < nearbyNodes.size(); ++i) {
        float pathLength = computePathLength(nearbyNodes[i]) + distances[i];
        // check if it is new shortest path length
        if (nodeIdx == -1 || pathLength < shortestPath) {
            // check if path is valid
            if (checkNoCollision(nearbyNodes[i], xNew, distances[i])) {
                shortestPath = pathLength;
                nodeIdx = i;
            }
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
    for (int i = 0; i < nearbyNodes.size(); ++i) {
        // skip checking xNew's parent
        /// (by right, can skip all of the nodes on the path from root to xNew)
        if (i == nodeIdx) {  // xNew's parent node
            continue;
        }

        // compute length of path from root to node to be checked (i.e. nearby node)
        float pathLength = computePathLength(nearbyNodes[i]);
        if (shortestPath + distances[i] + 1.f < pathLength) {  // a shorter path is found
            // rewire only if it does not pass through obstacles
            tryRewireNodes(nearbyNodes[i], xNew, distances[i]);
        }
    }
}


void RrtStarPlanner::tryRewireNodes(Rrt* nearbyNode, Rrt* xNew, float distance) {

    // check if can rewire nearby node
    if (checkNoCollision(nearbyNode, xNew, distance)) {  // if no obstacles
        // TODO: remove nearby node from parent, but currently buggy
        nearbyNode->getParent()->removeChild(nearbyNode);
        // rewire by setting new parent
        nearbyNode->setParent(xNew);
        xNew->addChild(nearbyNode);
        // publish changed node
        publishRrtNode(nearbyNode);
    }
}


bool RrtStarPlanner::checkNoCollision(Rrt* node, Rrt* other, float distance) {

    Coord pt1(node->getCoord()->_x, node->getCoord()->_y), pt2;
    // get vector
    float vecX = other->getCoord()->_x - pt1._x;
    float vecY = other->getCoord()->_y - pt1._y;
    // find unit vector
    float unitVecX = vecX / distance;
    float unitVecY = vecY / distance;

    for (float dist = 0; dist < distance; dist += 0.5f*_incrementalStep) {
        // check there is an obstacle at every incremental step
        pt2._x = node->getCoord()->_x + unitVecX * dist;
        pt2._y = node->getCoord()->_y + unitVecY * dist;

        if (!noCollision(pt1, pt2)) {  // there was collision
            return false;
        }

        // advance to next point
        pt1._x = pt2._x;
        pt1._y = pt2._y;
    }
    return true;
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
