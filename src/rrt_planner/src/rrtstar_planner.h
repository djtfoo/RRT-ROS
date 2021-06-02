#ifndef RRTSTAR_PLANNER_H_
#define RRTSTAR_PLANNER_H_

#include <vector>

#include "rrt_planner.h"

class RrtStarPlanner : public RrtPlanner {
public:
    // Constructor
    RrtStarPlanner(ros::NodeHandle& nh);

protected:
    virtual void addToRrt(Rrt* rrt, Rrt* xNear, Rrt* xNew);

private:
    void addAndOptimiseNodes(Rrt* rrt, Rrt* xNew);
    void getNearbyNodes(Rrt* rrt, Rrt* xNew, float radius, std::vector<Rrt*>& nearbyNodes);
    float computePathLength(Rrt* node);
};

#endif  // RRTSTAR_PLANNER_H_
