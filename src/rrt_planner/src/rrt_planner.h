#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_

#include "planner.h"

class RrtPlanner : public Planner
{
public:
    RrtPlanner(ros::NodeHandle& nh);

protected:
    // plan path
    virtual void planPath();

    // RRT stuff
    //probably helper funcs to do stuff like compute Voronoi region, get config, collision detection, etc
};

#endif	// RRT_PLANNER_H_
