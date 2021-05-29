// RRT Tree Node

#ifndef RRT_H_
#define RRT_H_

#include "coord.h"

class Rrt {
    Coord _coord;  // grid coordinate
    Rrt* _parent;  // parent node

public:
    Rrt(int x, int y, Rrt* parent) : _coord(Coord(x, y)), _parent(parent) {}
    Rrt(Coord coord, Rrt* parent) : _coord(coord), _parent(parent) {}
};


#endif  // RRT_H_
