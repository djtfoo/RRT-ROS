// RRT Tree Node

#ifndef RRT_H_
#define RRT_H_

#include <vector>
#include "coord.h"

class Rrt {
    Coord _coord;  // grid coordinate
    Rrt* _parent;  // parent node for easier backtracking
    std::vector<Rrt*> _children;  // children nodes

public:
    Rrt(int x, int y, Rrt* parent) : _coord(Coord(x, y)), _parent(parent), _children() {}
    Rrt(Coord coord, Rrt* parent) : _coord(coord), _parent(parent), _children() {}

    int getNumChildren() {
        return _children.size();
    }
    Rrt* getChild(int idx) {
        return _children.at(idx);
    }
    void addChild(Rrt* child) {
        _children.push_back(child);
    }
};


#endif  // RRT_H_
