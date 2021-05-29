// RRT Tree Node

#ifndef RRT_H_
#define RRT_H_

#include <vector>
#include "coord.h"

class Rrt {
    int _id;  // node id - easier for visualization
    Coord _coord;  // grid coordinate
    Rrt* _parent;  // parent node for easier backtracking
    std::vector<Rrt*> _children;  // children nodes

public:
    Rrt(float x, float y, Rrt* parent) : _id(-1), _coord(Coord(x, y)), _parent(parent), _children() {}
    Rrt(const Coord& coord, Rrt* parent) : _id(-1), _coord(coord), _parent(parent), _children() {}

    ~Rrt() {
        while (!_children.empty()) {
            Rrt* child = _children.back();
            _children.pop_back();
            delete child;
        }
    }

    // id
    int getId() {
        return _id;
    }
    void setId(int id) {
        _id = id;
    }
    // coord
    Coord* getCoord() {
        return &_coord;
    }
    // parent
    Rrt* getParent() {
        return _parent;
    }
    // children
    int getNumChildren() {
        return _children.size();
    }
    Rrt* getChild(int idx) {
        return _children.at(idx);
    }
    void addChild(Rrt* child) {
        _children.push_back(child);
    }

    // helper functions
    bool equalsState(const Coord& state) {
        return _coord == state;
    }
};


#endif  // RRT_H_
