// RRT Tree Node

#ifndef RRT_H_
#define RRT_H_

#include <math.h>
#include <vector>
#include "coord.h"

class Rrt {
    static int currId;

    int _id;  // node id - easier for visualization
    Coord _coord;  // grid coordinate
    Rrt* _parent;  // parent node for easier backtracking
    std::vector<Rrt*> _children;  // children nodes

public:
    Rrt(float x, float y, Rrt* parent) : _id(currId), _coord(Coord(x, y)), _parent(parent), _children(0) {
    ++currId;
}
    Rrt(const Coord& coord, Rrt* parent) : _id(currId), _coord(Coord(coord._x, coord._y)), _parent(parent), _children(0) {
    ++currId;
}

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
    void setParent(Rrt* newParent) {
        _parent = newParent;
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

    float distanceToNode(Rrt* other) {
        return sqrt(distanceSquaredToNode(other));
    }
    float distanceSquaredToNode(Rrt* other) {
        return distanceSquaredToCoord( *(other->getCoord()) );
    }
    float distanceSquaredToCoord(const Coord& coord) {
        float xLen = _coord._x - coord._x;
        float yLen = _coord._y - coord._y;
        return (xLen*xLen + yLen*yLen);
    }
};

#endif  // RRT_H_
