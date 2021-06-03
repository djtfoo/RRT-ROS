# RRT-ROS
Describe project here, ROS Melodic, Ubuntu 18.04 LTS.

## Table of Contents
- [RRT-ROS](#rrt-ros)
- [Table of Contents](#table-of-contents)
- [How to Run](#how-to-run)
  - [Setup nodes](#setup-nodes)
  - [Start RRT algorithm](#start-rrt-algorithm)
- [Screenshots](#screenshots)
- [References](#references)

## Prerequisites
Supported Platform, necessary libraries/packages/tools
- Ubuntu 18.04 LTS
- ROS Melodic ([Required Packages](https://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023))
  - OpenCV 3.2.0

## Installation Guide
Steps to clone repo and build
```cmd
> git clone https://github.com/djtfoo/rrt-ros
> cd rrt-ros
```

Use `catkin build` to build the repository.
```cmd
> catkin build
```
If the following error shows, run catkin build again.
<br> insert here

If the build is successful, you should see the following:
<br> insert here

## How to Run
### Setup nodes
In each new terminal, source for the local setup file:
```cmd
> cd rrt-ros
> source devel/setup.bash
```

First, run planner_node:

```
rosrun rrt_planner planner_node
```

Optionally, run visualizer_node in a new terminal to show a GUI to configure the navigation parameters and visualize the tree building on the map:

```
rosrun rrtplanner_map visualizer_node
```

Run mapserver_node in a new terminal to generate the occupancy grid from an image of a map, and publish it for the planner and visualizer:

```
rosrun rrtplanner_map mapserver_node <path_to_map_image> <gridsize>
```
- path_to_map_image: the file path (relative or absolute) to the map image file. The image should be PNG/JPG/PGM.
- gridsize: the size in pixels of the occupancy grid.


### Start RRT algorithm
If using visualizer_node, <instructions and screenshots ...>

If not using visualizer_node, run pathrequest_node in an open terminal to provide the start and goal positions to start the RRT search:

```
rosrun rrt_planner pathrequest_node <start_x> <start_y> <goal_x> <goal_y> <rrt_ver>
```
- start_[xy]: the pixel coordinates of the start position
- goal_[xy]: the pixel coordinates of the goal position
- rrt_ver: 0 for basic RRT, 1 for RRT*

If a path was found, <output ...>

## Screenshots
Throw in some screenshots

## References
- [Steven M. Lavalle’s RRT web page](http://lavalle.pl/rrt/)
- [Original Lavalle et al. RRT paper](http://msl.cs.uiuc.edu/~lavalle/papers/LavKuf01.pdf)
- [Wikipedia page on RRT](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree)
- [RRT* Brief Explanation (YouTube)](https://www.youtube.com/watch?v=JM7kmWE8Gtc)
