#ifndef MAPPARSER_H_
#define MAPPARSER_H_

#include <iostream>
#include <ros/ros.h>

#include <opencv2/imgproc.hpp>

using namespace cv;

class MapParser {
public:
    static void parseMapData(const Mat& img, unsigned int width, unsigned int height, unsigned int gridSize, std::vector<unsigned char>& occupanyData);
};

#endif  // MAPPARSER_H_
