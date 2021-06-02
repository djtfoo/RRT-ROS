#include "mapparser.h"

void MapParser::parseMapData(const Mat& img, unsigned int width, unsigned int height, unsigned int gridSize, std::vector<unsigned char>& occupanyData) {

    ROS_INFO("Parsing map data ...");

    int cn = img.channels();
    std::cout << "No. channels: " << cn << std::endl;
    int threshold = ceil(gridSize*gridSize*0.5);    // threshold no. obstacle pixels to determine if a grid is an obstacle
    // iterate through grids
    for (int row = 0; row < height; row += gridSize) {
        for (int col = 0; col < width; col += gridSize) {
            // compute occupancy for each grid
            int obstacleCount = 0;  // no. obstacle pixels within this grid
            for (int i = row; i < row+gridSize; ++i) {
                for (int j = col; j < col+gridSize; ++j) {
                    // compute color of pixel
                    int b = img.data[i*width*cn + j*cn];
                    int g = img.data[i*width*cn + j*cn + 1];
                    int r = img.data[i*width*cn + j*cn + 2];
                    int color = b + g + r;    // sum of pixel's RGB values
                    // check if pixel is an obstacle
                    if (color/3 > 200)  // obstacle pixel is white
                        ++obstacleCount;
                }
            }
            // check if grid is occupied or not: obstacleCount > threshold, it is occupied
            unsigned char occupied = (obstacleCount >= threshold) ? 1 : 0;
            occupanyData.push_back(occupied);
            //	std::cout << (occupied ? 1 : 0);
        }
        //std::cout << std::endl;
    }
}
