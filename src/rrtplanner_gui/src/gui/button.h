#ifndef BUTTON_H_
#define BUTTON_H_

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#include "visualizer_window.h"

using namespace cv;

enum AnchorType {
    BottomLeft,
    // TBD: implement other possible anchor positions: Top/Bottom, Left/Right/Center combinations
};

struct Anchor {

    AnchorType type;

    // position or relative distance from anchor (ignore if irrelevant to the selected anchor type)
    int posX;
    int posY;

    // Constructor
    Anchor(AnchorType type, int posX, int posY) : type(type), posX(posX), posY(posY) {}
};

class Button {

    // Constructor
    Button(Anchor anchor, int width, int height, int r, int g, int b, std::string text, void* callback);

    // bounding box (in pixel coordinates)
    Point _min;
    Point _max;
    // color
    Scalar _color;
    std::string _text;

    // Button position data
    Anchor _anchor;
    int _width;   // button width
    int _height;  // button height

    // Button click callback function
    void* _callback;

    void computeButtonPosition(int windowWidth, int windowHeight);
    void setMinPoint(int minX, int minY);
    void setMaxPoint(int maxX, int maxY);

    // list of Buttons
    static std::vector<Button> buttons;

public:
    // Create button
    static void createButton(Anchor anchor, int width, int height, int r, int g, int b, std::string text, void* callback);
    // Position buttons
    static void drawButtons(int windowWidth, int windowHeight, VisualizerWindow* window);
    // Check for Button click
    static void processButtonClick(int mouseX, int mouseY);

    // Button click functions
    void click();  // Invoke click
    bool mouseOver(int mouseX, int mouseY);  // Check if mouse is over Button
};

#endif  // BUTTON_H_
