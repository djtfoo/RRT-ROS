#include "button.h"

std::vector<Button> Button::buttons;

Button::Button(Anchor anchor, int width, int height, int r, int g, int b, std::string text, void* callback)
 : _min(Point()), _max(Point()),
  _anchor(anchor),
  _width(width), _height(height),
  _color(Scalar(r, g, b)),
  _text(text),
  _callback(callback)
{}

 // Invoke click
 void Button::click() {
    ((void(*)())_callback)();
}

void Button::createButton(Anchor anchor, int width, int height, int r, int g, int b, std::string text, void* callback) {
    // add to list of buttons
    buttons.push_back(Button(anchor, width, height, r, g, b, text, callback));
}

void Button::processButtonClick(int mouseX, int mouseY) {
    for (int i = 0; i < buttons.size(); ++i) {
        Button* button = &(buttons[i]);
        // check if mouse position is inside button
        if (button->mouseOver(mouseX, mouseY)) {
            // invoke button click
            button->click();
            break;
        }
    }
}

void Button::computeButtonPosition(int windowWidth, int windowHeight) {
    switch (_anchor.type) {
    case BottomLeft:
        setMaxPoint(_anchor.posX + _width, windowHeight - 1 - _anchor.posY);
        setMinPoint(_anchor.posX, _max.y - _height);
        break;
    }
}

void Button::drawButtons(int windowWidth, int windowHeight, VisualizerWindow* window) {
    for (int i = 0; i < buttons.size(); ++i) {
        buttons[i].computeButtonPosition(windowWidth, windowHeight);
        // display button
        window->drawRectangle(buttons[i]._min, buttons[i]._max, buttons[i]._color, -1);
        float fontSize = 0.6;
        window->putText(buttons[i]._text.c_str(), Point(buttons[i]._min.x, buttons[i]._max.y - 0.5*(buttons[i]._height-10*fontSize)), fontSize, Scalar(255, 255, 255), 1);
    }
}

void Button::setMinPoint(int minX, int minY) {
    _min.x = minX;
    _min.y = minY;
}

void Button::setMaxPoint(int maxX, int maxY) {
    _max.x = maxX;
    _max.y = maxY;
}

bool Button::mouseOver(int mouseX, int mouseY) {
    return (mouseX >= _min.x && mouseX <= _max.x &&
        mouseY >= _min.y && mouseY <= _max.y);
}
