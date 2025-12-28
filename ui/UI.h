#pragma once
#include <raylib.h>
#include <vector>
#include <string>

namespace UI {
    void setClipRect(Rectangle r);
    Rectangle getClipRect();
    
    bool isBlocked();

    bool Button(Rectangle r, const char* text);
    bool Radio(Rectangle r, const char* text, bool selected);
    float Slider(int id, Rectangle r, float value, float minV, float maxV, bool& changed);
    void Dropdown(int id, Rectangle r, const std::vector<std::string>& items, int* selectedIndex);

    int DrawDropdownOverlay();
}
