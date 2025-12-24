#pragma once
#include <vector>
#include <string>
#include <raylib.h>
#include <algorithm>

struct UiDropdownState {
    bool isOpen = false;
    int selectedIndex = 0;
    std::vector<std::string> items;
    Rectangle rect;
};

static int g_activeDropdownId = -1;
static Rectangle g_activeDropdownRect = {0,0,0,0};
static std::vector<std::string> g_activeDropdownItems;
static int* g_activeDropdownSelection = nullptr;

static bool uiIsBlocked() {
    if (g_activeDropdownId != -1) {
        Vector2 m = GetMousePosition();
        float h = (float)g_activeDropdownItems.size() * 24.0f;
        Rectangle listRect = {g_activeDropdownRect.x, g_activeDropdownRect.y + g_activeDropdownRect.height, g_activeDropdownRect.width, h};
        if (CheckCollisionPointRec(m, listRect)) return true;
    }
    return false;
}

static bool uiDropdown(int id, Rectangle r, const std::vector<std::string>& items, int& selectedIndex) {
    if (uiIsBlocked() && g_activeDropdownId != id) return false;

    Vector2 m = GetMousePosition();
    bool hot = CheckCollisionPointRec(m, r);
    
    bool isOpen = (g_activeDropdownId == id);
    
    Color bg = hot ? Color{45, 45, 45, 255} : Color{25, 25, 25, 255};
    DrawRectangleRec(r, bg);
    DrawRectangleLinesEx(r, 1.0f, Color{70, 70, 70, 255});
    
    const char* text = (selectedIndex >= 0 && selectedIndex < (int)items.size()) ? items[selectedIndex].c_str() : "";
    DrawText(text, (int)(r.x + 8), (int)(r.y + 5), 18, RAYWHITE);
    
    DrawText("v", (int)(r.x + r.width - 20), (int)(r.y + 5), 14, RAYWHITE);

    if (hot && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        if (isOpen) g_activeDropdownId = -1;
        else g_activeDropdownId = id;
        isOpen = !isOpen;
    }
    
    return false;
}

static bool uiDrawDropdownOverlay(int& activeId, Rectangle r, const std::vector<std::string>& items, int& selectedIndex) {
    if (activeId == -1) return false;
    
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 m = GetMousePosition();
        float h = (float)items.size() * 24.0f;
        Rectangle listRect = {r.x, r.y + r.height, r.width, h};
        if (!CheckCollisionPointRec(m, listRect) && !CheckCollisionPointRec(m, r)) {
            activeId = -1;
            return false;
        }
    }

    float itemHeight = 24.0f;
    float listHeight = (float)items.size() * itemHeight;
    Rectangle listRect = {r.x, r.y + r.height, r.width, listHeight};
    
    DrawRectangleRec(listRect, Color{35, 35, 35, 255});
    DrawRectangleLinesEx(listRect, 1.0f, Color{80, 80, 80, 255});
    
    Vector2 m = GetMousePosition();
    bool changed = false;

    for (int i = 0; i < (int)items.size(); ++i) {
        Rectangle itemRect = {listRect.x, listRect.y + i * itemHeight, listRect.width, itemHeight};
        bool itemHot = CheckCollisionPointRec(m, itemRect);
        
        if (itemHot) {
            DrawRectangleRec(itemRect, Color{60, 60, 60, 255});
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                selectedIndex = i;
                activeId = -1;
                changed = true;
            }
        }
        
        Color textColor = (i == selectedIndex) ? YELLOW : RAYWHITE;
        DrawText(items[i].c_str(), (int)(itemRect.x + 8), (int)(itemRect.y + 4), 18, textColor);
    }
    
    return changed;
}
