#include "UI.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace UI {

    static int g_activeDropdownId = -1;
    static Rectangle g_activeDropdownRect = {0,0,0,0};
    static std::vector<std::string> g_activeDropdownItems;
    static int* g_activeDropdownSelection = nullptr;
    static Rectangle g_uiClipRect = {0,0,0,0};

    void setClipRect(Rectangle r) {
        g_uiClipRect = r;
    }

    Rectangle getClipRect() {
        return g_uiClipRect;
    }

    bool isBlocked() {
        if (g_activeDropdownId != -1) {
            Vector2 m = GetMousePosition();
            float h = (float)g_activeDropdownItems.size() * 24.0f;
            Rectangle listRect = {g_activeDropdownRect.x, g_activeDropdownRect.y + g_activeDropdownRect.height, g_activeDropdownRect.width, h};
            if (CheckCollisionPointRec(m, listRect)) return true;
        }
        return false;
    }

    bool Button(Rectangle r, const char* text) {
        if (isBlocked()) return false;
        Vector2 m = GetMousePosition();
        bool hot = CheckCollisionPointRec(m, r);
        if (g_uiClipRect.width > 0 && !CheckCollisionPointRec(m, g_uiClipRect)) hot = false;

        Color bg = hot ? Color{45, 45, 45, 255} : Color{25, 25, 25, 255};
        DrawRectangleRec(r, bg);
        DrawRectangleLinesEx(r, 1.0f, Color{70, 70, 70, 255});

        int tw = MeasureText(text, 18);
        int tx = (int)(r.x + 0.5f * (r.width - (float)tw));
        int ty = (int)(r.y + 0.5f * (r.height - 18.0f));
        DrawText(text, tx, ty, 18, RAYWHITE);
        return hot && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    }

    bool Radio(Rectangle r, const char* text, bool selected) {
        if (isBlocked()) return false;
        Vector2 m = GetMousePosition();
        bool hot = CheckCollisionPointRec(m, r);
        if (g_uiClipRect.width > 0 && !CheckCollisionPointRec(m, g_uiClipRect)) hot = false;

        Color bg = hot ? Color{45, 45, 45, 255} : Color{25, 25, 25, 255};
        DrawRectangleRec(r, bg);
        DrawRectangleLinesEx(r, 1.0f, Color{70, 70, 70, 255});

        Rectangle c = {r.x + 6, r.y + 7, 14, 14};
        DrawRectangleLinesEx(c, 1.0f, RAYWHITE);
        if (selected) {
            DrawRectangle((int)c.x + 3, (int)c.y + 3, (int)c.width - 6, (int)c.height - 6, RAYWHITE);
        }
        DrawText(text, (int)(r.x + 26), (int)(r.y + 5), 18, RAYWHITE);

        return hot && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    }

    float Slider(int id, Rectangle r, float value, float minV, float maxV, bool& changed) {
        if (isBlocked()) return value;
        Vector2 m = GetMousePosition();
        bool hot = CheckCollisionPointRec(m, r);
        if (g_uiClipRect.width > 0 && !CheckCollisionPointRec(m, g_uiClipRect)) hot = false;

        static bool dragging = false;
        static int dragId = -1;

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && hot) {
            dragging = true;
            dragId = id;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && dragging && dragId == id) {
            dragging = false;
            dragId = -1;
        }

        float t = (maxV > minV) ? ((value - minV) / (maxV - minV)) : 0.0f;
        t = std::clamp(t, 0.0f, 1.0f);

        DrawRectangleRec(r, Color{25, 25, 25, 255});
        DrawRectangleLinesEx(r, 1.0f, Color{70, 70, 70, 255});
        Rectangle fill = {r.x, r.y, r.width * t, r.height};
        DrawRectangleRec(fill, Color{60, 60, 60, 255});

        float knobX = r.x + r.width * t;
        DrawRectangle((int)(knobX - 3), (int)r.y - 2, 6, (int)r.height + 4, RAYWHITE);

        if (dragging && dragId == id) {
            float nt = (m.x - r.x) / r.width;
            nt = std::clamp(nt, 0.0f, 1.0f);
            float nv = minV + (maxV - minV) * nt;
            if (fabsf(nv - value) > 1e-6f) {
                value = nv;
                changed = true;
            }
        }
        return value;
    }

    void Dropdown(int id, Rectangle r, const std::vector<std::string>& items, int* selectedIndex) {
        if (isBlocked() && g_activeDropdownId != id) return;

        Vector2 m = GetMousePosition();
        bool hot = CheckCollisionPointRec(m, r);
        if (g_uiClipRect.width > 0 && !CheckCollisionPointRec(m, g_uiClipRect)) hot = false;
        
        bool isOpen = (g_activeDropdownId == id);
        
        Color bg = hot ? Color{45, 45, 45, 255} : Color{25, 25, 25, 255};
        DrawRectangleRec(r, bg);
        DrawRectangleLinesEx(r, 1.0f, Color{70, 70, 70, 255});
        
        int idx = selectedIndex ? *selectedIndex : -1;
        const char* text = (idx >= 0 && idx < (int)items.size()) ? items[idx].c_str() : "";
        DrawText(text, (int)(r.x + 8), (int)(r.y + 5), 18, RAYWHITE);
        
        DrawText("v", (int)(r.x + r.width - 20), (int)(r.y + 5), 14, RAYWHITE);

        if (hot && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (isOpen) {
                g_activeDropdownId = -1;
            } else {
                g_activeDropdownId = id;
                g_activeDropdownRect = r;
                g_activeDropdownItems = items;
                g_activeDropdownSelection = selectedIndex;
            }
        }
    }

    int DrawDropdownOverlay() {
        if (g_activeDropdownId == -1) return -1;
        
        Rectangle r = g_activeDropdownRect;
        const auto& items = g_activeDropdownItems;

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Vector2 m = GetMousePosition();
            float h = (float)items.size() * 24.0f;
            Rectangle listRect = {r.x, r.y + r.height, r.width, h};
            if (!CheckCollisionPointRec(m, listRect) && !CheckCollisionPointRec(m, r)) {
                g_activeDropdownId = -1;
                return -1;
            }
        }

        float itemHeight = 24.0f;
        float listHeight = (float)items.size() * itemHeight;
        Rectangle listRect = {r.x, r.y + r.height, r.width, listHeight};
        
        DrawRectangleRec(listRect, Color{35, 35, 35, 255});
        DrawRectangleLinesEx(listRect, 1.0f, Color{80, 80, 80, 255});
        
        Vector2 m = GetMousePosition();
        int changedId = -1;

        int currentSel = g_activeDropdownSelection ? *g_activeDropdownSelection : -1;

        for (int i = 0; i < (int)items.size(); ++i) {
            Rectangle itemRect = {listRect.x, listRect.y + i * itemHeight, listRect.width, itemHeight};
            bool itemHot = CheckCollisionPointRec(m, itemRect);
            
            if (itemHot) {
                DrawRectangleRec(itemRect, Color{60, 60, 60, 255});
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                    if (g_activeDropdownSelection) *g_activeDropdownSelection = i;
                    changedId = g_activeDropdownId;
                    g_activeDropdownId = -1;
                }
            }
            
            Color textColor = (i == currentSel) ? YELLOW : RAYWHITE;
            DrawText(items[i].c_str(), (int)(itemRect.x + 8), (int)(itemRect.y + 4), 18, textColor);
        }
        
        return changedId;
    }
}
