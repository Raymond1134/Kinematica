#pragma once
#include <string>
#include <raylib.h>

namespace JsonUtils {
    bool findBool(const std::string& json, const char* key, bool& out);
    bool findColor(const std::string& json, const char* key, Color& out);
    bool findString(const std::string& json, const char* key, std::string& out);
    bool findNumber(const std::string& json, const char* key, float& out);
}
