#include "JsonUtils.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <vector>

namespace JsonUtils {

    static bool parseHexByte(const std::string& s, size_t pos, unsigned char& out) {
        if (pos + 2 > s.size()) return false;
        auto hex = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
            if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
            return -1;
        };
        int a = hex(s[pos]);
        int b = hex(s[pos + 1]);
        if (a < 0 || b < 0) return false;
        out = (unsigned char)((a << 4) | b);
        return true;
    }

    bool findBool(const std::string& json, const char* key, bool& out) {
        std::string k = std::string("\"") + key + "\"";
        size_t p = json.find(k);
        if (p == std::string::npos) return false;
        p = json.find(':', p + k.size());
        if (p == std::string::npos) return false;
        ++p;
        while (p < json.size() && std::isspace((unsigned char)json[p])) ++p;
        if (p + 3 < json.size() && json.compare(p, 4, "true") == 0) { out = true; return true; }
        if (p + 4 < json.size() && json.compare(p, 5, "false") == 0) { out = false; return true; }
        return false;
    }

    bool findColor(const std::string& json, const char* key, Color& out) {
        std::string k = std::string("\"") + key + "\"";
        size_t p = json.find(k);
        if (p == std::string::npos) return false;
        p = json.find(':', p + k.size());
        if (p == std::string::npos) return false;
        ++p;
        while (p < json.size() && std::isspace((unsigned char)json[p])) ++p;
        if (p >= json.size()) return false;

        if (json[p] == '"') {
            ++p;
            size_t e = p;
            while (e < json.size() && json[e] != '"') ++e;
            if (e >= json.size()) return false;
            std::string s = json.substr(p, e - p);
            if (s.size() == 7 && s[0] == '#') {
                unsigned char r, g, b;
                if (!parseHexByte(s, 1, r) || !parseHexByte(s, 3, g) || !parseHexByte(s, 5, b)) return false;
                out = Color{r, g, b, 255};
                return true;
            }
            return false;
        }

        if (json[p] != '[') return false;
        ++p;
        float c[3] = {0.0f, 0.0f, 0.0f};
        for (int i = 0; i < 3; ++i) {
            while (p < json.size() && std::isspace((unsigned char)json[p])) ++p;
            size_t e = p;
            while (e < json.size()) {
                char ch = json[e];
                if (!(std::isdigit((unsigned char)ch) || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E')) break;
                ++e;
            }
            if (e == p) return false;
            try {
                c[i] = std::stof(json.substr(p, e - p));
            } catch (...) {
                return false;
            }
            p = e;
            while (p < json.size() && std::isspace((unsigned char)json[p])) ++p;
            if (i < 2) {
                if (p >= json.size() || json[p] != ',') return false;
                ++p;
            }
        }

        while (p < json.size() && json[p] != ']') ++p;
        if (p >= json.size()) return false;

        bool is255 = (c[0] > 1.0f || c[1] > 1.0f || c[2] > 1.0f);
        auto toByte = [&](float v) -> unsigned char {
            if (!std::isfinite(v)) v = 0.0f;
            if (is255) v = std::clamp(v, 0.0f, 255.0f);
            else v = std::clamp(v, 0.0f, 1.0f) * 255.0f;
            return (unsigned char)std::lround(v);
        };
        out = Color{toByte(c[0]), toByte(c[1]), toByte(c[2]), 255};
        return true;
    }

    bool findString(const std::string& json, const char* key, std::string& out) {
        std::string k = std::string("\"") + key + "\"";
        size_t p = json.find(k);
        if (p == std::string::npos) return false;
        p = json.find(':', p + k.size());
        if (p == std::string::npos) return false;
        ++p;
        while (p < json.size() && std::isspace((unsigned char)json[p])) ++p;
        if (p >= json.size() || json[p] != '"') return false;
        ++p;
        size_t e = p;
        while (e < json.size() && json[e] != '"') ++e;
        if (e >= json.size()) return false;
        out = json.substr(p, e - p);
        return true;
    }

    bool findNumber(const std::string& json, const char* key, float& out) {
        std::string k = std::string("\"") + key + "\"";
        size_t p = json.find(k);
        if (p == std::string::npos) return false;
        p = json.find(':', p + k.size());
        if (p == std::string::npos) return false;
        ++p;
        while (p < json.size() && std::isspace((unsigned char)json[p])) ++p;
        size_t e = p;
        while (e < json.size()) {
            char c = json[e];
            if (!(std::isdigit((unsigned char)c) || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E')) break;
            ++e;
        }
        if (e == p) return false;
        try {
            out = std::stof(json.substr(p, e - p));
            return true;
        } catch (...) {
            return false;
        }
    }
}
