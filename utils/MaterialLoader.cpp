#include "MaterialLoader.h"
#include "JsonUtils.h"
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <cmath>

namespace MaterialLoader {

    static std::string readFileText(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) return {};
        std::string s;
        f.seekg(0, std::ios::end);
        s.resize((size_t)f.tellg());
        f.seekg(0, std::ios::beg);
        f.read(s.data(), (std::streamsize)s.size());
        return s;
    }

    std::vector<MaterialProps> loadMaterialsFromFolder(const char* folder) {
        std::vector<MaterialProps> out;
        namespace fs = std::filesystem;
        std::error_code ec;
        if (!fs::exists(folder, ec) || !fs::is_directory(folder, ec)) return out;

        for (const auto& entry : fs::directory_iterator(folder, ec)) {
            if (ec) break;
            if (!entry.is_regular_file()) continue;
            if (entry.path().extension() != ".json") continue;

            std::string text = readFileText(entry.path().string());
            if (text.empty()) continue;

            MaterialProps m;
            if (!JsonUtils::findString(text, "name", m.name)) {
                m.name = entry.path().stem().string();
            }
            JsonUtils::findNumber(text, "density", m.density);
            JsonUtils::findNumber(text, "friction", m.friction);
            JsonUtils::findNumber(text, "restitution", m.restitution);
            JsonUtils::findNumber(text, "opacity", m.opacity);
            JsonUtils::findBool(text, "outline", m.outline);
            JsonUtils::findColor(text, "color", m.color);

            if (m.density <= 0.0f) m.density = 1.0f;
            m.friction = std::clamp(m.friction, 0.0f, 2.0f);
            m.restitution = std::clamp(m.restitution, 0.0f, 1.0f);
            m.opacity = std::clamp(m.opacity, 0.0f, 1.0f);
            out.push_back(m);
        }

        std::sort(out.begin(), out.end(), [](const MaterialProps& a, const MaterialProps& b) { return a.name < b.name; });
        return out;
    }
}
