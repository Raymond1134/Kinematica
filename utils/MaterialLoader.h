#pragma once
#include <vector>
#include "../physics/Material.h"

namespace MaterialLoader {
    std::vector<MaterialProps> loadMaterialsFromFolder(const char* folder);
}
