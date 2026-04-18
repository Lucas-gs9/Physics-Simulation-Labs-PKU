#pragma once
#include<vector>
#include <algorithm>
#include <glm/glm.hpp>
#include "FluidData.h"

namespace VCX::Labs::Fluid {
    template <typename func>
    float triInterpolate(const std::vector<float> & field, glm::vec3 gCoord, func idx);
}