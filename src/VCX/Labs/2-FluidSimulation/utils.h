#pragma once
#include<vector>
#include <algorithm>
#include <glm/glm.hpp>

namespace VCX::Labs::Fluid {
    template <typename func>
    float triInterpolate(const std::vector<float> & field, int nI, int nJ, int nK, glm::vec3 localPos, func idx);
}