#include "utils.h"

namespace VCX::Labs::Fluid {
    float quadraticKernel(float x) {
        x = std::abs(x);
        if (x < 0.5f) {
            return 0.75f - x * x;
        } else if (x < 1.5f) {
            float v = 1.5f - x;
            return 0.5f * v * v;
        }
        return 0.0f;
    }
}