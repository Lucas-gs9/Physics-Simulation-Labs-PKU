#pragma once
#include<vector>
#include <algorithm>
#include <glm/glm.hpp>
#include "FluidData.h"

namespace VCX::Labs::Fluid {
    template<typename func>
    inline float triInterpolate(const std::vector<float> & field, glm::vec3 gCoord, func idx) {
        int x0 = static_cast<int>(std::floor(gCoord.x));
        int y0 = static_cast<int>(std::floor(gCoord.y));
        int z0 = static_cast<int>(std::floor(gCoord.z));

        float tx = gCoord.x - static_cast<float>(x0);
        float ty = gCoord.y - static_cast<float>(y0);
        float tz = gCoord.z - static_cast<float>(z0);

        if (field.empty()) return 0.0f;

        float v000 = field[idx(x0, y0, z0)];
        float v100 = field[idx(x0 + 1, y0, z0)];
        float v010 = field[idx(x0, y0 + 1, z0)];
        float v110 = field[idx(x0 + 1, y0 + 1, z0)];
        float v001 = field[idx(x0, y0, z0 + 1)];
        float v101 = field[idx(x0 + 1, y0, z0 + 1)];
        float v011 = field[idx(x0, y0 + 1, z0 + 1)];
        float v111 = field[idx(x0 + 1, y0 + 1, z0 + 1)];

        return glm::mix(
            glm::mix(glm::mix(v000, v100, tx), glm::mix(v010, v110, tx), ty),
            glm::mix(glm::mix(v001, v101, tx), glm::mix(v011, v111, tx), ty),
            tz);
    }

    float quadraticKernel(float x);

    template<typename func>
    inline float quadInterpolate(const std::vector<float> & field, glm::vec3 gCoord, func idx) {
        if (field.empty()) return 0.0f;

        int i = static_cast<int>(std::round(gCoord.x));
        int j = static_cast<int>(std::round(gCoord.y));
        int k = static_cast<int>(std::round(gCoord.z));

        float sum = 0.0f;
        for (int di = -1; di <= 1; ++di) {
            for (int dj = -1; dj <= 1; ++dj) {
                for (int dk = -1; dk <= 1; ++dk) {
                    int ni = i + di;
                    int nj = j + dj;
                    int nk = k + dk;

                    float wx = quadraticKernel(static_cast<float>(ni) - gCoord.x);
                    float wy = quadraticKernel(static_cast<float>(nj) - gCoord.y);
                    float wz = quadraticKernel(static_cast<float>(nk) - gCoord.z);

                    sum += wx * wy * wz * field[idx(ni, nj, nk)];
                }
            }
        }
        return sum;
    }

    inline float poly6(float rSq, float h) {
        float hSq = h * h;
        if (rSq < 0 || rSq > hSq) return 0.0f;
        float term = hSq - rSq;
        return (315.0f / (64.0f * 3.14 * pow(h, 9))) * term * term * term;
    }

    inline glm::vec3 gradSpiky(glm::vec3 r, float h) {
        float rLen = glm::length(r);
        if (rLen <= 0.0f || rLen >= h) return glm::vec3(0.0f);
        float factor = -45.0f / (3.14 * pow(h, 6));
        float term   = h - rLen;
        return (factor * term * term) * (r / rLen);
    }
}