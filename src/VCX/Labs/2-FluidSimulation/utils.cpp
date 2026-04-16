#include "utils.h"

namespace VCX::Labs::Fluid {
    template <typename func>
    float triInterpolate(const std::vector<float>& field, int nx, int ny, int nz, glm::vec3 localPos, func idx) {
        float x = localPos.x;
        float y = localPos.y;
        float z = localPos.z;

        int x0 = std::clamp(int(x), 0, nx - 2);
        int y0 = std::clamp(int(y), 0, ny - 2);
        int z0 = std::clamp(int(z), 0, nz - 2);
        float tx = x - x0;
        float ty = y - y0;
        float tz = z - z0;

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
}