#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <algorithm>
namespace VCX::Labs::Fluid {
    enum class CellType : int {
        Empty = 0,
        Fluid = 1,
        Solid = 2
    };

    enum class FieldType { U,
                           V,
                           W };

    struct Particles {
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> velocities;
        std::vector<glm::vec3> colors;

        float radius;

        void resize(int n);
        void clear();
        int  size() const;
    };

    struct Grid {
        int nx, ny, nz;
        float h;
        float inv_h;

        std::vector<float>     u, v, w;
        std::vector<float> u_prev, v_prev, w_prev;

        std::vector<float>     pressure;
        std::vector<float>     s_field;
        std::vector<CellType>  type;
        std::vector<float>     density;

        inline int uIdx(int i, int j, int k) const {
            i = std::clamp(i, 0, nx);
            j = std::clamp(j, 0, ny - 1);
            k = std::clamp(k, 0, nz - 1);
            return (i * ny * nz) + (j * nz) + k;
        }
        inline int vIdx(int i, int j, int k) const {
            i = std::clamp(i, 0, nx - 1);
            j = std::clamp(j, 0, ny);
            k = std::clamp(k, 0, nz - 1);
            return (i * (ny + 1) * nz) + (j * nz) + k;
        }
        inline int wIdx(int i, int j, int k) const {
            i = std::clamp(i, 0, nx - 1);
            j = std::clamp(j, 0, ny - 1);
            k = std::clamp(k, 0, nz);
            return (i * ny * (nz + 1)) + (j * (nz + 1)) + k;
        }
        inline int cIdx(int i, int j, int k) const {
            i = std::clamp(i, 0, nx - 1);
            j = std::clamp(j, 0, ny - 1);
            k = std::clamp(k, 0, nz - 1);
            return (i * ny * nz) + (j * nz) + k;
        }

        glm::vec3 worldToGrid(const glm::vec3 & worldPos) const;
        glm::ivec3 getCellCoord(const glm::vec3 & worldPos) const;
        int       getCellIdx(const glm::vec3 & worldPos) const;
        glm::vec3  sampleVelocity(const glm::vec3 & worldPos, bool isPrev = false) const;
        void fillFromParticles(FieldType type, const Particles & particles, const SpatialHash & hash);

        int  size() const;
        void resetStep();
        void reset(int resX, int resY, int resZ, float spacing);
    };

    struct SpatialHash {
        std::vector<int> particleList;
        std::vector<int> cellStart;

        void build(const Grid & grid, const Particles & particles);
    };

    struct FluidData {
        Particles particles;
        Grid      grid;
        SpatialHash hash;
    };
}