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

        std::vector<glm::mat3> c_mat;

        float radius;

        void resize(int n);
        void clear();
        int  size() const;
    };

    struct SpatialHash;

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
            int I = i + 1, J = j + 1, K = k + 1;
            return (I * (ny + 2) * (nz + 2)) + (J * (nz + 2)) + K;
        }
        inline int vIdx(int i, int j, int k) const {
            int I = i + 1, J = j + 1, K = k + 1;
            return (I * (ny + 3) * (nz + 2)) + (J * (nz + 2)) + K;
        }
        inline int wIdx(int i, int j, int k) const {
            int I = i + 1, J = j + 1, K = k + 1;
            return (I * (ny + 2) * (nz + 3)) + (J * (nz + 3)) + K;
        }
        inline int cIdx(int i, int j, int k) const {
            int I = i + 1, J = j + 1, K = k + 1;
            return (I * (ny + 2) * (nz + 2)) + (J * (nz + 2)) + K;
        }
        inline int gridIdx(int i, int j, int k) const {
            return (i * ny * nz) + (j * nz) + k;
        }

        glm::vec3 worldToGrid(const glm::vec3 & worldPos) const;
        glm::ivec3 getCellCoord(const glm::vec3 & worldPos) const;
        int       getCellIdx(const glm::vec3 & worldPos) const;
        glm::vec3  sampleVelocity(const glm::vec3 & worldPos, bool isPrev = false) const;
        void       fillFromParticles(FieldType type, const Particles & particles, const SpatialHash & hash, bool useC = false);
        glm::vec3  sampleAffine(FieldType type, const glm::vec3 & pos) const;

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