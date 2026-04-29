#include "FluidData.h"
#include "utils.h"
namespace VCX::Labs::Fluid {
    void Particles::resize(int n) {
        positions.resize(n);
        velocities.resize(n);
        colors.resize(n, glm::vec3(1.f, 1.f, 1.f));
    }

    void Particles::clear() {
        positions.clear();
        velocities.clear();
        colors.clear();
    }

    int Particles::size() const {
        return positions.size();
    }

    glm::vec3 Grid::worldToGrid(const glm::vec3& worldPos) const {
        return worldPos * inv_h;
    }

    glm::ivec3 Grid::getCellCoord(const glm::vec3& worldPos) const {
        glm::vec3 gPos = worldToGrid(worldPos);
        return glm::ivec3(
            std::clamp(int(std::floor(gPos.x)), 0, nx - 1),
            std::clamp(int(std::floor(gPos.y)), 0, ny - 1),
            std::clamp(int(std::floor(gPos.z)), 0, nz - 1));
    }

    int Grid::getCellIdx(const glm::vec3& worldPos) const{
        glm::ivec3 coord = getCellCoord(worldPos);
        return cIdx(coord.x, coord.y, coord.z);
    }

    glm::vec3 Grid::sampleVelocity(const glm::vec3 & worldPos, bool isPrev = false) const {
        glm::vec3 g = worldPos * inv_h;
        float     uVal, vVal, wVal;

        if (isPrev) {
            uVal = triInterpolate(u_prev, { g.x, g.y - 0.5f, g.z - 0.5f }, [this](int i, int j, int k) { return uIdx(i, j, k); });
            vVal = triInterpolate(v_prev, { g.x - 0.5f, g.y, g.z - 0.5f }, [this](int i, int j, int k) { return vIdx(i, j, k); });
            wVal = triInterpolate(w_prev, { g.x - 0.5f, g.y - 0.5f, g.z }, [this](int i, int j, int k) { return wIdx(i, j, k); });
        }
        else {
            uVal = triInterpolate(u, { g.x, g.y - 0.5f, g.z - 0.5f }, [this](int i, int j, int k) { return uIdx(i, j, k); });
            vVal = triInterpolate(v, { g.x - 0.5f, g.y, g.z - 0.5f }, [this](int i, int j, int k) { return vIdx(i, j, k); });
            wVal = triInterpolate(w, { g.x - 0.5f, g.y - 0.5f, g.z }, [this](int i, int j, int k) { return wIdx(i, j, k); });
        }

        return glm::vec3(uVal, vVal, wVal);
    }

    void Grid::fillFromParticles(FieldType type, const Particles& particles, const SpatialHash& hash) {
        std::vector<float>* field;
        glm::ivec3           fSize;
        glm::vec3            offset;

        if (type == FieldType::U) {
            field  = &u;
            fSize  = { nx + 1, ny, nz };
            offset = { 0.0f, 0.5f, 0.5f };
        } else if (type == FieldType::V) {
            field  = &v;
            fSize  = { nx, ny + 1, nz };
            offset = { 0.5f, 0.0f, 0.5f };
        } else {
            field  = &w;
            fSize  = { nx, ny, nz + 1 };
            offset = { 0.5f, 0.5f, 0.0f };
        }

        #pragma omp parallel for collapse(3)
        for (int i = 0; i < fSize.x; ++i) {
            for (int j = 0; j < fSize.y; ++j) {
                for (int k = 0; k < fSize.z; ++k) {
                    glm::vec3  faceGPos = glm::vec3(i, j, k) + offset;
                    float      sumV = 0.0f, sumW = 0.0f;

                    glm::ivec3 minC = glm::floor(faceGPos - 0.5f);
                    glm::ivec3 maxC = glm::floor(faceGPos + 0.5f);

                    for (int ni = minC.x; ni <= maxC.x; ++ni) {
                        for (int nj = minC.y; nj <= maxC.y; ++nj) {
                            for (int nk = minC.z; nk <= maxC.z; ++nk) {
                                if (ni < 0 || ni >= nx || nj < 0 || nj >= ny || nk < 0 || nk >= nz) continue;

                                int cIdx = this->cIdx(ni, nj, nk);
                                for (int p = hash.cellStart[cIdx]; p < hash.cellStart[cIdx + 1]; ++p) {
                                    int       pIdx  = hash.particleList[p];
                                    glm::vec3 pGPos = particles.positions[pIdx] * inv_h;

                                    glm::vec3 d = glm::abs(faceGPos - pGPos);
                                    if (d.x < 1.0f && d.y < 1.0f && d.z < 1.0f) {
                                        float weight = (1.0f - d.x) * (1.0f - d.y) * (1.0f - d.z);

                                        float pVel = 0.0f;
                                        if (type == FieldType::U) pVel = particles.velocities[pIdx].x;
                                        else if (type == FieldType::V) pVel = particles.velocities[pIdx].y;
                                        else pVel = particles.velocities[pIdx].z;

                                        sumV += pVel * weight;
                                        sumW += weight;
                                    }
                                }
                            }
                        }
                    }
                    int index       = i * fSize.y * fSize.z + j * fSize.z + k;
                    (*field)[index] = (sumW > 1e-6f) ? (sumV / sumW) : 0.0f;
                }
            }
        }
    }

    int Grid::size() const{
        return nx * ny * nz;
    }

    void Grid::resetStep() {
        u_prev = u;
        v_prev = v;
        w_prev = w;

        std::fill(u.begin(), u.end(), 0.f);
        std::fill(v.begin(), v.end(), 0.f);
        std::fill(w.begin(), w.end(), 0.f);
        std::fill(density.begin(), density.end(), 0.f);
    }

    void Grid::reset(int resX, int resY, int resZ, float spacing) {
        nx    = resX;
        ny    = resY;
        nz    = resZ;
        h     = spacing;
        inv_h = 1.f / spacing;

        u.assign((nx + 1) * ny * nz, 0.f);
        v.assign(nx * (ny + 1) * nz, 0.f);
        w.assign(nx * ny * (nz + 1), 0.f);
        u_prev.assign(u.size(), 0.f);
        v_prev.assign(v.size(), 0.f);
        w_prev.assign(w.size(), 0.f);

        int N = nx * ny * nz;
        pressure.assign(N, 0.f);
        s_field.assign(N, 1.f);
        type.assign(N, CellType::Empty);
        density.assign(N, 0.f);
    }

    void SpatialHash::build(const Grid& grid, const Particles& particles) {
        int numParticles = particles.size();
        int numCells     = grid.size();
        particleList.resize(numParticles);
        cellStart.assign(numCells + 1, 0);

        for (auto & pos : particles.positions) {
            int cIdx = grid.getCellIdx(pos);
            cellStart[cIdx]++;
        }

        int currentOffset = 0;
        for (int i = 0; i < numCells; ++i) {
            int count    = cellStart[i];
            cellStart[i] = currentOffset;
            currentOffset += count;
        }
        cellStart[numCells] = currentOffset;

        std::vector<int> tempCounter(numCells, 0);
        for (int pIdx = 0; pIdx < numParticles; ++pIdx) {
            int cIdx              = grid.getCellIdx(particles.positions[pIdx]);
            int listPos           = cellStart[cIdx] + tempCounter[cIdx];
            particleList[listPos] = pIdx;
            tempCounter[cIdx]++;
        }
    }
}