#include "FluidData.h"
#include "utils.h"
#include<iostream>
namespace VCX::Labs::Fluid {
    void Particles::resize(int n) {
        positions.resize(n);
        velocities.resize(n);
        colors.resize(n, glm::vec3(1.f, 1.f, 1.f));
        c_mat.resize(n, glm::mat3(0.0f));
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
        //this is logical idx
        glm::ivec3 coord = getCellCoord(worldPos);
        return (coord.x * ny * nz) + (coord.y * nz) + coord.z;
    }

    glm::vec3 Grid::sampleVelocity(const glm::vec3 & worldPos, bool isPrev, bool isLinear) const {
        glm::vec3 g = worldPos * inv_h;
        float     uVal, vVal, wVal;

        auto & u_f = isPrev ? u_prev : u;
        auto & v_f = isPrev ? v_prev : v;
        auto & w_f = isPrev ? w_prev : w;

        if (isLinear) {
            uVal = triInterpolate(u_f, { g.x, g.y - 0.5f, g.z - 0.5f }, [this](int i, int j, int k) { return uIdx(i, j, k); });
            vVal = triInterpolate(v_f, { g.x - 0.5f, g.y, g.z - 0.5f }, [this](int i, int j, int k) { return vIdx(i, j, k); });
            wVal = triInterpolate(w_f, { g.x - 0.5f, g.y - 0.5f, g.z }, [this](int i, int j, int k) { return wIdx(i, j, k); });
        }
        else {
            uVal = quadInterpolate(u_f, { g.x, g.y - 0.5f, g.z - 0.5f }, [this](int i, int j, int k) { return uIdx(i, j, k); });
            vVal = quadInterpolate(v_f, { g.x - 0.5f, g.y, g.z - 0.5f }, [this](int i, int j, int k) { return vIdx(i, j, k); });
            wVal = quadInterpolate(w_f, { g.x - 0.5f, g.y - 0.5f, g.z }, [this](int i, int j, int k) { return wIdx(i, j, k); });
        }

        return glm::vec3(uVal, vVal, wVal);
    }

    void Grid::fillFromParticles(FieldType type, const Particles& particles, const SpatialHash& hash, bool useC) {
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
        for (int i = -1; i <= fSize.x; ++i) {
            for (int j = -1; j <= fSize.y; ++j) {
                for (int k = -1; k <= fSize.z; ++k) {
                    glm::vec3  faceGPos = glm::vec3(i, j, k) + offset;
                    float      sumV = 0.0f, sumW = 0.0f;

                    glm::vec3 xi = faceGPos * h;

                    glm::ivec3 minC = glm::floor(faceGPos - 1.5f);
                    glm::ivec3 maxC = glm::floor(faceGPos + 1.5f);

                    for (int ni = minC.x; ni <= maxC.x; ++ni) {
                        for (int nj = minC.y; nj <= maxC.y; ++nj) {
                            for (int nk = minC.z; nk <= maxC.z; ++nk) {
                                if (ni < 0 || ni >= nx || nj < 0 || nj >= ny || nk < 0 || nk >= nz) continue;

                                int cIdx = this->gridIdx(ni, nj, nk);
                                for (int p = hash.cellStart[cIdx]; p < hash.cellStart[cIdx + 1]; ++p) {
                                    int       pIdx  = hash.particleList[p];
                                    glm::vec3 pGPos = particles.positions[pIdx] * inv_h;

                                    glm::vec3 d = glm::abs(faceGPos - pGPos);

                                    float weight = 0.0f;
                                    if (useC) {
                                        if (d.x < 1.5f && d.y < 1.5f && d.z < 1.5f) {
                                            weight = quadraticKernel(d.x) * quadraticKernel(d.y) * quadraticKernel(d.z);
                                        }
                                    } else {
                                        if (d.x < 1.0f && d.y < 1.0f && d.z < 1.0f) {
                                            weight = (1.0f - d.x) * (1.0f - d.y) * (1.0f - d.z);
                                        }
                                    }

                                    if (weight > 1e-9f) {
                                        float pVel = 0.0f;
                                        if (! useC) {
                                            pVel = (&particles.velocities[pIdx].x)[(int) type];
                                        } else {
                                            glm::vec3 xp         = particles.positions[pIdx];
                                            glm::vec3 dist = (xi - xp);
                                            if (type == FieldType::U) pVel = particles.velocities[pIdx].x + glm::dot(particles.c_mat[pIdx][0], dist);
                                            else if (type == FieldType::V) pVel = particles.velocities[pIdx].y + glm::dot(particles.c_mat[pIdx][1], dist);
                                            else pVel = particles.velocities[pIdx].z + glm::dot(particles.c_mat[pIdx][2], dist);
                                        }

                                        sumV += pVel * weight;
                                        sumW += weight;
                                    }
                                }
                            }
                        }
                    }
                    int gridIdx;
                    if (type == FieldType::U) gridIdx = uIdx(i, j, k);
                    else if (type == FieldType::V) gridIdx = vIdx(i, j, k);
                    else gridIdx = wIdx(i, j, k);
                    (*field)[gridIdx] = (sumW > 1e-6f) ? (sumV / sumW) : 0.0f;
                }
            }
        }
    }

    glm::mat3 Grid::sampleAffine(const glm::vec3& pos) const{
        glm::vec3 row { 0.f };

        FieldType types[3] = { FieldType::U, FieldType::V, FieldType::W };

        glm::mat3 C { 0.f };


        for (int f = 0; f < 3; ++f) {
            glm::vec3 offset(0.5f);
            if (f == 0) offset.x = 0.0f;      
            else if (f == 1) offset.y = 0.0f; 
            else offset.z = 0.0f;            

            glm::vec3  grid_pos = pos * inv_h - offset;
            glm::ivec3 base_idx = glm::floor(grid_pos);

            float weightX[3], weightY[3], weightZ[3];
            for (int m = -1; m <= 1; ++m) {
                weightX[m + 1] = quadraticKernel(float(base_idx.x + m) - grid_pos.x);
                weightY[m + 1] = quadraticKernel(float(base_idx.y + m) - grid_pos.y);
                weightZ[m + 1] = quadraticKernel(float(base_idx.z + m) - grid_pos.z);
            }

            for (int k = -1; k <= 1; ++k)
                for (int j = -1; j <= 1; ++j) {
                    float wyz = weightY[j + 1] * weightZ[k + 1];
                    for (int i = -1; i <= 1; ++i) {
                        glm::ivec3 cur = base_idx + glm::ivec3(i, j, k);
                        glm::vec3  d   = glm::vec3(cur) - grid_pos;

                        float weight = weightX[i + 1] * wyz;
                        float val = 0.f;

                        if (f == 0) val = u[uIdx(cur.x, cur.y, cur.z)];
                        else if (f == 1) val = v[vIdx(cur.x, cur.y, cur.z)];
                        else val = w[wIdx(cur.x, cur.y, cur.z)];

                        C[f] += weight * val * (d * h);
                    }
                }
        }
        return C * (4.f / (h * h)) * 0.9f;
    }

    int Grid::size() const{
        return nx * ny * nz;
    }

    void Grid::resetStep() {
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

        int ax = nx + 2, ay = ny + 2, az = nz + 2;

        u.assign((ax + 1) * ay * az, 0.f);
        v.assign(ax * (ay + 1) * az, 0.f);
        w.assign(ax * ay * (az + 1), 0.f);
        u_prev.assign(u.size(), 0.f);
        v_prev.assign(v.size(), 0.f);
        w_prev.assign(w.size(), 0.f);

        int N = ax * ay * az;
        pressure.assign(N, 0.f);
        s_field.assign(N, 0.f);
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