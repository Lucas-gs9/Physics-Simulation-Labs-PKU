#include "FluidSolver.h"
namespace VCX::Labs::Fluid {
    void HybridSolver::step(float dt) {

    }

    void HybridSolver::reset() {

    }

    void HybridSolver::integrateParticles(float sdt) {
        int numParticles = data.particles.size();
        #pragma omp parallel for
        for (int i = 0; i < numParticles; ++i) {
            data.particles.velocities[i] += gravity * sdt;
            data.particles.positions[i] += data.particles.velocities[i] * sdt;
        }
    }

    void HybridSolver::handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRad, glm::vec3 obstacleVel) {
        float h = data.grid.h;
        float r = 0.05f * h;

        float minX = r, maxX = data.grid.nx * h - r;
        float minY = r, maxY = data.grid.ny * h - r;
        float minZ = r, maxZ = data.grid.nz * h - r;

        int numParticles = data.particles.size();

        #pragma omp parallel for
        for (int i = 0; i < numParticles; ++i) {
            glm::vec3 & p = data.particles.positions[i];
            glm::vec3 & v = data.particles.velocities[i];

            if (p.x < minX) {
                p.x = minX;
                v.x = 0.0f;
            }
            if (p.x > maxX) {
                p.x = maxX;
                v.x = 0.0f;
            }

            if (p.y < minY) {
                p.y = minY;
                v.y = 0.0f;
            }
            if (p.y > maxY) {
                p.y = maxY;
                v.y = 0.0f;
            }

            if (p.z < minZ) {
                p.z = minZ;
                v.z = 0.0f;
            }
            if (p.z > maxZ) {
                p.z = maxZ;
                v.z = 0.0f;
            }
        }
    }

    void HybridSolver::pushParticlesApart(int numIters) {
        Particles & particles = data.particles;
        Grid &      grid      = data.grid;
        SpatialHash & hash      = data.hash;
        float r = particles.radius;

        for (int n = 0; n < numIters; n++) {
            hash.build(grid, particles);

            for (int i = 0; i < particles.size(); i++) {
                glm::vec3&  pos_i = particles.positions[i];
                glm::ivec3 cell  = grid.getCellCoord(pos_i);

                for (int ni = cell.x - 1; ni <= cell.x + 1; ni++) {
                    for (int nj = cell.y - 1; nj <= cell.y + 1; nj++) {
                        for (int nk = cell.z - 1; nk <= cell.z + 1; nk++) {
                            if (ni < 0 || ni >= grid.nx || nj < 0 || nj >= grid.ny || nk < 0 || nk >= grid.nz)
                                continue;

                            int cIdx  = grid.cIdx(ni, nj, nk);
                            int start = hash.cellStart[cIdx];
                            int end   = hash.cellStart[cIdx + 1];

                            for (int p = start; p < end; p++) {
                                int j = hash.particleList[p];
                                if (j <= i) continue;

                                glm::vec3 & pos_j = particles.positions[j];
                                glm::vec3   d     = pos_i - pos_j;
                                float       dist  = glm::length(d);

                                if (dist < 2 * r) {
                                    glm::vec3 s = 0.5f * d * (2 * r - dist) / dist;
                                    pos_i += s;
                                    pos_j -= s;
                                }
                            }
                        }
                    }
                }

            }
        }
    }

    void HybridSolver::updateParticleDensity() {
        Grid & grid = data.grid;
        Particles & particles = data.particles;
        const SpatialHash & hash      = data.hash;
        int    nx   = grid.nx;
        int    ny   = grid.ny;
        int    nz   = grid.nz;
        #pragma omp parallel for collapse(3)
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {
                for (int k = 0; k < nz; k++) {
                    int idx = grid.cIdx(i, j, k);

                    if (grid.s_field[idx] == 0.0f) {
                        grid.type[idx] = CellType::Solid;
                        continue;
                    }

                    float     sumWeight = 0.0f;
                    glm::vec3 cellGPos  = glm::vec3(i + 0.5f, j + 0.5f, k + 0.5f);
                    for (int ni = i - 1; ni <= i + 1; ++ni) {
                        for (int nj = j - 1; nj <= j + 1; ++nj) {
                            for (int nk = k - 1; nk <= k + 1; ++nk) {
                                if (ni < 0 || ni >= nx || nj < 0 || nj >= ny || nk < 0 || nk >= nz) continue;

                                int neighborCIdx = grid.cIdx(ni, nj, nk);
                                int pStart       = hash.cellStart[neighborCIdx];
                                int pEnd         = hash.cellStart[neighborCIdx + 1];

                                for (int p = pStart; p < pEnd; ++p) {
                                    int       pIdx  = hash.particleList[p];
                                    glm::vec3 pGPos = particles.positions[pIdx] * grid.inv_h;

                                    glm::vec3 d = glm::abs(cellGPos - pGPos);
                                    if (d.x < 1.0f && d.y < 1.0f && d.z < 1.0f) {
                                        sumWeight += (1.0f - d.x) * (1.0f - d.y) * (1.0f - d.z);
                                    }
                                }
                            }
                        }
                    }

                    int centerIdx  = grid.cIdx(cellGPos.x, cellGPos.y, cellGPos.z);
                    grid.density[centerIdx] = sumWeight;

                    if (sumWeight > 0.0001f) {
                        grid.type[centerIdx] = CellType::Fluid;
                    } else {
                        grid.type[centerIdx] = CellType::Empty;
                    }
                }
            }
        }
    }

}