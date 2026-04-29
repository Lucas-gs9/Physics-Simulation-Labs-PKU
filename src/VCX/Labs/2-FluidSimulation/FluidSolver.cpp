#include "FluidSolver.h"
namespace VCX::Labs::Fluid {
    void HybridSolver::step(float dt) {
        float sdt = dt / numSubSteps;
        glm::vec3 obstaclePos(0.0f); // obstacle can be moved with mouse, as a user interaction
        glm::vec3 obstacleVel(0.0f);
        for (int step = 0; step < numSubSteps; step++) {
            integrateParticles(sdt);
            handleParticleCollisions(obstaclePos, 0.0, obstacleVel);
            if (separateParticles) {
                pushParticlesApart(numParticleIters);
                handleParticleCollisions(obstaclePos, 0.0, obstacleVel);
            }
            data.hash.build(data.grid, data.particles);

            tStrategy->transferToGrid(data.grid, data.particles, data.hash);
            data.grid.u_prev = data.grid.u;
            data.grid.v_prev = data.grid.v;
            data.grid.w_prev = data.grid.w;

            updateParticleDensity();
            iStrategy->solve(data.grid, numPressureIters, sdt, overRelaxation, compensateDrift);
            tStrategy->transferFromGrid(data.grid, data.particles);
        }
    }

    void HybridSolver::reset() {
        auto & grid      = data.grid;
        auto & particles = data.particles;
        auto & hash      = data.hash;
        
        //set up the scene
        int       res = 32;
        glm::vec3 tankSize(1.0f);
        glm::vec3 waterRatio(0.6f, 0.8f, 0.6f);

        float h        = tankSize.y / res;
        float p_radius = 0.3f * h;

        grid.reset(res, res, res, h);
        particles.clear();
        particles.radius = p_radius;

        //calculate for the particles
        float dx = 2.0f * p_radius;
        float dy = std::sqrt(3.0f) / 2.0f * dx;
        float dz = dx;

        float margin = h + p_radius;
        int   numX   = std::floor((waterRatio.x * tankSize.x - 2.0f * margin) / dx);
        int   numY   = std::floor((waterRatio.y * tankSize.y - 2.0f * margin) / dy);
        int   numZ   = std::floor((waterRatio.z * tankSize.z - 2.0f * margin) / dz);

        //generate particles
        data.particles.resize(numX * numY * numZ);
        int pIdx = 0;
        for (int j = 0; j < numY; ++j) {
            for (int i = 0; i < numX; ++i) {
                for (int k = 0; k < numZ; ++k) {
                    float offsetX = (j % 2 == 0) ? 0.0f : p_radius;
                    float offsetZ = (j % 2 == 0) ? 0.0f : p_radius;

                    glm::vec3 pos(
                        margin + dx * i + offsetX,
                        margin + dy * j,
                        margin + dz * k + offsetZ);

                    particles.positions[pIdx]  = pos - glm::vec3(0.5f);
                    particles.velocities[pIdx] = glm::vec3(0.0f);
                    particles.colors[pIdx]     = glm::vec3(0.2f, 0.5f, 1.0f);
                    pIdx++;
                }
            }
        }

        //set grid boundaries
        for (int i = 0; i < grid.nx; ++i) {
            for (int j = 0; j < grid.ny; ++j) {
                for (int k = 0; k < grid.nz; ++k) {
                    int cidx = grid.cIdx(i, j, k);
                    if (i <= 1 || i >= grid.nx - 2 || j <= 1 || j >= grid.ny - 2 || k <= 1 || k >= grid.nz - 2) {
                        grid.s_field[cidx] = 0.0f; 
                        grid.type[cidx]    = CellType::Solid;
                    } else {
                        grid.s_field[cidx] = 1.0f;
                        grid.type[cidx]    = CellType::Empty;
                    }
                }
            }
        }

        hash.build(grid, particles);

        float sum   = 0.0f;
        int   count = 0;
        for (int i = 0; i < data.grid.size(); ++i) {
            if (data.grid.type[i] == CellType::Fluid) {
                sum += data.grid.density[i];
                count++;
            }
        }

        if (count > 0) {
            restDensity = sum / count;
        } else {
            restDensity = 1.0f;
        }
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
                        grid.density[idx] = 0.0f;
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

                    grid.density[idx] = sumWeight;

                    if (sumWeight > 0.01f) {
                        grid.type[idx] = CellType::Fluid;
                    } else {
                        grid.type[idx] = CellType::Empty;
                    }
                }
            }
        }
    }

}