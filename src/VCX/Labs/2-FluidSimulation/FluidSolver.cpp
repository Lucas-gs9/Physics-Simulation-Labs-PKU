#include "FluidSolver.h"
#include "utils.h"
#include <Eigen/Sparse>
#include <iostream>
#include <Eigen/IterativeLinearSolvers>
#include <random>
namespace VCX::Labs::Fluid {
    void HybridSolver::step(float dt) {
        Grid & grid = data.grid;
        SpatialHash & hash = data.hash;
        float sdt   = dt / numSubSteps;
        obstaclePos = glm::vec3(0.f); // obstacle can be moved with mouse, as a user interaction
        glm::vec3 obstacleVel(0.0f);
        for (int step = 0; step < numSubSteps; step++) {
            integrateParticles(sdt);
            handleParticleCollisions(obstaclePos, 0.0, obstacleVel);
            if (separateParticles) {
                pushParticlesApart(numParticleIters);
                if (useIDP) {
                    solveIDP(sdt);
                }
            }
            handleParticleCollisions(obstaclePos, 0.0, obstacleVel);

            data.grid.resetStep(); // clear u,v,w
            hash.build(grid,data. particles);

            tStrategy->transferToGrid(data.grid, data.particles, data.hash);
            data.grid.u_prev = data.grid.u;
            data.grid.v_prev = data.grid.v;
            data.grid.w_prev = data.grid.w;

            updateParticleDensity();
            iStrategy->solve(data.grid, numPressureIters, sdt, overRelaxation, compensateDrift, restDensity);
            tStrategy->transferFromGrid(data.grid, data.particles);
        }
        updateParticleColor();
    }

    void HybridSolver::reset() {
        auto & grid      = data.grid;
        auto & particles = data.particles;
        auto & hash      = data.hash;

        // set up the scene
        int       res = 22;
        glm::vec3 tankSize(1.0f);
        glm::vec3 waterRatio(0.6f, 0.8f, 0.6f);

        float h        = tankSize.y / res;
        float p_radius = 0.3f * h;

        grid.reset(res, res, res, h);
        particles.clear();
        particles.radius = p_radius;

        // calculate for the particles
        float dx = 2.0f * p_radius;
        float dy = std::sqrt(3.0f) / 2.0f * dx;
        float dz = dx;

        float margin = h + p_radius;
        int   numX   = std::floor((waterRatio.x * tankSize.x - 2.0f * margin) / dx);
        int   numY   = std::floor((waterRatio.y * tankSize.y - 2.0f * margin) / dy);
        int   numZ   = std::floor((waterRatio.z * tankSize.z - 2.0f * margin) / dz);

        // generate particles

        float totalWidthX = (numX - 1) * dx;
        float totalWidthY = (numY - 1) * dy;
        float totalWidthZ = (numZ - 1) * dz;

        float startX = (tankSize.x - totalWidthX) * 0.5f;
        float startY = margin;
        float startZ = (tankSize.z - totalWidthZ) * 0.5f;

        particles.resize(numX * numY * numZ);
        int pIdx = 0;
        for (int j = 0; j < numY; ++j) {
            for (int i = 0; i < numX; ++i) {
                for (int k = 0; k < numZ; ++k) {
                    float offsetX = (j % 2 == 0) ? 0.0f : p_radius;
                    float offsetZ = (j % 2 == 0) ? 0.0f : p_radius;

                    glm::vec3 pos(
                        startX + dx * i + offsetX,
                        startY + dy * j,
                        startZ + dz * k + offsetZ);

                    particles.positions[pIdx]  = pos;
                    particles.velocities[pIdx] = glm::vec3(0.0f);
                    particles.colors[pIdx]     = fixedColor;
                    pIdx++;
                }
            }
        }

        // set grid boundaries
        for (int i = -1; i <= grid.nx; ++i) {
            for (int j = -1; j <= grid.ny; ++j) {
                for (int k = -1; k <= grid.nz; ++k) {
                    int cidx = grid.cIdx(i, j, k);
                    if (i <= 0 || i >= grid.nx - 1 || j <= 0 || j >= grid.ny - 1 || k <= 0 || k >= grid.nz - 1) {
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

        updateParticleDensity(); 
        float sum   = 0;
        int   count = 0;
        for (int i = 0; i < grid.size(); i++) {
            if (grid.type[i] == CellType::Fluid) {
                sum += grid.density[i];
                count++;
            }
        }
        this->restDensity = sum / count;
    }

    void HybridSolver::integrateParticles(float sdt) {
        int       numParticles = data.particles.size();
        glm::vec3 dv           = -gravity * sdt;
#pragma omp parallel for
        for (int i = 0; i < numParticles; ++i) {
            data.particles.velocities[i] += dv;
            data.particles.positions[i] += data.particles.velocities[i] * sdt;
        }
    }

    void HybridSolver::handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRad, glm::vec3 obstacleVel) {
        float h = data.grid.h;
        float r = data.particles.radius;
        float k = 2.2;

        float minX = k * r, maxX = data.grid.nx * h - k * r;
        float minY = k * r, maxY = data.grid.ny * h - k * r;
        float minZ = k * r, maxZ = data.grid.nz * h - k * r;

        int numParticles = data.particles.size();
        float restitution  = 0.0f;
        float friction     = 0.05f;

#pragma omp parallel for
        for (int i = 0; i < numParticles; ++i) {
            glm::vec3 & p = data.particles.positions[i];
            glm::vec3 & v = data.particles.velocities[i];

            if (p.x < minX) {
                p.x = minX;
                if (v.x < 0) v.x = -restitution * v.x;
                v.y *= (1.0f - friction); 
                v.z *= (1.0f - friction);
            } else if (p.x > maxX) {
                p.x = maxX;
                if (v.x > 0) v.x = -restitution * v.x;
                v.y *= (1.0f - friction);
                v.z *= (1.0f - friction);
            }

            if (p.y < minY) {
                p.y = minY;
                v.y = -restitution * v.y;
                if (v.y < 0) v.y = -restitution * v.y;
                v.x *= (1.0f - friction);
                v.z *= (1.0f - friction);
            } else if (p.y > maxY) {
                p.y = maxY;
                if (v.y > 0) v.y = -restitution * v.y;
                v.x *= (1.0f - friction);
                v.z *= (1.0f - friction);
            }

            if (p.z < minZ) {
                p.z = minZ;
                if (v.z < 0) v.z = -restitution * v.z;
                v.x *= (1.0f - friction);
                v.y *= (1.0f - friction);
            } else if (p.z > maxZ) {
                p.z = maxZ;
                if (v.z > 0) v.z = -restitution * v.z;
                v.x *= (1.0f - friction);
                v.y *= (1.0f - friction);
            }
        }
    }

    void HybridSolver::pushParticlesApart(int numIters) {
        Particles &   particles = data.particles;
        Grid &        grid      = data.grid;
        SpatialHash & hash      = data.hash;
        float         r         = particles.radius;

        hash.build(grid, particles);
        static std::mt19937                   gen(std::random_device {}());
        std::uniform_real_distribution<float> dis(-0.001f, 0.001f);

        for (int n = 0; n < numIters; n++) {
            for (int i = 0; i < particles.size(); i++) {
                glm::vec3 & pos_i = particles.positions[i];
                glm::ivec3  cell  = grid.getCellCoord(pos_i);

                int x_start = std::max(0, cell.x - 1);
                int x_end   = std::min(grid.nx - 1, cell.x + 1);
                int y_start = std::max(0, cell.y - 1);
                int y_end   = std::min(grid.ny - 1, cell.y + 1);
                int z_start = std::max(0, cell.z - 1);
                int z_end   = std::min(grid.nz - 1, cell.z + 1);
                for (int ni = x_start; ni <= x_end; ni++) {
                    for (int nj = y_start; nj <= y_end; nj++) {
                        for (int nk = z_start; nk <= z_end; nk++) {
                            int cIdx  = grid.gridIdx(ni, nj, nk);
                            int start = hash.cellStart[cIdx];
                            int end   = hash.cellStart[cIdx + 1];

                            for (int p = start; p < end; p++) {
                                int j = hash.particleList[p];
                                if (j <= i) continue;

                                glm::vec3 & pos_j     = particles.positions[j];
                                glm::vec3   d         = pos_i - pos_j;
                                float       dist2     = glm::dot(d, d);
                                float       min_dist  = 2 * r;
                                float       min_dist2 = min_dist * min_dist;

                                if (dist2 > 0.000001f && dist2 < min_dist2) {
                                    float     dist = sqrt(dist2);
                                    glm::vec3 s    = 0.5f * d * (2 * r - dist) / dist;
                                    pos_i += s;
                                    pos_j -= s;
                                } else if (dist2 <= 0.000001f) {
                                    pos_i += glm::vec3(dis(gen), dis(gen), dis(gen));
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void HybridSolver::updateParticleDensity() {
        Grid &              grid      = data.grid;
        Particles &         particles = data.particles;
        SpatialHash & hash      = data.hash;
        int                 nx        = grid.nx;
        int                 ny        = grid.ny;
        int                 nz        = grid.nz;

        hash.build(grid, particles);
#pragma omp parallel for collapse(3)
        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {
                for (int k = 0; k < nz; k++) {
                    int idx = grid.cIdx(i, j, k);

                    float     sumWeight   = 0.0f;
                    float     solidWeight = 0.0f;
                    glm::vec3 cellGPos    = glm::vec3(i + 0.5f, j + 0.5f, k + 0.5f);
                    for (int ni = i - 1; ni <= i + 1; ++ni) {
                        for (int nj = j - 1; nj <= j + 1; ++nj) {
                            for (int nk = k - 1; nk <= k + 1; ++nk) {
                                if (ni < 0 || ni >= nx || nj < 0 || nj >= ny || nk < 0 || nk >= nz) continue;

                                int neighborCIdx = grid.gridIdx(ni, nj, nk);
                                int pStart       = hash.cellStart[neighborCIdx];
                                int pEnd         = hash.cellStart[neighborCIdx + 1];

                                for (int p = pStart; p < pEnd; ++p) {
                                    int       pIdx  = hash.particleList[p];
                                    glm::vec3 pGPos = particles.positions[pIdx] * grid.inv_h;

                                    glm::vec3 d = glm::abs(cellGPos - pGPos);
                                    float     w = quadraticKernel(cellGPos.x - pGPos.x) * quadraticKernel(cellGPos.y - pGPos.y) * quadraticKernel(cellGPos.z - pGPos.z);
                                    sumWeight += w;
                                }
                            }
                        }
                    }

                    for (int ni = i - 1; ni <= i + 1; ++ni) {
                        for (int nj = j - 1; nj <= j + 1; ++nj) {
                            for (int nk = k - 1; nk <= k + 1; ++nk) {
                                if (ni < 0 || ni >= nx || nj < 0 || nj >= ny || nk < 0 || nk >= nz || grid.s_field[grid.cIdx(ni, nj, nk)] == 0.0f) {
                                    glm::vec3 solidCellPos = glm::vec3(ni + 0.5f, nj + 0.5f, nk + 0.5f);
                                    glm::vec3 d            = glm::abs(cellGPos - solidCellPos);

                                    solidWeight += quadraticKernel(d.x) * quadraticKernel(d.y) * quadraticKernel(d.z) * restDensity;
                                }
                            }
                        }
                    }

                    float totalDensity = sumWeight;
                    bool  nearBoundary = (i == 0 || i == nx - 1 || j == 0 || j == ny - 1 || k == 0 || k == nz - 1);

                    if (nearBoundary) {
                        //totalDensity *= 1.5f;
                    }
                    if (grid.s_field[idx] == 0.0f) {
                        grid.type[idx] = CellType::Solid;
                        grid.density[idx] = restDensity;
                    } else if (sumWeight > 0.3*restDensity) {
                        grid.type[idx] = CellType::Fluid;
                        grid.density[idx] = totalDensity;
                    } else {
                        grid.type[idx]    = CellType::Empty;
                        grid.density[idx] = 0.0f;
                    }
                }
            }
        }

    }

    void HybridSolver::updateParticleColor() {
        Particles & particles = data.particles;

        if (colorFromV) {
            glm::vec3 coldColor(0.2f, 0.2f, 1.0f);
            glm::vec3 hotColor(1.0f, 0.2f, 0.2f);
#pragma omp parallel for
            for (int i = 0; i < particles.size(); ++i) {
                float speed         = glm::length(particles.velocities[i]);
                float t             = glm::clamp(speed * 2.0f, 0.0f, 1.0f);
                particles.colors[i] = glm::mix(coldColor, hotColor, t);
            }
        } else {
            std::fill(particles.colors.begin(), particles.colors.end(), fixedColor);
        }
    }

    void HybridSolver::solveIDP(float sdt) {
        Grid & grid = data.grid;

        Particles & particles = data.particles;

        int numCells = (grid.nx + 2) * (grid.ny + 2) * (grid.nz + 2);

        
        updateParticleDensity();

        float maxD = -1.0f;

        float minD = 1e10f;

        float avgD = 0.0f;

        int fluidCount = 0;



        /* for (int i = 0; i < numCells; ++i) {

            if (grid.type[i] == CellType::Fluid) {

                float d = grid.density[i];

                if (d > maxD) maxD = d;

                if (d < minD) minD = d;

                avgD += d;

                fluidCount++;

            }

        }



        if (fluidCount > 0) {

            avgD /= fluidCount;

            printf("Density Stats: Max: %.4f, Min: %.4f, Avg: %.4f, Rest: %.4f, Count: %d\n", maxD, minD, avgD, restDensity, fluidCount);

        } else {

            printf("No Fluid cells found!\n");

        }*/

        std::vector<int> gridToMatrix(numCells, -1);

        int activeCount = 0;

        for (int i = 0; i < numCells; ++i) {
            if (grid.type[i] == CellType::Fluid) {
                gridToMatrix[i] = activeCount++;
            }
        }

        if (activeCount == 0) return;

        Eigen::SparseMatrix<float> A(activeCount, activeCount);

        Eigen::VectorXf b(activeCount);

        std::vector<Eigen::Triplet<float>> triplets;

        float inv_h2 = 1.0f / (grid.h * grid.h);

        for (int x = 0; x < grid.nx; ++x) {
            for (int y = 0; y < grid.ny; ++y) {
                for (int z = 0; z < grid.nz; ++z) {
                    int idx = grid.cIdx(x, y, z);

                    int row = gridToMatrix[idx];

                    if (row == -1)

                    {
                        continue;
                    }

                    float currentDensity = grid.density[idx];

                    float densityError = currentDensity - restDensity;

                    if (densityError > 0) b(row) = densityError; 
                    else b(row) = 0;
                    

                    int neighbors[6] = {

                        grid.cIdx(x + 1, y, z), grid.cIdx(x - 1, y, z), grid.cIdx(x, y + 1, z), grid.cIdx(x, y - 1, z), grid.cIdx(x, y, z + 1), grid.cIdx(x, y, z - 1)

                    };

                    float diag = 0;

                    for (int nIdx : neighbors) {
                        if (grid.type[nIdx] == CellType::Fluid) {
                            int col = gridToMatrix[nIdx];

                            triplets.push_back(Eigen::Triplet<float>(row, col, -inv_h2));

                            diag += inv_h2;

                        } else if (grid.type[nIdx] == CellType::Empty) {
                            diag += inv_h2;

                        } else if (grid.type[nIdx] == CellType::Solid) {
                        }
                    }

                    triplets.push_back(Eigen::Triplet<float>(row, row, diag + 1e-6f));
                }
            }
        }

        A.setFromTriplets(triplets.begin(), triplets.end());

        Eigen::ConjugateGradient<Eigen::SparseMatrix<float>, Eigen::Lower | Eigen::Upper> solver;
        solver.setMaxIterations(500);
        solver.setTolerance(1e-6);

        solver.compute(A);

        float b_max  = b.maxCoeff();

        float b_min  = b.minCoeff();

        float b_norm = b.norm();

        //printf("B-Vector Debug: Max: %f, Min: %f, Norm: %f, Active: %d\n", b_max, b_min, b_norm, activeCount);
        Eigen::VectorXf p = solver.solve(b);
        //printf("CG Iterations: %ld, Error: %f\n", solver.iterations(), solver.error());

        // printf("P-Vector Debug: Max: %f, Norm: %f\n", p.maxCoeff(), p.norm());

        std::vector<float> p_grid(numCells, 0.0f);

        for (int i = 0; i < numCells; ++i) {
            if (gridToMatrix[i] != -1) p_grid[i] = p(gridToMatrix[i]);
        }
        //std::cout << "p_max: " << *std::max_element(p_grid.begin(), p_grid.end()) << std::endl;
        float deltaXFactor = -1.f / restDensity;

        float inv_2h = 0.5f / grid.h;

        auto idxFunc = [&](int i, int j, int k) {
            return grid.cIdx(i, j, k);
        };

#pragma omp parallel for
        for (int pIdx = 0; pIdx < (int) particles.positions.size(); pIdx++) {
            glm::vec3 pos = particles.positions[pIdx];

            glm::vec3 samplePos = pos * grid.inv_h;//-glm::vec3(0.5f);

            float eps = 0.2f;

            auto getSafeP = [&](glm::vec3 samplePos, float centerP) {
                int ix = (int) floor(samplePos.x);
                int iy = (int) floor(samplePos.y);
                int iz = (int) floor(samplePos.z);

                if (ix < 0 || ix >= grid.nx || iy < 0 || iy >= grid.ny || iz < 0 || iz >= grid.nz) {
                    return centerP;
                }

                int idx = grid.cIdx(ix, iy, iz);
                if (grid.type[idx] == CellType::Solid) {
                    return centerP; 
                }

                return triInterpolate(p_grid, samplePos, idxFunc);
            };

            float p_center = triInterpolate(p_grid, samplePos, idxFunc);

            glm::vec3 gradP;

            float p_right = getSafeP(samplePos + glm::vec3(eps, 0, 0), p_center);
            float p_left  = getSafeP(samplePos - glm::vec3(eps, 0, 0), p_center);
            gradP.x       = (p_right - p_left) / (2.0f * eps * grid.h);

            float p_up   = getSafeP(samplePos + glm::vec3(0, eps, 0), p_center);
            float p_down = getSafeP(samplePos - glm::vec3(0, eps, 0), p_center);
            gradP.y      = (p_up - p_down) / (2.0f * eps * grid.h);

            float p_front = getSafeP(samplePos + glm::vec3(0, 0, eps), p_center);
            float p_back  = getSafeP(samplePos - glm::vec3(0, 0, eps), p_center);
            gradP.z       = (p_front - p_back) / (2.0f * eps * grid.h);

            glm::vec3 deltaX = gradP * deltaXFactor;

            float maxStep = 0.5f * grid.h;

            if (glm::length(deltaX) > maxStep) deltaX = glm::normalize(deltaX) * maxStep;

            particles.positions[pIdx] += deltaX;
            particles.velocities[pIdx] += deltaX / sdt;
            //std::cout << deltaX.y << std::endl;
        }
    }
} // namespace VCX::Labs::Fluid