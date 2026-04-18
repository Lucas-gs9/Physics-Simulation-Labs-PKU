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
}