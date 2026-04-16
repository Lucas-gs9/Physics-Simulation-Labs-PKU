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
}