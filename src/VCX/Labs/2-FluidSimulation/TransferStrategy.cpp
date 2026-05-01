#include "TransferStrategy.h"

namespace VCX::Labs::Fluid {
    void FlipStrategy::transferToGrid(Grid& grid, const Particles& particles, const SpatialHash& hash){
        grid.fillFromParticles(FieldType::U, particles, hash);
        grid.fillFromParticles(FieldType::V, particles, hash);
        grid.fillFromParticles(FieldType::W, particles, hash);
    }

    void FlipStrategy::transferFromGrid(const Grid& grid, Particles& particles){
#pragma omp parallel for
        for (int i = 0; i < particles.size(); ++i) {
            glm::vec3 pos = particles.positions[i];

            glm::vec3 v_old = grid.sampleVelocity(pos, true);
            glm::vec3 v_new = grid.sampleVelocity(pos);
            glm::vec3 v_delta = v_new - v_old;

            glm::vec3 v_pic = v_new;
            glm::vec3 v_flip = particles.velocities[i] + v_delta;

            particles.velocities[i] = glm::mix(v_pic, v_flip, flipRatio);
        }
    }

    void ApicStrategy::transferFromGrid(const Grid& grid, Particles& particles) {
#pragma omp parallel for
        for (int i = 0; i < particles.size(); ++i) {
            glm::vec3 pos = particles.positions[i];
            particles.velocities[i] = grid.sampleVelocity(pos, false, false);

            particles.c_mat[i] = grid.sampleAffine(pos);
        }
    }

    void ApicStrategy::transferToGrid(Grid& grid, const Particles& particles, const SpatialHash& hash) {
        grid.fillFromParticles(FieldType::U, particles, hash, true);
        grid.fillFromParticles(FieldType::V, particles, hash, true);
        grid.fillFromParticles(FieldType::W, particles, hash, true);
    }
}