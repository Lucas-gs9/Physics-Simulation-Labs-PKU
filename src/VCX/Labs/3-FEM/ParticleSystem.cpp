#include "ParticleSystem.h"

namespace VCX::Labs::FEM {
    void ParticleSystem::clearForces(const glm::vec3& gravity) {
#pragma omp parallel for
        for (int i = 0; i < size; ++i) {
            if (is_fixed[i]) f[i] = glm::vec3(0);
            else f[i] = mass[i] * gravity;
        }
    }

    void ParticleSystem::addForce(int id, const glm::vec3& f_val) {
        if (! is_fixed[id]) f[id] += f_val;
    }

    void ParticleSystem::step(float dt, float damp) {
#pragma omp parallel for
        for (int i = 0; i < size; ++i) {
            if (is_fixed[i]) continue;
            v[i] += f[i] * inv_m[i] * dt;
            v[i] *= damp;
            x[i] += v[i] * dt;
        }
    }

    void ParticleSystem::setParticle(int id, const glm::vec3 & pos, bool fixed) {
        x[id] = pos;
        is_fixed[id] = fixed;
    }

    void ParticleSystem::resize(int n) {
        x.resize(n);
        v.resize(n, glm::vec3(0.0f));
        f.resize(n, glm::vec3(0.0f));
        mass.assign(n, 0.0f);
        inv_m.resize(n, 1.0f);
        is_fixed.resize(n, false);
        size = n;
    }

    void ParticleSystem::addMass(int id, float m_val) {
        mass[id] += m_val;
    }

    void ParticleSystem::updateMass() {
        for (int i = 0; i < size; ++i) {
            if (mass[i] > 1e-9f) inv_m[i] = 1.0f / mass[i];
            else inv_m[i] = 0.0f;
        }
    }
}