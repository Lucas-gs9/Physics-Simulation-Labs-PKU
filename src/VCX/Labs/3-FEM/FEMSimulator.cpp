#include "FEMSimulator.h"
#include <iostream>

namespace VCX::Labs::FEM {
    void FEMSimulator::init() {
        float E   = 20000.0f;
        float nu  = 0.2f;
        float rho = 400.0f;

        mesh.mu = E / (2.0f * (1.0f + nu));
        mesh.lambda = (E * nu) / ((1.0f + nu) * (1.0f - 2.0f * nu));
        mesh.rho    = rho;

        mesh.initialize(ps, 16, 4, 4, 0.5f);
    }

    void FEMSimulator::reset() {
        ps.reset();
        mesh.reset();
    }

    void FEMSimulator::update(float dt) {
        mesh.computeForces(ps, dt, mdId, useEP);
        handleCollision();
        ps.step(dt, 0.9988f);
    }

    void FEMSimulator::handleCollision() {
        float k_penalty = 5000.0f; 
        float c_damping = 12.0f;
#pragma omp parallel for
        for (int i = 0; i < ps.size; ++i) {
            glm::vec3 & x = ps.x[i];
            glm::vec3 & v = ps.v[i];
            glm::vec3   f_ext(0.0f);

            float ground_y = -2.5f;
            if (x.y < ground_y) {
                float d       = ground_y - x.y; 
                float v_n     = v.y;           
                float force_y = k_penalty * d - c_damping * v_n;
                f_ext.y += force_y;
            }

            float wall_x = 0.0f;
            if (x.x < wall_x) {
                float d       = wall_x - x.x; 
                float v_n     = v.x;          
                float force_x = k_penalty * d - c_damping * v_n;
                f_ext.x += force_x;
            }

            ps.f[i] += f_ext;
        }
    }
}