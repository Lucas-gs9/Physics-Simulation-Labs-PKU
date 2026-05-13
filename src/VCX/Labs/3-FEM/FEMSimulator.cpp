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

    void FEMSimulator::update(float dt) {
        int   subSteps = 80;
        float sdt      = dt / subSteps;
        for (int i = 0; i < subSteps; ++i) {
            step(sdt);
        }
    }

    void FEMSimulator::step(float dt) {
        ps.clearForces(gravity);
        mesh.computeForces(ps);
        ps.step(dt, 0.999f);
    }

    void FEMSimulator::reset() {
        ps.reset();
    }
}