#include "FEMSimulator.h"

namespace VCX::Labs::FEM {
    void FEMSimulator::init() {
        float E   = 20000.0f;
        float nu  = 0.2f;
        float rho = 400.0f;

        mesh.mu = E / (2.0f * (1.0f + nu));
        mesh.lambda = (E * nu) / ((1.0f + nu) * (1.0f - 2.0f * nu));

        mesh.initialize(ps, 32, 8, 8, 0.25f);
    }

    void FEMSimulator::update() {
        int   subSteps = 5;
        float sdt      = dt / subSteps;
        for (int i = 0; i < subSteps; ++i) {
            step(sdt);
        }
    }

    void FEMSimulator::step(float dt) {
        ps.clearForces(gravity);
        mesh.computeForces(ps);
        ps.step(dt);
    }
}