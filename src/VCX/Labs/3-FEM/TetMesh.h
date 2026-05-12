#pragma once
#include "ParticleSystem.h"

namespace VCX::Labs::FEM {
    struct Element {
        int indices[4];
        glm::mat3 inv_E;
        float     V_ref;
    };

    class TetMesh {
    public:
        std::vector<Element> elements;

        float mu, lambda, rho;

        void initialize(ParticleSystem & ps, int nx, int ny, int nz, float delta);

        void computeForces(ParticleSystem & ps);

    private:
        void addTet(ParticleSystem & ps, int i0, int i1, int i2, int i3);

        glm::mat3 calculateStressP(const glm::mat3 & F);
    };
}