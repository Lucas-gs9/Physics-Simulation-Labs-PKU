#pragma once
#include "ParticleSystem.h"

namespace VCX::Labs::FEM {
    struct Element {
        int indices[4];
        glm::mat3 inv_E;
        float     V_ref;

        glm::mat3 Fp = glm::mat3(1.0f);
        glm::mat3 F_prev = glm::mat3(1.0f);
    };

    class TetMesh {
    public:
        std::vector<Element> elements;
        std::vector<std::uint32_t> surfaceTriangles;

        float mu, lambda, rho;

        float eta       = 0.9f; 

        void initialize(ParticleSystem & ps, int nx, int ny, int nz, float delta);
        void reset();

        void computeForces(ParticleSystem & ps, float dt, int model = 0, bool elastoplastic = false);

    private:
        void addTet(ParticleSystem & ps, int i0, int i1, int i2, int i3);
        void extractSurface();

        glm::mat3 calculateStressP(const glm::mat3 & F, int model);
    };
}