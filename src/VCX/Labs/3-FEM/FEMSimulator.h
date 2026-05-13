#pragma once
#include "TetMesh.h"

namespace VCX::Labs::FEM {

	class FEMSimulator {
    public:
        ParticleSystem ps;
        TetMesh        mesh;

        glm::vec3 gravity = glm::vec3(0.0f, 0.05f, 0.0f);

        void init();
        void update(float dt);
        void reset();

    private:
        void step(float dt);
	};
}