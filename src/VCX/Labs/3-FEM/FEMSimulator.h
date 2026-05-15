#pragma once
#include "TetMesh.h"

namespace VCX::Labs::FEM {

	class FEMSimulator {
    public:
        ParticleSystem ps;
        TetMesh        mesh;

        glm::vec3 gravity = glm::vec3(0.0f, 0.05f, 0.0f);

        int mdId = 0;
        bool useEP = false;

        void init();
        void reset();
        void update(float dt);
        void handleCollision();
	};
}