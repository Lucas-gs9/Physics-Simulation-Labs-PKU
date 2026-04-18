#pragma once
#include "FluidData.h"

namespace VCX::Labs::Fluid {
    class FluidSolver {
    public:
        virtual ~FluidSolver() = default;

        virtual void step(float dt) = 0;
        virtual void reset() = 0;

    protected:
        FluidData data;
        glm::vec3 gravity = { 0.0f, -9.8f, 0.0f };
    };

    class HybridSolver : public FluidSolver {
    public:
        float flipRatio = 0.95f;

        void step(float dt) override;
        void reset() override;

    private:
        void integrateParticles(float sdt);
        void handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRad, glm::vec3 obstacleVel);
        void pushParticlesApart(int numIters);
    };
}
