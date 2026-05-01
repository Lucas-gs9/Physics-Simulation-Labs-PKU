#pragma once
#include "FluidData.h"
#include <memory>
#include "TransferStrategy.h"
#include "IncompressibilityStrategy.h"

namespace VCX::Labs::Fluid {
    class FluidSolver {
    public:
        virtual ~FluidSolver() = default;

        virtual void step(float dt) = 0;
        virtual void reset() = 0;

        FluidData data;
        glm::vec3 gravity = { 0.0f, -9.8f, 0.0f };
    };

    class HybridSolver : public FluidSolver {
    public:
        std::unique_ptr<IncompressibilityStrategy> iStrategy;
        std::unique_ptr<TransferStrategy>          tStrategy;

        int   numSubSteps       = 2;
        int   numParticleIters  = 1;
        int   numPressureIters  = 15;
        bool  separateParticles = true;
        float overRelaxation    = 1.9;
        bool  compensateDrift   = true;
        float restDensity       = 8.0f;

        void step(float dt) override;
        void reset() override;

    private:
        void integrateParticles(float sdt);
        void handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRad, glm::vec3 obstacleVel);
        void pushParticlesApart(int numIters);
        void updateParticleDensity();
        void updateParticleColor();
    };
}
