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

    protected:
        FluidData data;
        glm::vec3 gravity = { 0.0f, -9.8f, 0.0f };
    };

    class HybridSolver : public FluidSolver {
    public:
        float flipRatio = 0.95f;
        std::unique_ptr<IncompressibilityStrategy> iStrategy;
        std::unique_ptr<TransferStrategy>          tStrategy;

        int   numSubSteps       = 1;
        int   numParticleIters  = 5;
        int   numPressureIters  = 30;
        bool  separateParticles = true;
        float overRelaxation    = 1.9;
        bool  compensateDrift   = true;
        float restDensity       = 0.0f;

        void step(float dt) override;
        void reset() override;

    private:
        void integrateParticles(float sdt);
        void handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRad, glm::vec3 obstacleVel);
        void pushParticlesApart(int numIters);
        void updateParticleDensity();
    };
}
