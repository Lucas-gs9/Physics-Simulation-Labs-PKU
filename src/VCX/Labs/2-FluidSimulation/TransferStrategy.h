#pragma once
#include "FluidData.h"

namespace VCX::Labs::Fluid {
    class TransferStrategy {
    public:
        virtual ~TransferStrategy() = default;
        virtual void transferToGrid(Grid & grid, const Particles & particles, const SpatialHash & hash) = 0;
        virtual void transferFromGrid(const Grid & grid, Particles & particles) = 0;
    };

    class FlipStrategy:TransferStrategy {
    public:
        float flipRatio = 0.95;

        void transferToGrid(Grid & grid, const Particles & particles, const SpatialHash & hash) override;
        void transferFromGrid(const Grid & grid, Particles & particles) override;
    };
}