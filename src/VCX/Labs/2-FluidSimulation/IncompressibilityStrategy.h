#pragma once

#include "FluidData.h"
namespace VCX::Labs::Fluid {
    class IncompressibilityStrategy {
    public:
        virtual ~IncompressibilityStrategy() = default;
        
        virtual void solve(
            Grid & grid,
            int    numIters,
            float  dt,
            float  overRelaxation,
            bool   compensateDrift,
            float restDensity) = 0;
    };

    class GaussSiedelStrategy : public IncompressibilityStrategy {
        void solve(
            Grid & grid,
            int    numIters,
            float  dt,
            float  overRelaxation,
            bool   compensateDrift,
            float  restDensity) override;
    };
}