#pragma once

#include "FluidData.h"
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
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

    class CGStrategy : public IncompressibilityStrategy {
        void solve(
            Grid & grid,
            int    numIters,
            float  dt,
            float  overRelaxation,
            bool   compensateDrift,
            float  restDensity) override;

    private:
        Eigen::SparseMatrix<float> A;
        Eigen::ConjugateGradient<Eigen::SparseMatrix<float>, Eigen::Lower | Eigen::Upper, Eigen::DiagonalPreconditioner<float>> solver;
    };
}