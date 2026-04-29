#include "IncompressibilityStrategy.h"

namespace VCX::Labs::Fluid {
    void GaussSiedelStrategy::solve(Grid& grid, int numIters, float dt, float overRelaxation, bool compensateDrift, float restDensity) {
        float stiffness = 0.15f;

        for (int iter = 0; iter < numIters; ++iter) {
            for (int i = 0; i < grid.nx; ++i) {
                for (int j = 0; j < grid.ny; ++j) {
                    for (int k = 0; k < grid.nz; ++k) {
                        int centerIdx = grid.cIdx(i, j, k);
                        if (grid.type[centerIdx] != CellType::Fluid) continue;

                        int idxU0 = grid.uIdx(i, j, k); 
                        int idxU1 = grid.uIdx(i + 1, j, k); 
                        int idxV0 = grid.vIdx(i, j, k);    
                        int idxV1 = grid.vIdx(i, j + 1, k); 
                        int idxW0 = grid.wIdx(i, j, k);     
                        int idxW1 = grid.wIdx(i, j, k + 1);

                        float d = grid.u[idxU1] - grid.u[idxU0] + grid.v[idxV1] - grid.v[idxV0] + grid.w[idxW1] - grid.w[idxW0];

                        if (compensateDrift && restDensity > 0.0f) {
                            float rho = grid.density[centerIdx];
                            float compression = rho - restDensity;
                            if (compression > 0.0f) {
                                d -= stiffness * compression;
                            }
                        }

                        float s0 = grid.s_field[grid.cIdx(i - 1, j, k)];
                        float s1 = grid.s_field[grid.cIdx(i + 1, j, k)];
                        float s2 = grid.s_field[grid.cIdx(i, j - 1, k)];
                        float s3 = grid.s_field[grid.cIdx(i, j + 1, k)];
                        float s4 = grid.s_field[grid.cIdx(i, j, k - 1)];
                        float s5 = grid.s_field[grid.cIdx(i, j, k + 1)];
                        float s  = s0 + s1 + s2 + s3 + s4 + s5;

                        if (s <= 0.0f) continue;

                        float msg = (overRelaxation * d) / s;

                        grid.u[idxU0] += msg * s0;
                        grid.u[idxU1] -= msg * s1;
                        grid.v[idxV0] += msg * s2;
                        grid.v[idxV1] -= msg * s3;
                        grid.w[idxW0] += msg * s4;
                        grid.w[idxW1] -= msg * s5;
                    }
                }
            }
        }
    }
}