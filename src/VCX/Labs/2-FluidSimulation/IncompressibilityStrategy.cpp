#include "IncompressibilityStrategy.h"

namespace VCX::Labs::Fluid {
    void GaussSiedelStrategy::solve(Grid& grid, int numIters, float dt, float overRelaxation, bool compensateDrift, float restDensity) {
        float stiffness = 0.05f;

        grid.v_prev = grid.v;
        grid.u_prev = grid.u;
        grid.w_prev = grid.w;

        for (int iter = 0; iter < numIters; ++iter) {
            bool reverse = (iter % 2 == 1);
            for (int i_raw = 0; i_raw < grid.nx; ++i_raw) {
                int i = reverse ? (grid.nx - 1 - i_raw) : i_raw;

                for (int j_raw = 0; j_raw < grid.ny; ++j_raw) {
                    int j = reverse ? (grid.ny - 1 - j_raw) : j_raw;

                    for (int k_raw = 0; k_raw < grid.nz; ++k_raw) {
                        int k         = reverse ? (grid.nz - 1 - k_raw) : k_raw;
                        int centerIdx = grid.cIdx(i, j, k);
                        if (grid.type[centerIdx] == CellType::Solid) continue;

                        int idxU0 = grid.uIdx(i, j, k); 
                        int idxU1 = grid.uIdx(i + 1, j, k); 
                        int idxV0 = grid.vIdx(i, j, k);    
                        int idxV1 = grid.vIdx(i, j + 1, k); 
                        int idxW0 = grid.wIdx(i, j, k);     
                        int idxW1 = grid.wIdx(i, j, k + 1);

                        float d = grid.u[idxU1] - grid.u[idxU0] + grid.v[idxV1] - grid.v[idxV0] + grid.w[idxW1] - grid.w[idxW0];
                        d *= overRelaxation;

                        float rho = grid.density[centerIdx];
                        if (compensateDrift && rho > restDensity) {
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

                        float msg = d / s;

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