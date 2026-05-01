#include "IncompressibilityStrategy.h"

namespace VCX::Labs::Fluid {
    void GaussSiedelStrategy::solve(Grid& grid, int numIters, float dt, float overRelaxation, bool compensateDrift, float restDensity) {
        float stiffness = 0.05f;

        for (int iter = 0; iter < numIters; ++iter) {
            for (int color = 0; color <= 1; ++color) {
#pragma omp parallel for collapse(2) schedule(static)
                for (int i = 0; i < grid.nx; ++i) {
                    for (int j = 0; j < grid.ny; ++j) {
                        int k_start = (i + j) % 2 == color ? 0 : 1;

                        for (int k = k_start; k < grid.nz; k += 2) {
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

    void CGStrategy::solve(Grid& grid, int numIters, float dt, float overRelaxation, bool compensateDrift, float restDensity) {
        int nx = grid.nx;
        int ny = grid.ny;
        int nz = grid.nz;
        int n  = (nx + 2) * (ny + 2) * (nz + 2);

        if (A.rows() != n) {
            A.resize(n, n);
            std::vector<Eigen::Triplet<float>> triplets;
            triplets.reserve(n * 7);

            for (int i = -1; i <= nx; ++i) {
                for (int j = -1; j <= ny; ++j) {
                    for (int k = -1; k <= nz; ++k) {
                        int  centerIdx = grid.cIdx(i, j, k);
                        bool is_fluid  = (i >= 0 && i < nx && j >= 0 && j < ny && k >= 0 && k < nz && grid.type[centerIdx] == CellType::Fluid);

                        if (! is_fluid) {
                            triplets.push_back(Eigen::Triplet<float>(centerIdx, centerIdx, 1.0f));
                            continue;
                        }

                        int neighbors[6] = {
                            grid.cIdx(i - 1, j, k), grid.cIdx(i + 1, j, k), grid.cIdx(i, j - 1, k), grid.cIdx(i, j + 1, k), grid.cIdx(i, j, k - 1), grid.cIdx(i, j, k + 1)
                        };

                        float diagonal = 0;
                        for (int nbIdx : neighbors) {
                            if (grid.type[nbIdx] == CellType::Fluid) {
                                triplets.push_back(Eigen::Triplet<float>(centerIdx, nbIdx, -1.0f));
                                diagonal += 1.0f;
                            } else if (grid.type[nbIdx] == CellType::Empty) {
                                diagonal += 1.0f;
                            }
                        }
                        triplets.push_back(Eigen::Triplet<float>(centerIdx, centerIdx, diagonal + 1e-4f));
                    }
                }
            }

            A.setFromTriplets(triplets.begin(), triplets.end());
            A.makeCompressed();
            solver.compute(A);
        }

        float stiffness = 0.05f;
        Eigen::VectorXf b         = Eigen::VectorXf::Zero(n);
        #pragma omp parallel for collapse(3)
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < ny; ++j) {
                for (int k = 0; k < nz; ++k) {
                    int centerIdx = grid.cIdx(i, j, k);
                    if (grid.type[centerIdx] != CellType::Fluid) continue;

                    float div = (grid.u[grid.uIdx(i + 1, j, k)] - grid.u[grid.uIdx(i, j, k)] + grid.v[grid.vIdx(i, j + 1, k)] - grid.v[grid.vIdx(i, j, k)] + grid.w[grid.wIdx(i, j, k + 1)] - grid.w[grid.wIdx(i, j, k)]) / dt;

                    if (compensateDrift && grid.density[centerIdx] > restDensity) {
                        //div -= (stiffness * (grid.density[centerIdx] - restDensity)) / dt;
                    }
                    b[centerIdx] = -div;
                }
            }
        }
        solver.setMaxIterations(numIters*5);
        solver.setTolerance(1e-6);
        Eigen::VectorXf p = solver.solve(b);

        float maxP = 10.0f;
#pragma omp parallel for
        for (int i = 0; i < grid.size(); ++i) {
            p[i] = std::max(-maxP, std::min(maxP, p[i]));
        }

        float maxDeltaV = 0.5f * (grid.h / dt);

        #pragma omp parallel for collapse(3)
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < ny; ++j) {
                for (int k = 0; k < nz; ++k) {
                    int   cIdx  = grid.cIdx(i, j, k);
                    float press = p[cIdx];

                    if (i > 0) {
                        int cLeft = grid.cIdx(i - 1, j, k);
                        if (grid.type[cIdx] == CellType::Fluid || grid.type[cLeft] == CellType::Fluid) {
                            float dv = dt * (press - p[cLeft]);
                            dv       = std::max(-maxDeltaV, std::min(maxDeltaV, dv));
                            grid.u[grid.uIdx(i, j, k)] -= dv;
                        }
                    }

                    if (j > 0) {
                        int cDown = grid.cIdx(i, j - 1, k);
                        if (grid.type[cDown] == CellType::Fluid || grid.type[cIdx] == CellType::Fluid) {
                            float dv = dt * (press - p[cDown]);
                            dv       = std::max(-maxDeltaV, std::min(maxDeltaV, dv));
                            grid.v[grid.vIdx(i, j, k)] -= dv;
                        }
                    }

                    if (k > 0) {
                        int cBack  = grid.cIdx(i, j, k - 1);
                        if (grid.type[cBack] == CellType::Fluid || grid.type[cIdx] == CellType::Fluid) {
                            float dv = dt * (press - p[cBack]);
                            dv       = std::max(-maxDeltaV, std::min(maxDeltaV, dv));
                            grid.w[grid.wIdx(i, j, k)] -= dv;
                        }
                    }
                }
            }
        }
    }
}