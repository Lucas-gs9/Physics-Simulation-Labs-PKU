#include "TetMesh.h"
#include <map>
#include <algorithm>
#include "utils.h"
namespace VCX::Labs::FEM {
    void TetMesh::initialize(ParticleSystem& ps, int nx, int ny, int nz, float delta) {
        int n  = (nx + 1) * (ny + 1) * (nz + 1);
        ps.resize(n);
        int id = 0;
        for (int i = 0; i <= nx; i++) {
            for (int j = 0; j <= ny; j++) {
                for (int k = 0; k <= nz; k++) {
                    glm::vec3 pos(i * delta, j * delta, k * delta);
                    bool      fixed = (i == 0);
                    ps.setParticle(id, pos, fixed);
                    ++id;
                }
            }
        }

        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {
                for (int k = 0; k < nz; k++) {
                    int v0 = ps.getID(i, j, k, ny, nz);
                    int v1 = ps.getID(i + 1, j, k, ny, nz);
                    int v2 = ps.getID(i + 1, j + 1, k, ny, nz);
                    int v3 = ps.getID(i, j + 1, k, ny, nz);
                    int v4 = ps.getID(i, j, k + 1, ny, nz);
                    int v5 = ps.getID(i + 1, j, k + 1, ny, nz);
                    int v6 = ps.getID(i + 1, j + 1, k + 1, ny, nz);
                    int v7 = ps.getID(i, j + 1, k + 1, ny, nz);

                    addTet(ps, v0, v1, v2, v6);
                    addTet(ps, v0, v2, v3, v6);
                    addTet(ps, v0, v3, v7, v6);
                    addTet(ps, v0, v7, v4, v6);
                    addTet(ps, v0, v4, v5, v6);
                    addTet(ps, v0, v5, v1, v6);
                }
            }
        }

        ps.updateMass();

        extractSurface();
    }

    void TetMesh::reset() {
#pragma omp parallel for
        for (auto& e : elements) {
            e.Fp = glm::mat3(1.0f);
            e.F_prev = glm::mat3(1.0f);
        }
    }

    void TetMesh::addTet(ParticleSystem& ps, int i0, int i1, int i2, int i3) {
        Element tet;
        tet.indices[0] = i0;
        tet.indices[1] = i1;
        tet.indices[2] = i2;
        tet.indices[3] = i3;

        glm::vec3 X0 = ps.x[i0], X1 = ps.x[i1], X2 = ps.x[i2], X3 = ps.x[i3];

        glm::mat3 E = glm::mat3(X1 - X0, X2 - X0, X3 - X0);

        tet.inv_E = glm::inverse(E);
        tet.V_ref = std::abs(glm::determinant(E)) / 6.0f;

        float totalMass = rho * tet.V_ref;
        float nodeMass  = totalMass / 4.0f;

        for (int i = 0; i < 4; ++i) {
            ps.addMass(tet.indices[i], nodeMass);
        }

        elements.push_back(tet);
    }

    void TetMesh::computeForces(ParticleSystem & ps, float dt, int model, bool elastoplastic) {
#pragma omp parallel for
        for (auto& e : elements) {
            glm::vec3 x0 = ps.x[e.indices[0]];
            glm::vec3 x1 = ps.x[e.indices[1]];
            glm::vec3 x2 = ps.x[e.indices[2]];
            glm::vec3 x3 = ps.x[e.indices[3]];

            glm::mat3 current_E(x1 - x0, x2 - x0, x3 - x0);

            //Deformation gradient
            glm::mat3 F = current_E * e.inv_E;
            glm::mat3 P;

            if (elastoplastic) {
                glm::mat3 Fe = F * glm::inverse(e.Fp);
                glm::mat3 U, V;
                glm::vec3 sigma;
                svd(Fe, U, sigma, V);
                glm::vec3 sigma_new;
                for (int i = 0; i < 3; i++) {
                    if (sigma[i] < 0) {
                        sigma_new[i] = -std::clamp(std::abs(sigma[i]), 0.8f, 1.2f);
                    } else {
                        sigma_new[i] = std::clamp(sigma[i], 0.8f, 1.2f);
                    }
                }
                glm::mat3 Sigma_mat = glm::mat3(
                    sigma_new[0], 0.0f, 0.0f, 0.0f, sigma_new[1], 0.0f, 0.0f, 0.0f, sigma_new[2]);
                Fe            = U * Sigma_mat * glm::transpose(V);
                glm::mat3 F_dot = (F - e.F_prev) / dt;

                glm::mat3 Fe_inv = getInverseFromSVD(U, sigma_new, V);

                e.Fp                = Fe_inv * F;
                glm::mat3 P_elastic = calculateStressP(Fe, model);
                P         = P_elastic + eta * F_dot;
            }
            else {
                P = calculateStressP(F, model);
            }

            e.F_prev = F;


            glm::mat3 forces = -e.V_ref * P * glm::transpose(e.inv_E);
            glm::vec3 f1     = forces[0];
            glm::vec3 f2     = forces[1];
            glm::vec3 f3     = forces[2];
            glm::vec3 f0     = -(f1 + f2 + f3);

            ps.addForce(e.indices[0], f0); 
            ps.addForce(e.indices[1], f1);
            ps.addForce(e.indices[2], f2);
            ps.addForce(e.indices[3], f3);
        }
    }

    glm::mat3 TetMesh::calculateStressP(const glm::mat3 & F, int model) {
        glm::mat3 P;

        switch (model) {
        case 1: {
            float     J     = glm::determinant(F);
            float     safeJ = std::max(J, 0.001f);
            glm::mat3 invFT = glm::transpose(glm::inverse(F));
            P               = mu * (F - invFT) + lambda * std::log(safeJ) * invFT;
            break;
        }
        case 2:{
            glm::mat3 U, V;
            glm::vec3 sigma;
            svd(F, U, sigma, V);
            glm::mat3 R = U * glm::transpose(V);
            glm::mat3 S = glm::transpose(R) * F;
            glm::mat3 I(1.f);
            float     trS = S[0][0] + S[1][1] + S[2][2];
            P             = R * (2.f * mu * (S - I) + lambda * (trS - 3) * I);
            break;
        }
        default: {
            glm::mat3 FTF = glm::transpose(F) * F;
            float     trG = 0.5f * (FTF[0][0] + FTF[1][1] + FTF[2][2] - 3.0f);
            glm::mat3 S   = mu * (FTF - glm::mat3(1.0f)) + lambda * trG * glm::mat3(1.0f);
            P           = F * S;
            break;
        }
        }

        return P;
    }

    void TetMesh::extractSurface() {
        surfaceTriangles.clear();

        std::map<std::vector<int>, int> faceCounts;

        for (const auto & tet : elements) {
            int faces[4][3] = {
                {tet.indices[0], tet.indices[1], tet.indices[2]},
                {tet.indices[0], tet.indices[1], tet.indices[3]},
                {tet.indices[0], tet.indices[2], tet.indices[3]},
                {tet.indices[1], tet.indices[2], tet.indices[3]}
            };
            for (auto & f : faces) {
                std::vector<int> face = { f[0], f[1], f[2] };
                std::sort(face.begin(), face.end());
                faceCounts[face]++;
            }
        }

        for (auto const & [face, count] : faceCounts) {
            if (count == 1) {
                surfaceTriangles.push_back(face[0]);
                surfaceTriangles.push_back(face[1]);
                surfaceTriangles.push_back(face[2]);
            }
        }
    }

}