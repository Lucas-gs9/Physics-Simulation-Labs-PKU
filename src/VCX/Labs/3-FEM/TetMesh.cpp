#include "TetMesh.h"
#include <map>
#include <algorithm>
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

    void TetMesh::computeForces(ParticleSystem& ps) {
        for (auto& e : elements) {
            glm::vec3 x0 = ps.x[e.indices[0]];
            glm::vec3 x1 = ps.x[e.indices[1]];
            glm::vec3 x2 = ps.x[e.indices[2]];
            glm::vec3 x3 = ps.x[e.indices[3]];

            glm::mat3 current_E(x1 - x0, x2 - x0, x3 - x0);

            //Deformation gradient
            glm::mat3 F = current_E * e.inv_E;
            glm::mat3 P = calculateStressP(F);

            glm::mat3 forces = -e.V_ref * P * glm::transpose(e.inv_E);

            ps.addForce(e.indices[1], forces[0]);
            ps.addForce(e.indices[2], forces[1]);
            ps.addForce(e.indices[3], forces[2]);
        }
    }

    glm::mat3 TetMesh::calculateStressP(const glm::mat3& F) {
        //Green Strain
        glm::mat3 G = 0.5f * (glm::transpose(F) * F - glm::mat3(1.0f));
        //trace(G)
        float trG = G[0][0] + G[1][1] + G[2][2];
        //First PK Stress
        glm::mat3 S = 2.0f * mu * G + lambda * trG * glm::mat3(1.0f);
        return F * S;
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