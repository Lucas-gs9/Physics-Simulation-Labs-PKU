#include "utils.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace VCX::Labs::FEM {
    Ray getMouseRay(ImVec2 const & mousePos, Engine::Camera const & camera, std::pair<uint32_t, uint32_t> windowSize) {
        float x = (2.0f * mousePos.x) / windowSize.first - 1.0f;
        float y = 1.0f - (2.0f * mousePos.y) / windowSize.second;

        glm::mat4 invProjView = glm::inverse(camera.GetProjectionMatrix(float(windowSize.first) / windowSize.second) * camera.GetViewMatrix());
        glm::vec4 nearPt      = invProjView * glm::vec4(x, y, -1.0f, 1.0f);
        glm::vec4 farPt       = invProjView * glm::vec4(x, y, 1.0f, 1.0f);
        nearPt /= nearPt.w;
        farPt /= farPt.w;

        return { glm::vec3(nearPt), glm::normalize(glm::vec3(farPt) - glm::vec3(nearPt)) };
    }

    bool getSelectedId(ImVec2 const & mousePos, Engine::Camera const & camera, std::pair<uint32_t, uint32_t> windowSize, ParticleSystem & ps, int & selectedId) {
        Ray ray = getMouseRay(mousePos, camera, windowSize);

        int   globalClosestId = -1;
        float globalMinDist   = 0.5f;

#pragma omp parallel 
        {
        float localMinDist = globalMinDist;
        int   localId      = -1;

        #pragma omp for
        for (int i = 0; i < ps.size; ++i) {
            glm::vec3 relPos     = ps.x[i] - ray.origin;
            float     projection = glm::dot(relPos, ray.direction);

            if (projection < 0) continue;

            float d = glm::length(relPos - projection * ray.direction);

            if (d < localMinDist) {
                localMinDist = d;
                localId      = i;
            }
        }

        if (localId != -1) {
#pragma omp critical
            {
                if (localMinDist < globalMinDist) {
                    globalMinDist   = localMinDist;
                    globalClosestId = localId;
                }
            }
        }
        }

        if (globalClosestId != -1) {
            selectedId = globalClosestId;
            return true;
        }
        return false;
    }

    void svd(const glm::mat3& F, glm::mat3& U, glm::vec3& sigma, glm::mat3& V) {
        Eigen::Map<const Eigen::Matrix3f> m(glm::value_ptr(F));

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix3f U_e = svd.matrixU();
        Eigen::Matrix3f V_e = svd.matrixV();
        Eigen::Vector3f S_e = svd.singularValues();

        if (U_e.determinant() * V_e.determinant() < 0) {
            U_e.col(2) *= -1.0f;
            S_e(2) *= -1.0f;
        }

        Eigen::Map<Eigen::Matrix3f>(glm::value_ptr(U)) = U_e;
        Eigen::Map<Eigen::Matrix3f>(glm::value_ptr(V)) = V_e;

        sigma = glm::vec3(S_e(0), S_e(1), S_e(2));
    }

    glm::mat3 getInverseFromSVD(const glm::mat3& U, const glm::vec3& sigma, const glm::mat3& V) {
        float s0 = (std::abs(sigma[0]) > 1e-9f) ? 1.0f / sigma[0] : 0.0f;
        float s1 = (std::abs(sigma[1]) > 1e-9f) ? 1.0f / sigma[1] : 0.0f;
        float s2 = (std::abs(sigma[2]) > 1e-9f) ? 1.0f / sigma[2] : 0.0f;

        glm::mat3 Sigma_inv = glm::mat3(0.0f);
        Sigma_inv[0][0]     = s0;
        Sigma_inv[1][1]     = s1;
        Sigma_inv[2][2]     = s2;

        return V * Sigma_inv * glm::transpose(U);
    }
}