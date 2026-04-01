#include "Labs/1-RigidBody/utils.h"

namespace VCX::Labs::RigidBody {
    glm::mat3 skew_mat(glm::vec3 v) {
        return glm::mat3(0, v.z, -v.y, -v.z, 0, v.x, v.y, -v.x, 0);
    }

    glm::mat3 get_K(const RigidBody& body, glm::vec3 r) {
        if (body.isStatic) return glm::mat3(0.0f);
        glm::mat3 R = glm::mat3_cast(body.q);
        glm::mat3 I_inv = R * body.I_ref_inv * glm::transpose(R);

        glm::mat3 r_skew = skew_mat(r);
        return (1 / body.m) * glm::mat3(1.f) - r_skew * I_inv * r_skew;
    }

    std::optional<glm::vec3> GetRayBoxIntersection(ImVec2 const & mousePos, Engine::Camera const & camera, std::pair<uint32_t, uint32_t> windowSize, const RigidBody & body) {
        if (body.isStatic) return std::nullopt;

        float x = (2.0f * mousePos.x) / windowSize.first - 1.0f;
        float y = 1.0f - (2.0f * mousePos.y) / windowSize.second;

        glm::mat4 invProjView = glm::inverse(camera.GetProjectionMatrix(float(windowSize.first) / windowSize.second) * camera.GetViewMatrix());
        glm::vec4 nearPt      = invProjView * glm::vec4(x, y, -1.0f, 1.0f);
        glm::vec4 farPt       = invProjView * glm::vec4(x, y, 1.0f, 1.0f);
        nearPt /= nearPt.w;
        farPt /= farPt.w;

        glm::vec3 rayOrigin = glm::vec3(nearPt);
        glm::vec3 rayDir    = glm::normalize(glm::vec3(farPt) - rayOrigin);

        glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), body.x) * glm::mat4_cast(body.q);
        glm::mat4 invModel    = glm::inverse(modelMatrix);

        glm::vec3 locOrigin = glm::vec3(invModel * glm::vec4(rayOrigin, 1.0f));
        glm::vec3 locDir    = glm::normalize(glm::vec3(invModel * glm::vec4(rayDir, 0.0f)));

        glm::vec3 boxMin = -std::static_pointer_cast<BoxShape>(body.shape)->dim * 0.5f;
        glm::vec3 boxMax = std::static_pointer_cast<BoxShape>(body.shape)->dim * 0.5f;

        float tMin = (boxMin.x - locOrigin.x) / locDir.x;
        float tMax = (boxMax.x - locOrigin.x) / locDir.x;
        if (tMin > tMax) std::swap(tMin, tMax);

        float tyMin = (boxMin.y - locOrigin.y) / locDir.y;
        float tyMax = (boxMax.y - locOrigin.y) / locDir.y;
        if (tyMin > tyMax) std::swap(tyMin, tyMax);

        if ((tMin > tyMax) || (tyMin > tMax)) return std::nullopt;
        if (tyMin > tMin) tMin = tyMin;
        if (tyMax < tMax) tMax = tyMax;

        float tzMin = (boxMin.z - locOrigin.z) / locDir.z;
        float tzMax = (boxMax.z - locOrigin.z) / tzMin;
        tzMin       = (boxMin.z - locOrigin.z) / locDir.z;
        tzMax       = (boxMax.z - locOrigin.z) / locDir.z;
        if (tzMin > tzMax) std::swap(tzMin, tzMax);

        if ((tMin > tzMax) || (tzMin > tMax)) return std::nullopt;
        if (tzMin > tMin) tMin = tzMin;
        if (tzMax < tMax) tMax = tzMax;

        if (tMin < 0) return std::nullopt;

        return rayOrigin + rayDir * tMin;
    }

    RaycastResult GetNearestBody(ImVec2 const& mousePos, Engine::Camera const& camera, std::pair<uint32_t, uint32_t> windowSize, const std::vector<std::shared_ptr<RigidBody>>& bodies) {
        RaycastResult closestResult;
        
        int n = bodies.size();
        for (int i = 0; i < n; ++i) {
            auto const & body = bodies[i];
            if (body->isStatic) continue;

            auto intersection = GetRayBoxIntersection(mousePos, camera, windowSize, *body);

            glm::mat4 invView        = glm::inverse(camera.GetViewMatrix());
            glm::vec3 cameraPosition = glm::vec3(invView[3]);

            if (intersection.has_value()) {
                float currentT = glm::distance(cameraPosition, intersection.value());
                if (currentT < closestResult.t) {
                    closestResult.hit      = true;
                    closestResult.hitPoint = intersection.value();
                    closestResult.bodyId   = static_cast<int>(i);
                    closestResult.t        = currentT;
                }
            }
        }
        return closestResult;
    }
}