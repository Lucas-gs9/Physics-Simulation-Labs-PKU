#pragma once
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/1-RigidBody/RigidBody.h"
#include <limits>

namespace VCX::Labs::RigidBody {

    struct RaycastResult {
        bool      hit = false;
        glm::vec3 hitPoint;
        int       bodyId = -1;
        float     t      = std::numeric_limits<float>::max();
    };

    glm::mat3 skew_mat(glm::vec3 v);

    glm::mat3 get_K(const RigidBody & body, glm::vec3 r);

    std::optional<glm::vec3> GetRayBoxIntersection(ImVec2 const & mousePos, Engine::Camera const & camera, std::pair<uint32_t, uint32_t> windowSize, const RigidBody & body);

    RaycastResult GetNearestBody(ImVec2 const & mousePos, Engine::Camera const & camera, std::pair<uint32_t, uint32_t> windowSize, const std::vector<std::shared_ptr<RigidBody>> & bodies);
}