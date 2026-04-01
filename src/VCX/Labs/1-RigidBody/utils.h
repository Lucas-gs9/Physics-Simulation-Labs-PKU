#pragma once
#include <Labs/1-RigidBody/RigidBody.h>

namespace VCX::Labs::RigidBody {
    glm::mat3 skew_mat(glm::vec3 v);

    glm::mat3 get_K(const RigidBody & body, glm::vec3 r);
}