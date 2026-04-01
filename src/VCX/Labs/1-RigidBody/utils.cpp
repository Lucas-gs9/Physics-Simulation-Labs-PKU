#include "Labs/1-RigidBody/utils.h"

namespace VCX::Labs::RigidBody {
    glm::mat3 skew_mat(glm::vec3 v) {
        return glm::mat3(0, v.z, -v.y, -v.z, 0, v.x, v.y, -v.x, 0);
    }

    glm::mat3 get_K(const RigidBody& body, glm::vec3 r) {
        if (body.m > 1e6f) return glm::mat3(0.0f);
        glm::mat3 R = glm::mat3_cast(body.q);
        glm::mat3 I_inv = R * body.I_ref_inv * glm::transpose(R);

        glm::mat3 r_skew = skew_mat(r);
        return (1 / body.m) * glm::mat3(1.f) - r_skew * I_inv * r_skew;
    }
}