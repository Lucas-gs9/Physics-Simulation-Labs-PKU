#pragma once

#include "Labs/1-RigidBody/RigidBody.h"
#include "Labs/1-RigidBody/utils.h"
#include <vector>
#include <utility>
#include <algorithm>
#include <Eigen/Dense>

namespace VCX::Labs::RigidBody {

    struct ContactInfo {
        int                    id1, id2;
        glm::vec3              c_pos { 0.f };
        glm::vec3              c_normal { 0.f };
        float                  depth = 0.f;
        float                  weight;
        std::vector<glm::vec3> pos_list;
        ContactInfo(int i1, int i2, float w):
            id1(i1), id2(i2), weight(w) {}
    };

    class RigidBodySystem {
    public:
        std::vector<std::shared_ptr<RigidBody>> bodies;
        std::vector<ContactInfo>                contacts;
        float                                   muN = 0.7f;
        float                                   muT = 0.6f;

        bool  enableGravity = false;
        float g             = 9.8f;

        void Clear();
        void AddBody(std::shared_ptr<RigidBody> body);
        void Update(float dt);

    private:
        void CollisionDetect(int id1, int id2);
        glm::vec3 GetCollisionImpulse(const ContactInfo & contact, float dt);
    };
}