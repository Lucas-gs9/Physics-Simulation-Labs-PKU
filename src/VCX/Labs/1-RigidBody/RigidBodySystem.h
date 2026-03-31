#pragma once

#include "Labs/1-RigidBody/RigidBody.h"
#include <fcl/narrowphase/collision.h>
#include <vector>

namespace VCX::Labs::RigidBody {
    class RigidBodySystem {
        std::vector<std::shared_ptr<RigidBody>> Bodies;

        void Clear();
        void AddBody(std::shared_ptr<RigidBody> body);
        void Update(float dt);
    };
}