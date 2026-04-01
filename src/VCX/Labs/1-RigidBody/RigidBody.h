#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ImageRGB.h"
#include <memory>

namespace VCX::Labs::RigidBody {
    class Shape {
    public:
        enum class Type { Box,
                          Sphere };

        virtual ~Shape()             = default;
        virtual Type GetType() const = 0;

        virtual glm::mat3 GetInertia(float mass) const = 0;
    };

    class BoxShape : public Shape {
    public:
        glm::vec3 dim;

        BoxShape(glm::vec3 d):
            dim(d) {}

        Type GetType() const override { return Type::Box; }

        glm::mat3 GetInertia(float mass) const override {
            float ix = (1.0f / 12.0f) * mass * (dim.y * dim.y + dim.z * dim.z);
            float iy = (1.0f / 12.0f) * mass * (dim.x * dim.x + dim.z * dim.z);
            float iz = (1.0f / 12.0f) * mass * (dim.x * dim.x + dim.y * dim.y);
            return glm::mat3(ix, 0, 0, 0, iy, 0, 0, 0, iz);
        }
    };

    class RigidBody {
    public:
        float                  m { 1.f };
        std::shared_ptr<Shape> shape;
        glm::mat3              I_ref; // Inertia
        glm::mat3              I_ref_inv;
        bool                   isStatic = false;

        glm::vec3 x { 0.f };                // position
        glm::quat q { 1.f, 0.f, 0.f, 0.f }; // orientation

        glm::vec3 v { 0.f }; // velocity
        glm::vec3 w { 0.f }; // angular velocity
        glm::vec3 L { 0.f }; // angular momentum

        glm::vec3 f { 0.f }; // force
        glm::vec3 torque { 0.f };

        float damp_v       = 0.999f;
        float damp_w       = 0.999f;

        RigidBody(float mass, std::shared_ptr<Shape> s);
        void Update(float dt);
        void AddForce(glm::vec3 const & force, glm::vec3 const & f_point);
        void Reset(glm::vec3 const & v0);
        void ApplyImpulse(glm::vec3 const & impulse, glm::vec3 const & i_point);
    };
} // namespace VCX::Labs::RigidBody