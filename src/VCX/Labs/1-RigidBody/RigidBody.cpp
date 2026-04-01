#include "Labs/1-RigidBody/RigidBody.h"

namespace VCX::Labs::RigidBody {
    RigidBody::RigidBody(float mass, std::shared_ptr<Shape> s):
        m(mass), shape(s) {
        I_ref     = s->GetInertia(m);
        I_ref_inv = glm::inverse(I_ref);
    }

    void RigidBody::Update(float dt) {
        v += f * dt / m;
        x += v * dt;

        glm::mat3 R = glm::mat3_cast(q);
        L += torque * dt;
        w = R * I_ref_inv * glm::transpose(R) * L;
        q += 0.5f * dt * glm::quat(0, w.x, w.y, w.z) * q;
        q      = glm::normalize(q);
        f      = glm::vec3(0.f);
        torque = glm::vec3(0.f);
    }

    void RigidBody::AddForce(glm::vec3 const& force, glm::vec3 const& f_point) {
        f += force;
        glm::vec3 r = f_point - x;
        torque += glm::cross(r, force);
    }

    void RigidBody::Reset(glm::vec3 const& v0) {
        x = { 0.f,0.f,0.f };
        q = { 1.f, 0.f, 0.f, 0.f };
        v = v0;
        L = { 0.f, 0.f, 0.f };
        f = { 0.f, 0.f, 0.f };
        torque = { 0.f, 0.f, 0.f };
        w      = { 0.f, 0.f, 0.f };
    }

    void RigidBody::ApplyImpulse(glm::vec3 const& impulse, glm::vec3 const& i_point) {
        v += impulse / m;
        glm::vec3 r     = i_point - x;
        glm::mat3 R = glm::mat3_cast(q);
        glm::mat3 I_inv = R * I_ref_inv * glm::transpose(R);
        w += I_inv * glm::cross(r, impulse);
        glm::mat3 I = R * I_ref * glm::transpose(R);
        L           = I * w;
    }
}