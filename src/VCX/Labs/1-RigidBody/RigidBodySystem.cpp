#include "Labs/1-RigidBody/RigidBodySystem.h"

namespace VCX::Labs::RigidBody {
    void RigidBodySystem::Clear() {
        bodies.clear();
        contacts.clear();
    }

    void RigidBodySystem::AddBody(std::shared_ptr<RigidBody> body) {
        bodies.push_back(std::move(body));
    }

    void RigidBodySystem::Update(float dt) {

        if (enableGravity) {
            for (auto const & body : bodies) {
                body->AddForce(body->m * glm::vec3(0.f, -g, 0.f), body->x);
            }
        }

        contacts.clear();
        int n = bodies.size();
        for (int i = 0; i < n; ++i) 
            for (int j = i + 1; j < n; ++j) {
                CollisionDetect(i, j);
            }

        const int numIterations = 25;

        for (int i = 0; i < numIterations; ++i) {
            std::vector<glm::vec3> impulse_list;
            for (auto const & contact : contacts) {
                impulse_list.push_back(GetCollisionImpulse(contact, dt));
            }
            for (int k = 0; k < contacts.size(); ++k) {
                auto const & contact = contacts[k];
                glm::vec3    J       = impulse_list[k];
                for (auto const & pos : contact.pos_list) {
                    bodies[contact.id1]->ApplyImpulse(-J, contact.c_pos);
                    bodies[contact.id2]->ApplyImpulse(J, contact.c_pos);
                }
            }
        }
        for (auto & body : bodies) {
            if (enableDamp)
                body->Update(dt);
            else
                body->Update(dt, 1.f, 1.f);
        }
    }

    void RigidBodySystem::CollisionDetect(int id1, int id2) {
        auto const & b0 = *bodies[id1];
        auto const & b1 = *bodies[id2];

        using CollisionGeometryPtr_t          = std::shared_ptr<fcl::CollisionGeometryf>;
        CollisionGeometryPtr_t geometry_A = CreateFCLGeometry(b0.shape);
        CollisionGeometryPtr_t geometry_B = CreateFCLGeometry(b1.shape);

        fcl::CollisionObject<float>  obj_A(geometry_A, GetFCLTransform(b0));
        fcl::CollisionObject<float>  obj_B(geometry_B, GetFCLTransform(b1));

        fcl::CollisionRequest<float> collisionRequest(8, true);
        fcl::CollisionResult<float>  collisionResult;
        fcl::collide(&obj_A, &obj_B, collisionRequest, collisionResult);
        if (! collisionResult.isCollision()) return;
        std::vector<fcl::Contact<float>> f_contacts;
        collisionResult.getContacts(f_contacts);
        float weight = 1.0f / static_cast<float>(f_contacts.size());

        ContactInfo cur_contact { id1, id2, weight };
        for (auto const & contact : f_contacts) {
            cur_contact.pos_list.push_back({ contact.pos[0], contact.pos[1], contact.pos[2] });
            cur_contact.c_pos += glm::vec3 { contact.pos[0], contact.pos[1], contact.pos[2] };
            cur_contact.c_normal += glm::vec3 { contact.normal[0], contact.normal[1], contact.normal[2] };
            cur_contact.depth = std::max(cur_contact.depth, contact.penetration_depth);
        }
        cur_contact.c_pos *= weight;
        cur_contact.c_normal = glm::normalize(cur_contact.c_normal);
        contacts.push_back(cur_contact);
    }

    glm::vec3 RigidBodySystem::GetCollisionImpulse(const ContactInfo& contact, float dt) {
        RigidBody & body1 = *bodies[contact.id1];
        RigidBody & body2 = *bodies[contact.id2];

        glm::vec3   r1    = contact.c_pos - body1.x;
        glm::vec3   r2    = contact.c_pos - body2.x;
        if (body1.shape->GetType() == Shape::Type::Sphere) r1 = contact.c_normal * (-std::static_pointer_cast<SphereShape>(body1.shape)->radius);
        if (body2.shape->GetType() == Shape::Type::Sphere) r2 = contact.c_normal * (-std::static_pointer_cast<SphereShape>(body1.shape)->radius);
        glm::vec3 v1 = body1.v + glm::cross(body1.w, r1);
        glm::vec3   v2    = body2.v + glm::cross(body2.w, r2);

        glm::vec3 v_rel = v2 - v1;
        float     v_dot_n = glm::dot(v_rel, contact.c_normal);
        if (v_dot_n > 0.f && contact.depth < 0.001f) return { 0.f, 0.f, 0.f };

        float slop = 0.005f;
        float v_bias = (beta / dt) * std::max(0.f, contact.depth - slop);
        v_bias       = std::min(v_bias, 0.5f);

        glm::vec3 v_N = v_dot_n * contact.c_normal;
        glm::vec3 v_T = v_rel - v_N;

        float current_muN = muN;
        if (glm::length(v_rel) < 0.2f) {
            current_muN = 0.0f;
        }
        float a = 1.f;
        float v_N_norm = glm::length(v_N);
        float v_T_norm = glm::length(v_T);
        if (v_T_norm > 1e-6f) {
            a = std::max(1.f - muT * (1.f + current_muN) * v_N_norm / v_T_norm, 0.f);
        }
        if (v_T_norm < 0.01f) a = 0.0f;
        glm::vec3 v_rel_new = a * v_T - current_muN * v_N + v_bias * contact.c_normal;

        glm::mat3 K = get_K(body1, r1) + get_K(body2, r2);
        glm::vec3 J = (glm::inverse(K + glm::mat3(1e-6f)) * (v_rel_new - v_rel)) * contact.weight;
        if (glm::dot(J, contact.c_normal) < 0) return glm::vec3(0.f);
        return J;
    }
}