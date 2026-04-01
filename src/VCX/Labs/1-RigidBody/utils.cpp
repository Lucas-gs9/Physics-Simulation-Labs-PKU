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

    std::optional<glm::vec3> GetRayBodyIntersection(ImVec2 const & mousePos, Engine::Camera const & camera, std::pair<uint32_t, uint32_t> windowSize, const RigidBody & body) {
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

        if (body.shape->GetType() == Shape::Type::Sphere) {
            auto sphere = std::static_pointer_cast<SphereShape>(body.shape);

            glm::vec3 oc = rayOrigin - body.x;
            float     b  = glm::dot(oc, rayDir);
            float     c  = glm::dot(oc, oc) - sphere->radius * sphere->radius;

            float discriminant = b * b - c;

            if (discriminant < 0) return std::nullopt;

            float t = -b - std::sqrt(discriminant);

            if (t < 0) {
                t = -b + std::sqrt(discriminant);
            }

            if (t < 0) return std::nullopt;

            return rayOrigin + rayDir * t;
        } else if (body.shape->GetType() == Shape::Type::Box) {
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
            float tzMax = (boxMax.z - locOrigin.z) / locDir.z;
            if (tzMin > tzMax) std::swap(tzMin, tzMax);

            if ((tMin > tzMax) || (tzMin > tMax)) return std::nullopt;
            if (tzMin > tMin) tMin = tzMin;
            if (tzMax < tMax) tMax = tzMax;

            if (tMin < 0) return std::nullopt;

            return rayOrigin + rayDir * tMin;
        }
        return std::nullopt;
    }

    RaycastResult GetNearestBody(ImVec2 const& mousePos, Engine::Camera const& camera, std::pair<uint32_t, uint32_t> windowSize, const std::vector<std::shared_ptr<RigidBody>>& bodies) {
        RaycastResult closestResult;
        
        int n = bodies.size();
        for (int i = 0; i < n; ++i) {
            auto const & body = bodies[i];
            if (body->isStatic) continue;

            auto intersection = GetRayBodyIntersection(mousePos, camera, windowSize, *body);

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

    std::shared_ptr<fcl::CollisionGeometryf> CreateFCLGeometry(std::shared_ptr<Shape> shape) {
        if (shape->GetType() == Shape::Type::Box) {
            auto s = std::static_pointer_cast<BoxShape>(shape);
            return std::make_shared<fcl::Boxf>(s->dim.x, s->dim.y, s->dim.z);
        } else if (shape->GetType() == Shape::Type::Sphere) {
            auto s = std::static_pointer_cast<SphereShape>(shape);
            return std::make_shared<fcl::Spheref>(s->radius);
        }
        return nullptr;
    }

    fcl::Transform3f GetFCLTransform(const RigidBody& body) {
        const glm::vec3 & p = body.x;
        const glm::quat & q = body.q;
        return fcl::Transform3f(Eigen::Translation3f(p.x, p.y, p.z) * Eigen::Quaternionf(q.w, q.x, q.y, q.z));
    }

    void SyncMeshWithShape(std::shared_ptr<Shape> const& shape, Engine::GL::UniqueIndexedRenderItem& lineItem, Engine::GL::UniqueIndexedRenderItem& objItem) {
        std::vector<std::uint32_t> line_index;
        std::vector<std::uint32_t> tri_index;

        if (shape->GetType() == Shape::Type::Box) {
            auto      box = std::static_pointer_cast<BoxShape>(shape);
            glm::vec3 d   = box->dim * 0.5f;
            line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 };
            tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        } else if (shape->GetType() == Shape::Type::Sphere) {
            int sectors = 32, stacks = 16;
            for (int i = 0; i < stacks; ++i) {
                int k1 = i * (sectors + 1);
                int k2 = k1 + sectors + 1;
                for (int j = 0; j < sectors; ++j, ++k1, ++k2) {
                    if (i != 0) {
                        tri_index.push_back(k1);
                        tri_index.push_back(k2);
                        tri_index.push_back(k1 + 1);
                    }
                    if (i != (stacks - 1)) {
                        tri_index.push_back(k1 + 1);
                        tri_index.push_back(k2);
                        tri_index.push_back(k2 + 1);
                    }
                    line_index.push_back(k1);
                    line_index.push_back(k2);
                    if (i != 0) {
                        line_index.push_back(k1);
                        line_index.push_back(k1 + 1);
                    }
                }
            }
        }

        lineItem.UpdateElementBuffer(line_index);
        objItem.UpdateElementBuffer(tri_index);
    }

    void DrawBody(std::shared_ptr<RigidBody> const& body, Engine::GL::UniqueIndexedRenderItem& objItem, Engine::GL::UniqueIndexedRenderItem& lineItem, Engine::GL::UniqueProgram& program, const glm::vec3& color) {
        std::vector<glm::vec3> vertices;

        glm::vec3 x = body->x;
        glm::mat3 R = glm::mat3_cast(body->q);

        if (body->shape->GetType() == Shape::Type::Box) {
            auto      box = std::static_pointer_cast<BoxShape>(body->shape);
            glm::vec3 h   = box->dim * 0.5f;
            vertices      = {
                x + R * glm::vec3(-h.x, h.y, h.z),
                x + R * glm::vec3(h.x, h.y, h.z),
                x + R * glm::vec3(h.x, h.y, -h.z),
                x + R * glm::vec3(-h.x, h.y, -h.z),
                x + R * glm::vec3(-h.x, -h.y, h.z),
                x + R * glm::vec3(h.x, -h.y, h.z),
                x + R * glm::vec3(h.x, -h.y, -h.z),
                x + R * glm::vec3(-h.x, -h.y, -h.z)
            };
        } 
        else if (body->shape->GetType() == Shape::Type::Sphere) {
            auto  sphere  = std::static_pointer_cast<SphereShape>(body->shape);
            float r       = sphere->radius;
            int   sectors = 32, stacks = 16;
            for (int i = 0; i <= stacks; ++i) {
                float stackAngle = glm::half_pi<float>() - i * glm::pi<float>() / stacks;
                float xy         = r * cosf(stackAngle);
                float z          = r * sinf(stackAngle);
                for (int j = 0; j <= sectors; ++j) {
                    float     sectorAngle = j * 2 * glm::pi<float>() / sectors;
                    glm::vec3 localPos(xy * cosf(sectorAngle), xy * sinf(sectorAngle), z);
                    vertices.push_back(x + R * localPos);
                }
            }
        }
        auto span_bytes = Engine::make_span_bytes<glm::vec3>(vertices);
        program.GetUniforms().SetByName("u_Color", color);
        objItem.UpdateVertexBuffer("position", span_bytes);
        objItem.Draw({ program.Use() });

        program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        lineItem.UpdateVertexBuffer("position", span_bytes);
        lineItem.Draw({ program.Use() });
    }
}