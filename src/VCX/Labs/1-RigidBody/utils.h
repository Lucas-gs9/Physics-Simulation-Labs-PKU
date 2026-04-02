#pragma once
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/1-RigidBody/RigidBody.h"
#include <limits>
#include <fcl/narrowphase/collision.h>

namespace VCX::Labs::RigidBody {

    struct RaycastResult {
        bool      hit = false;
        glm::vec3 hitPoint;
        int       bodyId = -1;
        float     t      = std::numeric_limits<float>::max();
    };

    class Renderer {
    public:
        glm::vec3                                        roomColor { 65.0f / 255, 63.0f / 255, 65.0f / 255 };
        glm::vec3                                        objColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };

        Engine::GL::UniqueProgram program;
        Engine::GL::UniqueRenderFrame                    frame;
        std::vector<Engine::GL::UniqueIndexedRenderItem> lineItemList;
        std::vector<Engine::GL::UniqueIndexedRenderItem> objItemList;
        Engine::Camera                                   camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager                       cameraManager;

        Renderer();
        void RenderBody(const RigidBody & body);
        void Refresh(const std::vector<std::shared_ptr<RigidBody>> & bodies, std::pair<std::uint32_t, std::uint32_t> windowSize);
        void ControlCamera(ImVec2 const & pos);
    };

    glm::mat3 skew_mat(glm::vec3 v);

    glm::mat3 get_K(const RigidBody & body, glm::vec3 r);

    std::optional<glm::vec3> GetRayBodyIntersection(ImVec2 const & mousePos, Engine::Camera const & camera, std::pair<uint32_t, uint32_t> windowSize, const RigidBody & body);

    RaycastResult GetNearestBody(ImVec2 const & mousePos, Engine::Camera const & camera, std::pair<uint32_t, uint32_t> windowSize, const std::vector<std::shared_ptr<RigidBody>> & bodies);

    std::shared_ptr<fcl::CollisionGeometryf> CreateFCLGeometry(std::shared_ptr<Shape> shape);

    fcl::Transform3f GetFCLTransform(const RigidBody & body);

    void SyncMeshWithShape(Shape::Type shapeType, Engine::GL::UniqueIndexedRenderItem & lineItem, Engine::GL::UniqueIndexedRenderItem & objItem);

    void DrawBody(const RigidBody & body, Engine::GL::UniqueIndexedRenderItem & objItem, Engine::GL::UniqueIndexedRenderItem & lineItem, Engine::GL::UniqueProgram & program, const glm::vec3 & color);
}