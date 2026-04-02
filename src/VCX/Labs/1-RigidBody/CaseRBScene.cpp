#include "Labs/1-RigidBody/CaseRBScene.h"

namespace VCX::Labs::RigidBody {
    void CaseRBScene::Reset() {
        _system.Clear();
        _system.enableGravity = true;
        _system.enableDamp    = false;

        float floorSize     = 50.0f;
        float floorThickness   = 0.5f;
        float wallHeight    = 10.0f;
        float wallThickness = 0.5f;

        auto floor      = std::make_shared<RigidBody>(100.0f, std::make_shared<BoxShape>(glm::vec3(floorSize, floorThickness, floorSize)));
        floor->isStatic = true;
        floor->x        = { 0.f, -1.5f, 0.f }; 
        _system.AddBody(floor);

        auto wallShapeLR = std::make_shared<BoxShape>(glm::vec3(wallThickness, wallHeight, floorSize));
        auto wallShapeFB = std::make_shared<BoxShape>(glm::vec3(floorSize, wallHeight, wallThickness));
        float offset      = floorSize * 0.5f + wallThickness * 0.5f;
        float wallPosY    = floor->x.y + wallHeight * 0.5f - floorThickness * 0.5f;

        auto wallL      = std::make_shared<RigidBody>(100.f, wallShapeLR);
        wallL->isStatic = true;
        wallL->x        = { -offset, wallPosY, 0.f };
        _system.AddBody(wallL);

        auto wallR      = std::make_shared<RigidBody>(100.f, wallShapeLR);
        wallR->isStatic = true;
        wallR->x        = { offset, wallPosY, 0.f };
        _system.AddBody(wallR);

        auto wallB      = std::make_shared<RigidBody>(100.f, wallShapeFB);
        wallB->isStatic = true;
        wallB->x        = { 0.f, wallPosY, -offset };
        _system.AddBody(wallB);

        auto wallF      = std::make_shared<RigidBody>(100.f, wallShapeFB);
        wallF->isStatic = true;
        wallF->x        = { 0.f, wallPosY, offset };
        _system.AddBody(wallF);

        auto box1 = std::make_shared<RigidBody>(1.f, std::make_shared<BoxShape>(glm::vec3(1.f, 1.f, 1.f)));
        box1->x   = { 0.f, -0.05f, 0.f }; 
        _system.AddBody(box1);

        auto box2 = std::make_shared<RigidBody>(3.f, std::make_shared<BoxShape>(glm::vec3(0.8f, 0.7f, 0.5f)));
        box2->x   = { 0.f, 1.5f, 0.f };
        _system.AddBody(box2);

        auto box3 = std::make_shared<RigidBody>(5.f, std::make_shared<BoxShape>(glm::vec3(0.8f, 2.f, 0.8f)));
        box3->x   = { 3.f, 4.0f, -5.f };
        box3->v   = { 0.f, -2.0f, 0.f };
        box3->q   = glm::quat(glm::vec3(glm::radians(30.f), glm::radians(60.f), glm::radians(45.f)));
        _system.AddBody(box3);

        float sphereRadius = 1.0f;
        auto  sphere       = std::make_shared<RigidBody>(2.0f, std::make_shared<SphereShape>(sphereRadius));
        sphere->x          = { 5.0f, 4.0f, -2.0f }; 
        sphere->v          = { 2.0f, -1.0f, -1.0f };
        _system.AddBody(sphere);

        float cylRadius = 0.6f;
        float cylHeight = 1.5f;
        auto  cylinder  = std::make_shared<RigidBody>(3.0f, std::make_shared<CylinderShape>(cylRadius, cylHeight));
        cylinder->x     = { 2.0f, 5.0f, 3.0f }; 
        cylinder->q = glm::angleAxis(glm::radians(90.f), glm::vec3(1, 0, 0));
        cylinder->w = { 0.5f, 1.0f, 0.0f };
        glm::mat3 R     = glm::mat3_cast(cylinder->q);
        glm::mat3 I     = R * cylinder->I_ref * glm::transpose(R);
        cylinder->L               = I * cylinder->w;
        _system.AddBody(cylinder);
    }

    CaseRBScene::CaseRBScene(){
        Reset();
    }

    void CaseRBScene::OnSetupPropsUI() {
        ImGui::Checkbox("Move Camera", &_controlCamera);
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) Reset();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            ImGui::SliderFloat("Mouse Force", &_k, 5, 20);
        }
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_renderer.objColor));
            ImGui::ColorEdit3("Room Color", glm::value_ptr(_renderer.roomColor));
        }
    }

    Common::CaseRenderResult CaseRBScene::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped) {

            if (_isDragging) {
                ImVec2 delta = ImGui::GetIO().MouseDelta;

                glm::vec3 mouseDelta(delta.x, delta.y, 0.0f);

                glm::mat4 invView  = glm::inverse(_renderer.camera.GetViewMatrix());
                glm::vec3 camRight = glm::vec3(invView[0]);
                glm::vec3 camUp    = glm::vec3(invView[1]);

                RigidBody & body = *_system.bodies[_selectedBodyId];

                float     d               = 3.f;
                glm::vec3 velocityAtPoint = body.v + glm::cross(body.w, _f_point);
                glm::vec3 forceWorld      = (camRight * mouseDelta.x - camUp * mouseDelta.y) * _k - velocityAtPoint * d;

                glm::mat3 R                     = glm::mat3_cast(body.q);
                glm::vec3 current_f_point_world = body.x + R * _f_point;
                body.AddForce(forceWorld, current_f_point_world);
            }

            float dt = ImGui::GetIO().DeltaTime / 2;
            if (dt > 0.1f) dt = 0.1f;
            _system.Update(dt);
        }
        _windowSize = desiredSize;

         _renderer.Refresh(_system.bodies, _windowSize);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _renderer.frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseRBScene::OnProcessInput(ImVec2 const & pos) {
        if (_controlCamera)
            _renderer.ControlCamera(pos);
        else {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                auto result = GetNearestBody(pos, _renderer.camera, _windowSize, _system.bodies);
                if (result.hit) {
                    _isDragging     = true;
                    _selectedBodyId         = result.bodyId;
                    const RigidBody & body = *_system.bodies[result.bodyId];
                    glm::mat3         R_inv = glm::transpose(glm::mat3_cast(body.q));
                    _f_point                = R_inv * (result.hitPoint - body.x);
                }
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                _isDragging = false;
                _selectedBodyId = -1;
            }
        }
    }
}