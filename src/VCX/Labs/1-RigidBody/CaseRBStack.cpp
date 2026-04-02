#include "Labs/1-RigidBody/CaseRBStack.h"

namespace VCX::Labs::RigidBody {
    CaseRBStack::CaseRBStack() {
        Reset();
    }

    void CaseRBStack::OnSetupPropsUI() {
        ImGui::Checkbox("Move Camera", &_controlCamera);
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) Reset();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            const char * scNames[] = { "Scene 1", "Scene 2", "Scene 3", "Scene 4" };
            if (ImGui::Combo("Set Scene", &_caseId, scNames, IM_ARRAYSIZE(scNames))) {
                Reset();
            }
        }
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_renderer.objColor));
            ImGui::ColorEdit3("Room Color", glm::value_ptr(_renderer.roomColor));
        }
    }

    Common::CaseRenderResult CaseRBStack::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
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

    void CaseRBStack::OnProcessInput(ImVec2 const & pos) {
        if (_controlCamera)
            _renderer.ControlCamera(pos);
        else {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                auto result = GetNearestBody(pos, _renderer.camera, _windowSize, _system.bodies);
                if (result.hit) {
                    _isDragging             = true;
                    _selectedBodyId         = result.bodyId;
                    const RigidBody & body  = *_system.bodies[result.bodyId];
                    glm::mat3         R_inv = glm::transpose(glm::mat3_cast(body.q));
                    _f_point                = R_inv * (result.hitPoint - body.x);
                }
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                _isDragging     = false;
                _selectedBodyId = -1;
            }
        }
    }

    void CaseRBStack::Reset() {
        _system.Clear();
        _system.enableGravity = true;
        float floorSize       = 20.0f;
        float floorThickness  = 0.5f;

        auto floor      = std::make_shared<RigidBody>(0.0f, std::make_shared<BoxShape>(glm::vec3(floorSize, floorThickness, floorSize)));
        floor->isStatic = true;
        floor->x        = { 0.f, -1.0f, 0.f };
        _system.AddBody(floor);

        if (_caseId == 0) {
            float sizes[4] = { 2.f, 1.f, 1.5f, 0.7f };
            float currentY = -0.5f;

            for (int i = 0; i < 4; ++i) {
                float s    = sizes[i];
                auto  cube = std::make_shared<RigidBody>(s * 3, std::make_shared<BoxShape>(glm::vec3(s, s, s)));

                cube->x        = { 0.001f * i, currentY + s * 0.5f + 0.01f, 0.001f * i };
                cube->isStatic = false;

                _system.AddBody(cube);

                currentY += s + 0.01f;
            }
            float ballRadius  = 0.3f;
            float ballMass    = 1.0f;
            auto  sphereShape = std::make_shared<SphereShape>(ballRadius);
            auto  ball        = std::make_shared<RigidBody>(ballMass, sphereShape);

            ball->x        = { 0.001f * 3, currentY + ballRadius + 0.01f, 0.001f * 3 };
            ball->isStatic = false;

            _system.AddBody(ball);
        } else if (_caseId == 1) {
            auto base1 = std::make_shared<RigidBody>(8.0f, std::make_shared<BoxShape>(glm::vec3(0.5f, 0.5f, 0.5f)));
            base1->x = { 0.f, -0.2f, 0 };
            _system.AddBody(base1);

            auto base2 = std::make_shared<RigidBody>(8.0f, std::make_shared<BoxShape>(glm::vec3(0.3f, 1.5f, 0.3f)));
            base2->x = { 0, 1.f, 0 };
            _system.AddBody(base2);

            auto beam = std::make_shared<RigidBody>(4.0f, std::make_shared<BoxShape>(glm::vec3(6.0f, 0.5f, 1.0f)));
            beam->x = { 0, 2.25f, 0 };
            _system.AddBody(beam);

            auto boxL = std::make_shared<RigidBody>(1.0f, std::make_shared<BoxShape>(glm::vec3(0.8f)));
            boxL->x = { -2.5f, 3.f, 0 };
            _system.AddBody(boxL);

            auto boxR = std::make_shared<RigidBody>(1.0f, std::make_shared<BoxShape>(glm::vec3(0.8f)));
            boxR->x = { 2.5f, 3.f, 0 };
            _system.AddBody(boxR);

            auto ballL = std::make_shared<RigidBody>(1.0f, std::make_shared<SphereShape>(0.3f));
            ballL->x = { -2.5f, 3.8f, 0.02f };
            _system.AddBody(ballL);

            auto ballR = std::make_shared<RigidBody>(1.0f, std::make_shared<SphereShape>(0.3f));
            ballR->x = { 2.5f, 3.8f, 0 };
            _system.AddBody(ballR);
        } else if (_caseId == 2) {
            auto box1 = std::make_shared<RigidBody>(8.0f, std::make_shared<BoxShape>(glm::vec3(1.f, 1.f, 1.f)));
            box1->x   = { 0.f, 0.1f, 0 };
            _system.AddBody(box1);

            auto ball1 = std::make_shared<RigidBody>(5.0f, std::make_shared<SphereShape>(0.5f));
            ball1->x   = { 0, 1.3f, 0 };
            _system.AddBody(ball1);

            auto box2 = std::make_shared<RigidBody>(2.0f, std::make_shared<BoxShape>(glm::vec3(0.2f, 0.5f, 0.2f)));
            box2->x   = { 0, 2.1f, 0 };
            _system.AddBody(box2);

            auto box3 = std::make_shared<RigidBody>(2.0f, std::make_shared<BoxShape>(glm::vec3(0.4f, 4.f, 0.4f)));
            box3->x = { 0, 4.5f, 0 };
            _system.AddBody(box3);

            auto ball2 = std::make_shared<RigidBody>(13.0f, std::make_shared<SphereShape>(2.5f));
            ball2->x   = { 0.f, 12.f, 0 };
            _system.AddBody(ball2);
        }  else if (_caseId == 3) {
            float currentY = 0.f;
            for (int i = 0; i < 10; ++i) {
                auto cube = std::make_shared<RigidBody>(1.5f + 0.2f * i, std::make_shared<BoxShape>(glm::vec3(1.0f + i * 0.2f)));

                cube->x        = { 0.f, currentY + 0.5f + i * 0.1f + 1.f, 0.f };
                cube->isStatic = false;

                _system.AddBody(cube);

                currentY += 1.f + i * 0.2f + 1.f;
            }
        }
    }
} // namespace VCX::Labs::RigidBody