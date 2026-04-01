#include "Labs/1-RigidBody/CaseRBScene.h"

namespace VCX::Labs::RigidBody {
    void CaseRBScene::Reset() {
        _system.Clear();
        _system.enableGravity = true;

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

        auto box4 = std::make_shared<RigidBody>(7.f, std::make_shared<BoxShape>(glm::vec3(1.7f, 1.2f, 0.3f)));
        box4->x   = { 5.f, 3.0f, -1.f };
        box4->v   = { 0.f, -1.0f, 0.f };
        box4->q   = glm::angleAxis(glm::radians(45.f), glm::vec3(0, 0, 1));
        _system.AddBody(box4);
    }

    CaseRBScene::CaseRBScene():
    _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines){
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 };
        _lineItem.UpdateElementBuffer(line_index);
        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        _boxItem.UpdateElementBuffer(tri_index);
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
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
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_boxColor));
            ImGui::ColorEdit3("Room Color", glm::value_ptr(_floorColor));
        }
    }

    Common::CaseRenderResult CaseRBScene::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped) {

            if (_isDragging) {
                ImVec2 delta = ImGui::GetIO().MouseDelta;

                glm::vec3 mouseDelta(delta.x, delta.y, 0.0f);

                glm::mat4 invView  = glm::inverse(_camera.GetViewMatrix());
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

        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        for (auto body : _system.bodies) {
            glm::vec3              x       = body->x;
            glm::mat3              R       = glm::mat3_cast(body->q);
            glm::vec3              halfDim = std::static_pointer_cast<BoxShape>(body->shape)->dim * 0.5f;
            std::vector<glm::vec3> VertsPosition(8);
            VertsPosition[0] = x + R * glm::vec3(-halfDim.x, halfDim.y, halfDim.z);
            VertsPosition[1] = x + R * glm::vec3(halfDim.x, halfDim.y, halfDim.z);
            VertsPosition[2] = x + R * glm::vec3(halfDim.x, halfDim.y, -halfDim.z);
            VertsPosition[3] = x + R * glm::vec3(-halfDim.x, halfDim.y, -halfDim.z);
            VertsPosition[4] = x + R * glm::vec3(-halfDim.x, -halfDim.y, halfDim.z);
            VertsPosition[5] = x + R * glm::vec3(halfDim.x, -halfDim.y, halfDim.z);
            VertsPosition[6] = x + R * glm::vec3(halfDim.x, -halfDim.y, -halfDim.z);
            VertsPosition[7] = x + R * glm::vec3(-halfDim.x, -halfDim.y, -halfDim.z);

            auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);
            if (body->isStatic) {
                _program.GetUniforms().SetByName("u_Color", _floorColor);
            } else {
                _program.GetUniforms().SetByName("u_Color", _boxColor);
            }
            _boxItem.UpdateVertexBuffer("position", span_bytes);
            _boxItem.Draw({ _program.Use() });

            _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
            _lineItem.UpdateVertexBuffer("position", span_bytes);
            _lineItem.Draw({ _program.Use() });
        }

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseRBScene::OnProcessInput(ImVec2 const & pos) {
        if (_controlCamera)
            _cameraManager.ProcessInput(_camera, pos);
        else {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                auto result = GetNearestBody(pos, _camera, _windowSize, _system.bodies);
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