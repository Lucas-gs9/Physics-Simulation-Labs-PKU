#include "Labs/1-RigidBody/CaseSingleRB.h"

namespace VCX::Labs::RigidBody {
    CaseSingleRB::CaseSingleRB():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _objItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        reset();

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseSingleRB::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) reset();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            ImGui::SliderFloat("init V.x", &_initv[0], 0, 2);
            ImGui::SliderFloat("init V.y", &_initv[1], 0, 2);
            ImGui::SliderFloat("init V.z", &_initv[2], 0, 2);
        }
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_boxColor));
            const char * objNames[] = { "Cuboid", "Sphere" };
            if (ImGui::Combo("Set Object", &_objId, objNames, IM_ARRAYSIZE(objNames))) {
                reset();
            }
        }
    }

    Common::CaseRenderResult CaseSingleRB::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        

        if (!_stopped) {
            if (_isDragging) {
                ImVec2 delta = ImGui::GetIO().MouseDelta;

                glm::vec3 mouseDelta(delta.x, delta.y, 0.0f);

                glm::mat4 invView  = glm::inverse(_camera.GetViewMatrix());
                glm::vec3 camRight = glm::vec3(invView[0]);
                glm::vec3 camUp    = glm::vec3(invView[1]);

                float     forceScale = 0.1f;
                glm::vec3 forceWorld = (camRight * mouseDelta.x - camUp * mouseDelta.y) * forceScale;

                glm::mat3 R                     = glm::mat3_cast(_body->q);
                glm::vec3 current_f_point_world = _body->x + R * _f_point;
                _body->AddForce(forceWorld, current_f_point_world);
            }

            float dt = ImGui::GetIO().DeltaTime;
            if (dt > 0.1f) dt = 0.1f;
            _body->Update(dt);
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

        DrawBody(_body, _objItem, _lineItem, _program, _boxColor);

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

    void CaseSingleRB::OnProcessInput(ImVec2 const & pos) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            auto intersection = GetRayBodyIntersection(pos, _camera, _windowSize, *_body);
            if (intersection.has_value()) {
                _isDragging     = true;
                glm::mat3 R_inv = glm::transpose(glm::mat3_cast(_body->q));
                _f_point        = R_inv * (intersection.value() - _body->x);
            }
        }
        if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
            _isDragging = false;
        }
    }

    void CaseSingleRB::reset() {
        if (_objId == 0) _shape = std::make_shared<BoxShape>(glm::vec3(0.5f, 1.f, 1.5f));
        else _shape = std::make_shared<SphereShape>(1.2f);
        _body = std::make_unique<RigidBody>(RigidBody(1.f, _shape));
        SyncMeshWithShape(_shape, _lineItem, _objItem);
        DrawBody(_body, _objItem, _lineItem, _program, _boxColor);
        _body->Reset(_initv);
    }
}