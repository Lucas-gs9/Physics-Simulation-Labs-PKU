#include "Labs/1-RigidBody/CaseSingleRB.h"

namespace VCX::Labs::RigidBody {
    CaseSingleRB::CaseSingleRB():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index
        _lineItem.UpdateElementBuffer(line_index);
        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        _boxItem.UpdateElementBuffer(tri_index);
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        auto box = std::make_shared<BoxShape>(glm::vec3(0.5f, 1.f, 1.5f));
        _body    = std::make_unique<RigidBody>(RigidBody(1.f, box));
    }

    void CaseSingleRB::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) _body->Reset(_initv);
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            ImGui::SliderFloat("init V.x", &_initv[0], 0, 2);
            ImGui::SliderFloat("init V.y", &_initv[1], 0, 2);
            ImGui::SliderFloat("init V.z", &_initv[2], 0, 2);
        }
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_boxColor));
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

        glm::vec3 x = _body->x;
        glm::mat3 R = glm::mat3_cast(_body->q);
        glm::vec3 halfDim = std::static_pointer_cast<BoxShape>(_body->shape)->dim * 0.5f;
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
        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.UpdateVertexBuffer("position", span_bytes);
        _boxItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", span_bytes);
        _lineItem.Draw({ _program.Use() });

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
            auto intersection = GetRayBoxIntersection(pos, _camera, _windowSize, _body);
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

    std::optional<glm::vec3> GetRayBoxIntersection(ImVec2 const & mousePos, Engine::Camera const & camera, std::pair<uint32_t, uint32_t> windowSize, const std::unique_ptr<RigidBody> & body) {

        float x = (2.0f * mousePos.x) / windowSize.first - 1.0f;
        float y = 1.0f - (2.0f * mousePos.y) / windowSize.second;

        glm::mat4 invProjView = glm::inverse(camera.GetProjectionMatrix(float(windowSize.first) / windowSize.second) * camera.GetViewMatrix());
        glm::vec4 nearPt      = invProjView * glm::vec4(x, y, -1.0f, 1.0f);
        glm::vec4 farPt       = invProjView * glm::vec4(x, y, 1.0f, 1.0f);
        nearPt /= nearPt.w;
        farPt /= farPt.w;

        glm::vec3 rayOrigin = glm::vec3(nearPt);
        glm::vec3 rayDir    = glm::normalize(glm::vec3(farPt) - rayOrigin);


        glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), body->x) * glm::mat4_cast(body->q);
        glm::mat4 invModel    = glm::inverse(modelMatrix);

        glm::vec3 locOrigin = glm::vec3(invModel * glm::vec4(rayOrigin, 1.0f));
        glm::vec3 locDir    = glm::normalize(glm::vec3(invModel * glm::vec4(rayDir, 0.0f)));

        glm::vec3 boxMin = -std::static_pointer_cast<BoxShape>(body->shape)->dim * 0.5f;
        glm::vec3 boxMax = std::static_pointer_cast<BoxShape>(body->shape)->dim * 0.5f;

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
        float tzMax = (boxMax.z - locOrigin.z) / tzMin; 
        tzMin       = (boxMin.z - locOrigin.z) / locDir.z;
        tzMax       = (boxMax.z - locOrigin.z) / locDir.z;
        if (tzMin > tzMax) std::swap(tzMin, tzMax);

        if ((tMin > tzMax) || (tzMin > tMax)) return std::nullopt;
        if (tzMin > tMin) tMin = tzMin;
        if (tzMax < tMax) tMax = tzMax;

        if (tMin < 0) return std::nullopt;

        return rayOrigin + rayDir * tMin;
    }

}