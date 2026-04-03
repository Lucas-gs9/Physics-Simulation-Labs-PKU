#include "Labs/1-RigidBody/CaseSingleRB.h"

namespace VCX::Labs::RigidBody {
    CaseSingleRB::CaseSingleRB(){
        reset();
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
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_renderer.objColor));
            const char * objNames[] = { "Cuboid", "Sphere", "Cylinder" };
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

                glm::mat4 invView  = glm::inverse(_renderer.camera.GetViewMatrix());
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
            _body->ClearForce();
        }
        
        _windowSize = desiredSize;
        
        _renderer.Refresh(std::vector<std::shared_ptr<RigidBody>> { _body }, _windowSize);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _renderer.frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseSingleRB::OnProcessInput(ImVec2 const & pos) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            auto intersection = GetRayBodyIntersection(pos, _renderer.camera, _windowSize, *_body);
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
        std::shared_ptr<Shape> shape;
        if (_objId == 0) shape = std::make_shared<BoxShape>(glm::vec3(0.5f, 1.f, 1.5f));
        else if (_objId == 1) shape = std::make_shared<SphereShape>(1.2f);
        else if (_objId == 2) {
            shape = std::make_shared<CylinderShape>(0.5f, 1.5f);
        }
        _body = std::make_shared<RigidBody>(RigidBody(1.f, shape));
        _body->Reset(_initv);
    }
}