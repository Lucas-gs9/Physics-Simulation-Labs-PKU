#include "Labs/1-RigidBody/CaseDoubleRB.h"

namespace VCX::Labs::RigidBody {
    void CaseDoubleRB::LoadCase() {
        _system.Clear();

        glm::vec3 dims(1.5f, 1.5f, 1.f);

        if (_caseId == 0) {
            // Face to face
            auto b1 = std::make_shared<RigidBody>(1.0f, std::make_shared<BoxShape>(dims));
            b1->x   = glm::vec3(-3.0f, 0.0f, 0.0f);
            b1->v   = glm::vec3(2.5f, 0.0f, 0.0f);
            b1->q   = glm::quat(1.f, 0.f, 0.f, 0.f);

            auto b2 = std::make_shared<RigidBody>(1.0f, std::make_shared<BoxShape>(dims));
            b2->x   = glm::vec3(3.0f, 0.0f, 0.0f);
            b2->v   = glm::vec3(-2.5f, 0.0f, 0.0f);
            b2->q   = glm::quat(1.f, 0.f, 0.f, 0.f);

            _system.AddBody(b1);
            _system.AddBody(b2);
        } else if (_caseId == 1) {
            // Point to Face
            auto b1 = std::make_shared<RigidBody>(1.0f, std::make_shared<BoxShape>(dims));
            b1->x   = glm::vec3(-2.5f, 0.2f, 0.0f);
            b1->v   = glm::vec3(4.0f, 0.0f, 0.0f);

            auto b2 = std::make_shared<RigidBody>(1.0f, std::make_shared<BoxShape>(dims));
            b2->x   = glm::vec3(1.0f, 0.0f, -0.5f);
            b2->v   = glm::vec3(0.0f, 0.0f, 0.0f);
            b2->q = glm::quat(glm::vec3(glm::radians(30.f), glm::radians(60.f), glm::radians(45.f)));

            _system.AddBody(b1);
            _system.AddBody(b2);
        } else if (_caseId == 2) {
            // Edge to edge
            auto b1 = std::make_shared<RigidBody>(1.0f, std::make_shared<BoxShape>(dims));
            b1->x   = glm::vec3(-3.0f, 0.0f, 0.2f);
            b1->v   = glm::vec3(2.5f, 0.0f, 0.0f);
            b1->q   = glm::angleAxis(glm::radians(45.f), glm::vec3(0, 0, 1));

            auto b2 = std::make_shared<RigidBody>(1.0f, std::make_shared<BoxShape>(dims));
            b2->x   = glm::vec3(3.0f, 0.0f, 0.0f);
            b2->v   = glm::vec3(-2.5f, 0.0f, 0.0f);
            b2->q = glm::angleAxis(glm::radians(45.f), glm::vec3(0, 0, 1));

            _system.AddBody(b1);
            _system.AddBody(b2);
        } else if (_caseId == 3) {
            // Offset head to head
            auto b1 = std::make_shared<RigidBody>(1.0f, std::make_shared<BoxShape>(dims));
            b1->x   = glm::vec3(-2.0f, 0.7f, 0.0f);
            b1->v   = glm::vec3(2.5f, 0.0f, 0.0f);
            b1->q   = glm::quat(1.f, 0.f, 0.f, 0.f);

            auto b2 = std::make_shared<RigidBody>(1.0f, std::make_shared<SphereShape>(1.5f));
            b2->x   = glm::vec3(2.0f, -0.7f, 0.0f);
            b2->v   = glm::vec3(-3.f, 0.0f, 0.0f);
            b2->q   = glm::quat(1.f, 0.f, 0.f, 0.f);

            _system.AddBody(b1);
            _system.AddBody(b2);
        }
    }
    CaseDoubleRB::CaseDoubleRB(){
        LoadCase();
    }
    void CaseDoubleRB::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) LoadCase();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            const char * scNames[] = { "Scene 1", "Scene 2", "Scene 3", "Scene 4" };
            if (ImGui::Combo("Set Scene", &_caseId, scNames, IM_ARRAYSIZE(scNames))) {
                LoadCase();
            }
        }
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_renderer.objColor));
        }
    }
    Common::CaseRenderResult CaseDoubleRB::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped) {
            float dt = ImGui::GetIO().DeltaTime;
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

    void CaseDoubleRB::OnProcessInput(ImVec2 const& pos) {
        _renderer.ControlCamera(pos);
    }
}