#include "Labs/1-RigidBody/CaseRowOfRB.h"


namespace VCX::Labs::RigidBody {
    CaseRowOfRB::CaseRowOfRB(){
        Reset();
    }

    void CaseRowOfRB::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) Reset();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            ImGui::SliderFloat("init V", &_initv, 5, 50);
        }
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_renderer.objColor));
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_renderer.roomColor));
            const char * objNames[] = { "Cuboid", "Sphere" };
            if (ImGui::Combo("Set Object", &_objId, objNames, IM_ARRAYSIZE(objNames))) {
                Reset();
            }
        }
    }

    Common::CaseRenderResult CaseRowOfRB::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (!_stopped) {
            float dt = ImGui::GetIO().DeltaTime;
            if (dt > 0.1f) dt = 0.1f;
            _system.Update(dt);
            for (auto body : _system.bodies) {
                body->v.y = 0.f;
                if (! body->isStatic) body->x.y = -0.24f;
            }
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

    void CaseRowOfRB::OnProcessInput(ImVec2 const& pos) {
        _renderer.ControlCamera(pos);
    }

    void CaseRowOfRB::Reset() {
        _system.Clear();
        _system.enableGravity = false;
        _system.enableDamp    = false;
        _system.muN           = 1.f;
        _system.muT           = 0.f;
        _system.beta          = 0.f;

        float floorSize       = 25.0f;
        float floorThickness  = 0.5f;
        float wallHeight      = 4.0f;
        float wallThickness   = 0.5f;

        glm::vec3 boxDim(1.0f, 1.0f, 1.0f); 
        float     gap = 0.3f;               

        auto floor      = std::make_shared<RigidBody>(0.0f, std::make_shared<BoxShape>(glm::vec3(floorSize, floorThickness, floorSize)));
        floor->isStatic = true;
        floor->x        = { 0.f, -1.0f, 0.f };
        _system.AddBody(floor);

        auto  wallShape = std::make_shared<BoxShape>(glm::vec3(wallThickness, wallHeight, 5.0f));
        float wallPosY  = floor->x.y + wallHeight * 0.5f + floorThickness * 0.5f;

        auto wallL      = std::make_shared<RigidBody>(0.0f, wallShape);
        wallL->isStatic = true;
        wallL->x        = { -10.0f, wallPosY, 0.f }; 
        _system.AddBody(wallL);

        auto wallR      = std::make_shared<RigidBody>(0.0f, wallShape);
        wallR->isStatic = true;
        wallR->x        = { 10.0f, wallPosY, 0.f }; 
        _system.AddBody(wallR);

        int   numBoxes = 5;
        float startX   = -2.0f; 
        if (_objId==0) {
            auto boxShape = std::make_shared<BoxShape>(boxDim);
            for (int i = 0; i < numBoxes; ++i) {
                auto box = std::make_shared<RigidBody>(1.0f, boxShape);

                float posX = startX + i * (boxDim.x + gap);
                float posY = floor->x.y + floorThickness * 0.5f + boxDim.y * 0.5f + 0.01f;

                box->x = { posX, posY, 0.f };
                box->q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

                if (i == 0) {
                    box->v = { -_initv, 0.0f, 0.0f };
                    box->w = { 0.0f, 0.0f, 0.0f };
                } else {
                    box->v = { 0.f, 0.f, 0.f };
                    box->w = { 0.f, 0.f, 0.f };
                }

                _system.AddBody(box);
            }
        } else if (_objId == 1) {
            auto objShape = std::make_shared<SphereShape>(0.5f);
            for (int i = 0; i < numBoxes; ++i) {
                auto obj = std::make_shared<RigidBody>(1.0f, objShape);

                float posX = startX + i * (boxDim.x + gap);
                float posY = floor->x.y + floorThickness * 0.5f + boxDim.y * 0.5f + 0.02f;

                obj->x = { posX, posY, 0.f };
                obj->q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

                if (i == 0) {
                    obj->v = { -_initv, 0.0f, 0.0f };
                    obj->w = { 0.0f, 0.0f, 0.0f };
                } else {
                    obj->v = { 0.f, 0.f, 0.f };
                    obj->w = { 0.f, 0.f, 0.f };
                }

                _system.AddBody(obj);
            }
        }
    }
}