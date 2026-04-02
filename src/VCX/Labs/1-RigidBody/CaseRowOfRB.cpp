#include "Labs/1-RigidBody/CaseRowOfRB.h"

/*
namespace VCX::Labs::RigidBody {
    CaseRowOfRB::CaseRowOfRB():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _objItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _wallItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _wLineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        SyncMeshWithShape(_shape, _lineItem, _objItem);
        SyncMeshWithShape(_wallShape, _wLineItem, _wallItem);
        Reset();
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseRowOfRB::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) Reset();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
        }
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_objColor));
        }
    }

    Common::CaseRenderResult CaseRowOfRB::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (!_stopped) {
            float dt = ImGui::GetIO().DeltaTime;
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
            if (body->shape->GetType() == Shape::Type::Box) {
                DrawBody(body, _wallItem, _wLineItem, _program, _objColor);
            } else if (body->shape->GetType() == Shape::Type::Sphere) {
                DrawBody(body, _objItem, _lineItem, _program, _objColor);
            }
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

    void CaseRowOfRB::OnProcessInput(ImVec2 const& pos) {

    }

    void CaseRowOfRB::Reset() {
        _system.Clear();
        _system.enableGravity = true;
        float floorSize       = 20.0f;
        float floorThickness  = 0.5f;
        float wallHeight      = 5.0f;
        float wallThickness   = 0.5f;
        float ballRadius      = 0.5f;

        auto floor      = std::make_shared<RigidBody>(100.0f, std::make_shared<BoxShape>(glm::vec3(floorSize, floorThickness, floorSize)));
        floor->isStatic = true;
        floor->x        = { 0.f, -1.0f, 0.f };
        _system.AddBody(floor);

        auto wallShape = std::make_shared<BoxShape>(glm::vec3(wallThickness, wallHeight, 10.0f));
        float wallPosY  = floor->x.y + wallHeight * 0.5f + floorThickness * 0.5f;

        auto wallL      = std::make_shared<RigidBody>(100.f, wallShape);
        wallL->isStatic = true;
        wallL->x        = { -8.0f, wallPosY, 0.f };
        _system.AddBody(wallL);

        auto wallR      = std::make_shared<RigidBody>(100.f, wallShape);
        wallR->isStatic = true;
        wallR->x        = { 8.0f, wallPosY, 0.f };
        _system.AddBody(wallR);

        int   numBalls = 2;
        float startX   = -2.0f; 
        auto  sphere   = std::make_shared<SphereShape>(ballRadius);

        for (int i = 0; i < numBalls; ++i) {
            auto ball = std::make_shared<RigidBody>(1.0f, sphere);

            float posX = startX + i * (ballRadius * 2.0f + 0.03f);
            ball->x    = { posX, ballRadius + (floor->x.y + floorThickness * 0.5f), 0.f };

            if (i == 0) {
                ball->v = { 30.0f, 0.0f, 0.0f };
                ball->w = { 0.0f, 0.0f, 0.0f };
            } else {
                ball->v = { 0.f, 0.f, 0.f };
                ball->w = { 0.f, 0.f, 0.f };
            }

            _system.AddBody(ball);
        }
    }
}*/