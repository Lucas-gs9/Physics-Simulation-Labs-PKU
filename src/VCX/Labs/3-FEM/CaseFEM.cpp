#include "CaseFEM.h"
#include "Engine/app.h"

namespace VCX::Labs::FEM {
    CaseFEM::CaseFEM() :
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _verticesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0).Add<glm::vec4>("color", Engine::GL::DrawFrequency::Stream, 1), Engine::GL::PrimitiveType::Points),
        _linesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        
        _cameraManager.AutoRotate = false;
        _camera.Eye               = glm::vec3(7.0f, 3.0f, 10.0f);
        _camera.Target            = glm::vec3(4.0f, 1.0f, 1.0f);
        _camera.Up                = glm::vec3(0.0f, 1.0f, 0.0f);
        _camera.Fovy              = 45.0f;
        _cameraManager.Save(_camera);

        _simulator.init();

        std::vector<std::uint32_t> line_indices;
        for (const auto & tet : _simulator.mesh.elements) {
            int v[4] = { tet.indices[0], tet.indices[1], tet.indices[2], tet.indices[3] };
            int edges[6][2] = {
                {0, 1},{0, 2},{0, 3},
                {1, 2},{1, 3},{2, 3}
            };
            for (auto & edge : edges) {
                line_indices.push_back(v[edge[0]]);
                line_indices.push_back(v[edge[1]]);
            }
        }
        _linesItem.UpdateElementBuffer(line_indices);
    }

    void CaseFEM::OnSetupPropsUI() {
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
        if (ImGui::Button("Reset System")) _simulator.reset();
        ImGui::SliderFloat("Gravity", &_simulator.gravity[1], 0.05f, 0.3f);
    }

    Common::CaseRenderResult CaseFEM::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (!_stopped) {
            _simulator.update(Engine::GetDeltaTime());
        }

         _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);

        auto posSpan = Engine::make_span_bytes<glm::vec3>(_simulator.ps.x);

        _linesItem.UpdateVertexBuffer("position", posSpan);
        _verticesItem.UpdateVertexBuffer("position", posSpan);

        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glPointSize(_vertexSize);
        glLineWidth(_lineWidth);

        _program.GetUniforms().SetByName("u_Color", glm::vec3 { 0.0f, 0.9f, 0.6f });
        _linesItem.Draw({ _program.Use() });
        _program.GetUniforms().SetByName("u_Color", glm::vec3 { 1.0f, 0.2f, 0.8f });
        _verticesItem.Draw({ _program.Use() });

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

    void CaseFEM::OnProcessInput(ImVec2 const& pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }
}