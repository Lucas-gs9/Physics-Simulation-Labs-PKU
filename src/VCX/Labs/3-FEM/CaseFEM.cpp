#include "CaseFEM.h"
#include "Engine/app.h"
#include <iostream>

namespace VCX::Labs::FEM {
    CaseFEM::CaseFEM() :
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _verticesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0).Add<glm::vec4>("color", Engine::GL::DrawFrequency::Stream, 1), Engine::GL::PrimitiveType::Points),
        _linesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _rbItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles) {
        
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

        std::vector<std::uint32_t> rb_index { 0, 1, 2, 0, 2, 3,
                                              4, 5, 6, 4, 6, 7 };
        _rbItem.UpdateElementBuffer(rb_index);
    }

    void CaseFEM::OnSetupPropsUI() {
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
        ImGui::SameLine();
        if (ImGui::Button("Reset System")) _simulator.reset();
        ImGui::Separator();

        const char * mdNames[] = { "StVK", "Neo-Hookean", "Corotated" };
        if (ImGui::Combo("Model", &_mId, mdNames, IM_ARRAYSIZE(mdNames))) {
            _simulator.mdId = _mId;
            _simulator.reset();
        }
        ImGui::Checkbox("Enable Elasto-Plasticity", &_simulator.useEP);
        ImGui::Separator();

        ImGui::SliderFloat("Gravity", &_simulator.gravity[1], 0.f, 1.f, "%.2f");
        if (_simulator.useEP) {
            ImGui::SliderFloat("Viscosity (Eta)", &_simulator.mesh.eta, 0.f, 1.f, "%.2f");
        }

        ImGui::Separator();

        ImGui::Text("Interaction Mode:");
        if (ImGui::RadioButton("View Control", _controlCamera)) _controlCamera = true;
        ImGui::SameLine();
        if (ImGui::RadioButton("Apply Force", ! _controlCamera)) _controlCamera = false;
        if (! _controlCamera) {
            ImGui::SliderFloat("Drag Force", &_strength, 200.f, 500.f, "%.0f");
        }
    }

    Common::CaseRenderResult CaseFEM::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (!_stopped) {

            int   subSteps = 60;
            if (_mId == 1) subSteps = 30;
            else if (_mId == 2) subSteps = 25;
            float sdt      = Engine::GetDeltaTime() / subSteps;
            ImVec2 delta = ImGui::GetIO().MouseDelta;
            float  s        = (float) subSteps;
            ImVec2 subDelta = ImVec2(delta.x / s, delta.y / s);

            for (int i = 0; i < subSteps; ++i) {
                _simulator.ps.clearForces(_simulator.gravity);
                if (_isDragging)
                    applyDragging(subDelta);
                _simulator.update(sdt);
            }
        }

         _frame.Resize(desiredSize);
        _windowSize = desiredSize;

        _rbItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_rbPositions));

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

        _program.GetUniforms().SetByName("u_Color", glm::vec3 { 0.3f, 0.3f, 0.35f });
        _rbItem.Draw({ _program.Use() });

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
        if (_controlCamera)
            _cameraManager.ProcessInput(_camera, pos);
        else {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                _isDragging = getSelectedId(pos, _camera, _windowSize, _simulator.ps, _selectedId);
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                _isDragging = false;
            }
        }
    }

    void CaseFEM::applyDragging(ImVec2 delta) {
        glm::mat4 invView  = glm::inverse(_camera.GetViewMatrix());
        glm::vec3 camRight = glm::vec3(invView[0]);
        glm::vec3 camUp    = glm::vec3(invView[1]);

        glm::vec3 forceDir = camRight * delta.x - camUp * delta.y;

        float radius     = 2.0f; 
        float damping    = 10.0f;

        glm::vec3 interactionCenter = _simulator.ps.x[_selectedId];

#pragma omp parallel for
        for (int i = 0; i < _simulator.ps.size; ++i) {
            float dist = glm::distance(_simulator.ps.x[i], interactionCenter);
            if (dist < radius) {
                float r      = dist / radius;
                float r2     = r * r;
                float weight = (1.0f - r2) * (1.0f - r2);

                glm::vec3 relativeVel      = _simulator.ps.v[i];
                glm::vec3 interactionForce = (forceDir * _strength - relativeVel * damping) * weight;
                
                _simulator.ps.addForce(i, interactionForce);
            }
        }
    }
}