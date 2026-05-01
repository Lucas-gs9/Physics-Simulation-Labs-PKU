#include "CaseFLIP.h"
#include "Engine/app.h"

namespace VCX::Labs::Fluid {
    const std::vector<glm::vec3> vertex_pos = {
        glm::vec3(0.06f, 0.06f, 0.06f),
        glm::vec3(0.94f, 0.06f, 0.06f),
        glm::vec3(0.94f, 0.94f, 0.06f),
        glm::vec3(0.06f, 0.94f, 0.06f),
        glm::vec3(0.06f, 0.06f, 0.94f),
        glm::vec3(0.94f, 0.06f, 0.94f),
        glm::vec3(0.94f, 0.94f, 0.94f),
        glm::vec3(0.06f, 0.94f, 0.94f)
    };
    const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 };
    
    CaseFLIP::CaseFLIP(std::initializer_list<Assets::ExampleScene> && scenes):
        _scenes(scenes),
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/fluid.vert"),
                                        Engine::GL::SharedShader("assets/shaders/fluid.frag") })),
        _lineprogram(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _sceneObject(1),
        _BoundaryItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        _solver = std::make_unique<Fluid::HybridSolver>();
        _program.BindUniformBlock("PassConstants", 1);
        _program.GetUniforms().SetByName("u_DiffuseMap", 0);
        _program.GetUniforms().SetByName("u_SpecularMap", 1);
        _program.GetUniforms().SetByName("u_HeightMap", 2);
        _lineprogram.GetUniforms().SetByName("u_Color", glm::vec3(1.0f));
        _BoundaryItem.UpdateElementBuffer(line_index);
        ResetSystem();
        _sphere = Engine::Model { Engine::Sphere(6, _solver->data.particles.radius), 0 };
        _cameraManager.AutoRotate = false;
    }

    void CaseFLIP::OnSetupPropsUI() {
        if (ImGui::Button("Reset System"))
            ResetSystem();
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation"))
            _stopped = ! _stopped;
        ImGui::SliderFloat("FLIP Ratio", &_solver->flipRatio, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("Time Scale", &_timeScale, 0.2f, 1.0f, "%.3f");
    }
    Common::CaseRenderResult CaseFLIP::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_recompute) {
            _recompute = false;
            _sceneObject.ReplaceScene(GetScene(_sceneIdx));
            _cameraManager.Save(_sceneObject.Camera);
        }

        if (! _stopped)
            _solver->step(_timeScale * Engine::GetDeltaTime());

        _BoundaryItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(vertex_pos));
        _frame.Resize(desiredSize);

        _cameraManager.Update(_sceneObject.Camera);
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::Projection, _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::View, _sceneObject.Camera.GetViewMatrix());
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::ViewPosition, _sceneObject.Camera.Eye);
        _lineprogram.GetUniforms().SetByName("u_Projection", _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _lineprogram.GetUniforms().SetByName("u_View", _sceneObject.Camera.GetViewMatrix());

        if (_uniformDirty) {
            _uniformDirty = false;
            _program.GetUniforms().SetByName("u_AmbientScale", _ambientScale);
            _program.GetUniforms().SetByName("u_UseBlinn", _useBlinn);
            _program.GetUniforms().SetByName("u_Shininess", _shininess);
            _program.GetUniforms().SetByName("u_UseGammaCorrection", int(_useGammaCorrection));
            _program.GetUniforms().SetByName("u_AttenuationOrder", _attenuationOrder);
            _program.GetUniforms().SetByName("u_BumpMappingBlend", _bumpMappingPercent * .01f);
        }

        gl_using(_frame);

        glEnable(GL_DEPTH_TEST);
        glLineWidth(_BndWidth);
        _BoundaryItem.Draw({ _lineprogram.Use() });
        glLineWidth(1.f);

        Rendering::ModelObject m        = Rendering::ModelObject(_sphere, _solver->data.particles.positions,_solver->data.particles.colors);
        auto const &           material = _sceneObject.Materials[0];
        m.Mesh.Draw({ material.Albedo.Use(), material.MetaSpec.Use(), material.Height.Use(), _program.Use() }, _sphere.Mesh.Indices.size(), 0, numofSpheres);

        glDepthFunc(GL_LEQUAL);
        glDepthFunc(GL_LESS);
        glDisable(GL_DEPTH_TEST);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseFLIP::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_sceneObject.Camera, pos);
    }

    void CaseFLIP::ResetSystem() {
        _solver->reset();
        numofSpheres = _solver->data.particles.size();

        _stopped   = false; 
        //_recompute = true;
    }
}