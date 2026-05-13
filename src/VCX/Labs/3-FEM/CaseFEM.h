#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/UniformBlock.hpp"
#include "FEMSimulator.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Scene/Content.h"
#include "Labs/Scene/SceneObject.h"

namespace VCX::Labs::FEM {
    class CaseFEM : public Common::ICase {
    public:
        FEMSimulator _simulator;

        CaseFEM();

        virtual std::string_view const GetName() override { return "FEM Simulation"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        Engine::GL::UniqueProgram     _program;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::Camera                _camera { .Eye = glm::vec3(4.0f, -10.0f, 1.0f), .Target = glm::vec3(4.0f, 0.0f, 0.0f), .Up = glm::vec3(0.0f, 0.0f, 1.0f) };
        Common::OrbitCameraManager    _cameraManager;
        Engine::GL::UniqueRenderItem        _verticesItem; 
        Engine::GL::UniqueIndexedRenderItem _linesItem; 
        float                               _vertexSize { 5 };
        float                               _lineWidth { 0.5f };

        bool _stopped { false };
    };
}