#pragma once
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/1-RigidBody/RigidBodySystem.h"

namespace VCX::Labs::RigidBody {
    class CaseRBScene : public Common::ICase {
    public:
        Renderer                                _renderer;
        std::pair<std::uint32_t, std::uint32_t> _windowSize;

        RigidBodySystem                         _system;

        bool      _stopped = false;
        bool      _controlCamera = true;
        bool      _isDragging    = false;
        glm::vec3 _f_point { 0.f };
        int       _selectedBodyId = -1;
        float     _k              = 10.f;



        CaseRBScene();
        virtual std::string_view const GetName() override { return "Complex rigid body system"; }
        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void Reset();
    };
}