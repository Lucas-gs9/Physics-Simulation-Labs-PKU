#pragma once
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/1-RigidBody/RigidBodySystem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::RigidBody {
    class CaseRowOfRB : public Common::ICase {
    public:
        Renderer                                _renderer;
        std::pair<std::uint32_t, std::uint32_t> _windowSize;

        RigidBodySystem _system;

        bool      _stopped = false;
        int  _objId   = 0;
        float     _initv   = 10.f;

        CaseRowOfRB();
        virtual std::string_view const   GetName() override { return "Newton's pendulum simulation"; }
        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;
        void                             Reset();
    };
}