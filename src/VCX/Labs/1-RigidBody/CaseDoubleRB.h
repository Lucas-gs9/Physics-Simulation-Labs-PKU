#pragma once
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/1-RigidBody/RigidBodySystem.h"
#include "Labs/1-RigidBody/utils.h"

namespace VCX::Labs::RigidBody {

	class CaseDoubleRB : public Common::ICase {
    public:
        Renderer                                _renderer;
        std::pair<std::uint32_t, std::uint32_t> _windowSize;

        RigidBodySystem _system;
        int             _caseId = 0;

        bool _stopped = false;

        CaseDoubleRB();
        virtual std::string_view const GetName() override { return "Double rigid bodiy collision"; }
        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void LoadCase();
	};
}