#pragma once
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/1-RigidBody/RigidBodySystem.h"

namespace VCX::Labs::RigidBody {

	class CaseDoubleRB : public Common::ICase {
    public:
        Engine::GL::UniqueProgram               _program;
        Engine::GL::UniqueRenderFrame           _frame;
        Engine::Camera                          _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager              _cameraManager;
        Engine::GL::UniqueIndexedRenderItem     _boxItem;
        Engine::GL::UniqueIndexedRenderItem     _lineItem;
        std::pair<std::uint32_t, std::uint32_t> _windowSize;

        RigidBodySystem _system;
        int             _caseId = 0;

        bool _stopped = false;
        glm::vec3                  _boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };

        CaseDoubleRB();
        virtual std::string_view const GetName() override { return "Two Rigid Bodies Collide"; }
        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void LoadCase();
	};
}