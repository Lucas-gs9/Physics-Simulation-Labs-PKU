#pragma once
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/1-RigidBody/RigidBody.h"
#include "Labs/1-RigidBody/utils.h"
#include<optional>


namespace VCX::Labs::RigidBody {

	class CaseSingleRB : public Common::ICase {
    public:
        Engine::GL::UniqueProgram     _program;
        Engine::GL::UniqueRenderFrame _frame;
        Engine::Camera                _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager    _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _objItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;
        std::pair<std::uint32_t, std::uint32_t> _windowSize;

        std::shared_ptr<Shape>     _shape;
        std::shared_ptr<RigidBody> _body;
        glm::vec3                  _boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        glm::vec3                  _initv { 0.f, 0.f, 0.f };

        glm::vec3 _f_point { 0.f };
        bool      _isDragging = false;
        bool      _stopped    = false;
        int       _objId      = 0;

        CaseSingleRB();
        virtual std::string_view const   GetName() override { return "Single rigid body simulation"; }
		virtual void OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;
        void                             reset();
	};
}