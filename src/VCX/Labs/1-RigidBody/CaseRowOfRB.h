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
        Engine::GL::UniqueProgram               _program;
        Engine::GL::UniqueRenderFrame           _frame;
        Engine::Camera                          _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager              _cameraManager;
        Engine::GL::UniqueIndexedRenderItem     _objItem;
        Engine::GL::UniqueIndexedRenderItem     _lineItem;
        Engine::GL::UniqueIndexedRenderItem     _wallItem;
        Engine::GL::UniqueIndexedRenderItem     _wLineItem;
        std::pair<std::uint32_t, std::uint32_t> _windowSize;

        std::shared_ptr<Shape> _shape = std::make_shared<SphereShape>(0.5f);
        std::shared_ptr<Shape> _wallShape = std::make_shared<BoxShape>(glm::vec3 { 0.1f, 10.f, 5.f });
        RigidBodySystem _system;

        glm::vec3 _objColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        bool      _stopped = false;

        CaseRowOfRB();
        virtual std::string_view const   GetName() override { return "Newton's pendulum simulation"; }
        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;
        void                             Reset();
    };
}