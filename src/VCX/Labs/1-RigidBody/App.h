#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseSingleRB.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::RigidBody {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;
        CaseSingleRB _caseSingleRB;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseSingleRB };

    public:
        App();

        void OnFrame() override;
    };
}