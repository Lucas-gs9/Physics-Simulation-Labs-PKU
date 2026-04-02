#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseSingleRB.h"
#include "Labs/1-RigidBody/CaseDoubleRB.h"
#include "Labs/1-RigidBody/CaseRBScene.h"
#include "Labs/1-RigidBody/CaseRowOfRB.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::RigidBody {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;
        CaseSingleRB _caseSingleRB;
        CaseDoubleRB _caseDoubleRB;
        CaseRBScene  _caseRBScene;
        CaseRowOfRB  _caseRowOfRB;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseSingleRB, _caseDoubleRB, _caseRBScene, _caseRowOfRB };

    public:
        App();

        void OnFrame() override;
    };
}