#pragma once
#include <vector>

#include "Engine/app.h"
#include "CaseFLIP.h"
#include "CaseAPIC.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::Fluid {
    class App : public Engine::IApp {
    private:
        Common::UI   _ui;
        CaseFLIP   _caseFLIP;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseFLIP};

    public:
        App();

        void OnFrame() override;
    };
}