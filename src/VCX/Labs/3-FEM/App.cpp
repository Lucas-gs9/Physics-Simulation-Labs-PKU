#include "App.h"
#include "Assets/bundled.h"

namespace VCX::Labs::FEM {

    App::App():
        _ui(Labs::Common::UIOptions {}),
        _caseFEM() {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
