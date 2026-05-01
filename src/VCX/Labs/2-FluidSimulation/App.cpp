#include "App.h"
#include "Assets/bundled.h"

namespace VCX::Labs::Fluid {

    App::App():
        _ui(Labs::Common::UIOptions {}),
        _caseFLIP({ Assets::ExampleScene::Fluid }){
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
