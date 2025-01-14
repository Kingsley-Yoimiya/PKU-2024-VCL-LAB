#include "Labs/Final_Project/App.h"

namespace VCX::Labs::Animation {
    App::App() : _ui(Labs::Common::UIOptions { }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
