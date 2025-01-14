#include "Engine/app.h"
#include "Labs/Final_Project/CaseMassSpring.h"
#include "Labs/Final_Project/tasks.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Animation {
    CaseMassSpring::CaseMassSpring() :
        _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/flat.vert"),
                Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _particlesItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Points),
        _springsItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Lines) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        ResetSystem();
    }

    void CaseMassSpring::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Data", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderInt("Length of the data", &_num, 5, 30);
            // if (ImGui::Button("Reset System")) ResetSystem();
        }

        ImGui::Spacing();
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Algorithm Type");
            ImGui::Bullet();
            if (ImGui::Selectable("Orinal[IterNum *= 100]", _algType == AlgorithmType::Original)) _algType = AlgorithmType::Original;
            ImGui::Bullet();
            if (ImGui::Selectable("NewtonDescent", _algType == AlgorithmType::NewtonDescent)) _algType = AlgorithmType::NewtonDescent;
            ImGui::Bullet();
            if (ImGui::Selectable("Global-Local", _algType == AlgorithmType::Global_Local)) _algType = AlgorithmType::Global_Local;
            if (ImGui::Button("Reset System")) ResetSystem();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            ImGui::SliderFloat("Part. Mass", &_massSpringSystem.Mass, .5f, 10.f);
            ImGui::SliderFloat("Spr. Stiff.", &_massSpringSystem.Stiffness, 10.f, 300.f);
            ImGui::SliderFloat("Spr. Damp.", &_massSpringSystem.Damping, .1f, 10.f);
            ImGui::SliderFloat("Gravity", &_massSpringSystem.Gravity, .1f, 1.f);
            ImGui::SliderInt("Iteration Num", &_iteration_num, 1, 30);
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Appearance")) {
            ImGui::SliderFloat("Part. Size", &_particleSize, 1, 6);
            ImGui::ColorEdit3("Part. Color", glm::value_ptr(_particleColor));
            ImGui::SliderFloat("Spr. Width", &_springWidth, .001f, 1.f);
            ImGui::ColorEdit3("Spr. Color", glm::value_ptr(_springColor));
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseMassSpring::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped) AdvanceMassSpringSystem(_massSpringSystem, Engine::GetDeltaTime(), _algType, _iteration_num, _reseted);
        
        _particlesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_massSpringSystem.Positions));
        _springsItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_massSpringSystem.Positions));

        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);

        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View"      , _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glPointSize(_particleSize);
        glLineWidth(_springWidth);

        _program.GetUniforms().SetByName("u_Color", _springColor);
        _springsItem.Draw({ _program.Use() });
        _program.GetUniforms().SetByName("u_Color", _particleColor);
        _particlesItem.Draw({ _program.Use() });

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseMassSpring::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseMassSpring::ResetSystem() {
        _reseted = true;
        _massSpringSystem = { };
        std::size_t n = _num;
        float const delta = 2.f / 10;
        auto GetID = [&](std::size_t const i, std::size_t const j) { return i * (n + 1) + j; };
        for (std::size_t i = 0; i <= n; i++) {
            for (std::size_t j = 0; j <= n; j++) {
                _massSpringSystem.AddParticle(glm::vec3(i * delta , 1.5f, j * delta - 1.f));
                if (i > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j));
                if (i > 1) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 2, j));
                if (j > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i, j - 1));
                if (j > 1) _massSpringSystem.AddSpring(GetID(i, j), GetID(i, j - 2));
                if (i > 0 && j > 0) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j - 1));
                if (i > 0 && j < n) _massSpringSystem.AddSpring(GetID(i, j), GetID(i - 1, j + 1));
            }
        }
        _massSpringSystem.Fixed[GetID(0, 0)] = true;
        _massSpringSystem.Fixed[GetID(0, n)] = true;
        // _massSpringSystem.Fixed[GetID(n, 0)] = true;
        // _massSpringSystem.Fixed[GetID(n, n)] = true;
        std::vector<std::uint32_t> indices;
        for (auto const & spring : _massSpringSystem.Springs) {
            indices.push_back(std::uint32_t(spring.AdjIdx.first));
            indices.push_back(std::uint32_t(spring.AdjIdx.second));
        }
        _springsItem.UpdateElementBuffer(indices);
    }
}