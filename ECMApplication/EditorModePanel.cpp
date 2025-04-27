#include "EditorModePanel.h"

#include "Application.h"
#include "Area.h"

#include "SDL.h"
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

namespace ECM {

	namespace WindowApplication {

		EditorModePanel::EditorModePanel(Application* application)
			: UIWidget(application)
		{
			m_ApplicationState = application->GetApplicationState();
		}

		void EditorModePanel::Render()
		{
			if (m_ApplicationState->mode != ApplicationMode::CONNECT) return;

			float screenWidth = ImGui::GetIO().DisplaySize.x;

			// create playback panel
			float labelHeight = 50.0f;
			float labelWidth = 450.0f;
			ImGuiWindowFlags playbackPanelFlags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
				| ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings
				| ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar
				| ImGuiWindowFlags_NoScrollWithMouse;

			ImGui::SetNextWindowPos(ImVec2(screenWidth/2.0f, 50.0f), 0, ImVec2(0.5f, 0.5f));
			ImGui::SetNextWindowSize(ImVec2(labelWidth, labelHeight));
			
			ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.4f));
			
			ImGui::Begin("ConnectionModeLabel", nullptr, playbackPanelFlags);

			std::string text = "Entered connection mode. Click-drag to establish spawn- and goal area connections. Press ESC to quit connection mode.";

			float win_width = ImGui::GetWindowSize().x;
			float text_width = ImGui::CalcTextSize(text.c_str()).x;

			// calculate the indentation that centers the text on one line, relative
			// to window left, regardless of the `ImGuiStyleVar_WindowPadding` value
			float text_indentation = (win_width - text_width) * 0.5f;

			// if text is too long to be drawn on one line, `text_indentation` can
			// become too small or even negative, so we check a minimum indentation
			float min_indentation = 20.0f;
			if (text_indentation <= min_indentation) {
				text_indentation = min_indentation;
			}

			ImGui::SameLine(text_indentation);
			ImGui::PushTextWrapPos(win_width - text_indentation);
			ImGui::TextWrapped(text.c_str());
			ImGui::PopTextWrapPos();

			ImGui::PopStyleColor();

			ImGui::End();
		}
	}

}