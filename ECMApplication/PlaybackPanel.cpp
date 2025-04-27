#include "PlaybackPanel.h"

#include "Application.h"
#include "Area.h"
#include "Simulator.h"

#include "SDL.h"
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

namespace ECM {

	namespace WindowApplication {

		PlaybackPanel::PlaybackPanel(Application* application)
			: UIWidget(application)
		{
			m_ApplicationState = application->GetApplicationState();
		}

		void PlaybackPanel::Render()
		{
			float screenHeight = ImGui::GetIO().DisplaySize.y;
			float screenWidth = ImGui::GetIO().DisplaySize.x;

			const char* playbackModeText = m_ApplicationState->simulationPlaying ? "Pause" : "Play";

			// create playback panel
			float playbackPanelHeight = 30.0f;
			float playbackPanelWidth = 100.0f;
			ImGuiWindowFlags playbackPanelFlags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
				| ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings
				| ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar
				| ImGuiWindowFlags_NoScrollWithMouse;

			ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.f, 0.f));
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.f, 0.f));
			ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.f, 0.f));
			ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(0.f, 0.f));

			ImGui::SetNextWindowPos(ImVec2(screenWidth / 2.0f, screenHeight), 0, ImVec2(0.5f, 1.0f));
			ImGui::SetNextWindowSize(ImVec2(playbackPanelWidth, playbackPanelHeight));
			ImGui::Begin("Playback", nullptr, playbackPanelFlags);
			if (ImGui::Button(playbackModeText, ImVec2(playbackPanelWidth / 2.0f, playbackPanelHeight)))
			{
				m_ApplicationState->simulationPlaying = !m_ApplicationState->simulationPlaying;
				m_ApplicationState->simulationPaused = !m_ApplicationState->simulationPaused;
			}
			ImGui::SameLine();
			if (ImGui::Button("Stop", ImVec2(playbackPanelWidth / 2.0f, playbackPanelHeight)))
			{
				m_ApplicationState->simulationPaused = true;
				m_ApplicationState->simulationPlaying = false;
				m_ApplicationState->simulator->Reset();
			}

			ImGui::End();
			ImGui::PopStyleVar(4);
		}

	}
}
