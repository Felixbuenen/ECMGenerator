#include "SimAreaPanel.h"

#include "Application.h"
#include "Area.h"

#include "SDL.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl2.h"
#include "imgui/imgui_impl_sdlrenderer2.h"

namespace ECM {

	namespace WindowApplication {

		void SimAreaPanel::Render()
		{
			float simPanelHeight = 450.0f;
			float simPanelWidth = 250.0f;
			ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.f, 0.f));
			ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.f, 0.f));

			ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), 0, ImVec2(0.0f, 0));
			ImGui::SetNextWindowSize(ImVec2(simPanelWidth, simPanelHeight));
			ImGui::Begin("Simulation areas");

			// TODO:
			// > we gaan in eerste instantie geen "connect" mode toevoegen
			// > maak een properties panel waarin je eigenschappen van de spawn/goal area kan aanpassen
			// > bij spawn kun je dan ook dynamisch de goal areas toevoegen (connect), eventueel met een "chance" property
			// > evt kun je later de click/drag functionaliteit toevoegen

			// tabs

			if(ImGui::Button("Build", ImVec2(simPanelWidth / 2.0f, 20.0f)))
			{
				m_PanelMode = PanelMode::BUILD;
			}
			ImGui::SameLine();
			if (ImGui::Button("Connect", ImVec2(simPanelWidth / 2.0f, 20.0f)))
			{
				m_PanelMode = PanelMode::CONNECT;
			}
			ImGui::PopStyleVar(2);

			//if (changeToBuildMode) { connectorMode = false; buildMode = true; }
			//if (changeToConnectorMode) { connectorMode = true; buildMode = false; }

			if (m_PanelMode == PanelMode::BUILD)
			{
				ImGui::Text("This is the build panel");

				float buttonSize = 100.0f;
				if (ImGui::Button("Spawn Area", ImVec2(buttonSize, buttonSize)))
				{
					// start draging spawn area into scene...
					m_IsDragging = true;
					m_DragType = SimAreaDrag::SPAWN;
				}

				if (ImGui::Button("Goal Area", ImVec2(buttonSize, buttonSize)))
				{
					// start draging goal area into scene...
					m_IsDragging = true;
					m_DragType = SimAreaDrag::GOAL;
				}
			}
			if (m_PanelMode == PanelMode::CONNECT)
			{
				ImGui::Text("This is the connect panel");
			}

			ImGui::End();
		}

		void SimAreaPanel::Update(SDL_Event& e, Application& app)
		{
			if (m_IsDragging)
			{
				bool drop = e.type == SDL_MOUSEBUTTONDOWN;
				ImGuiIO& io = ImGui::GetIO();
				ImVec2 mousePos = io.MousePos;

				if (drop)
				{
					// add simulation area
					auto sim = app.GetSimulator();
					if (m_DragType == SimAreaDrag::SPAWN)
					{
						Simulation::SpawnConfiguration defaultConfig;
						Point worldCoords = app.GetRenderer()->ScreenToWorldCoordinates(mousePos.x, mousePos.y);
						sim->AddSpawnArea(worldCoords, Vec2(50.0f, 50.0f), defaultConfig);
					}

					if (m_DragType == SimAreaDrag::GOAL)
					{
						Point worldCoords = app.GetRenderer()->ScreenToWorldCoordinates(mousePos.x, mousePos.y);
						sim->AddGoalArea(worldCoords, Vec2(50.0f, 50.0f));
					}

					app.GetRenderer()->StopRenderDragSimulationArea();

					m_IsDragging = false;
					m_DragType = SimAreaDrag::NONE;
				}
				else
				{
					app.GetRenderer()->RenderDragSimulationArea(mousePos.x, mousePos.y, m_DragType);
				}
			}
		}

	}

}