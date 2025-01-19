#include "EnvironmentEditor.h"

#include "Application.h"
#include "Command.h"

#include "SDL.h"

namespace ECM {

	namespace WindowApplication {


		EnvironmentEditor::EnvironmentEditor(Application* application)
		{
			m_App = application;
			m_CurrentDragArea = Simulation::SimAreaType::NONE;
		}

		void EnvironmentEditor::HandleInput(SDL_Event& e)
		{
			// EVENT: place simulation area
			if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT && m_CurrentDragArea != Simulation::SimAreaType::NONE)
			{
				HandleDropArea(Point(e.button.x, e.button.y));
			}
		}

		void EnvironmentEditor::Update()
		{

		}

		void EnvironmentEditor::Render()
		{
			DragAreaInternal();
		}

		void EnvironmentEditor::StartDragArea(Simulation::SimAreaType areaType)
		{
			m_CurrentDragArea = areaType;
		}

		void EnvironmentEditor::StopDragArea()
		{
			m_CurrentDragArea = Simulation::SimAreaType::NONE;
		}

		void EnvironmentEditor::DragAreaInternal()
		{
			if (m_CurrentDragArea == Simulation::SimAreaType::NONE) return;

			// TODO: place in variables
			float halfWidth = 50;
			float halfHeight = 50;

			m_App->GetECMRenderer()->RenderDragArea(m_CurrentDragArea, m_App->GetApplicationState()->mousePosition, Vec2(halfWidth, halfHeight));
		}

		void EnvironmentEditor::HandleDropArea(const Point& screenPos)
		{
			m_App->GetUndoRedoManager()->Invoke(new CMD_AddSimulationArea(m_CurrentDragArea, screenPos, Vec2(50.0f, 50.0f), m_App));

			// add simulation area
			//auto sim = m_App->GetSimulator();
			//
			//if (m_CurrentDragArea == Simulation::SimAreaType::SPAWN)
			//{
			//	Simulation::SpawnConfiguration defaultConfig;
			//	Point worldCoords = m_App->GetECMRenderer()->ScreenToWorldCoordinates(screenPos.x, screenPos.y);
			//	sim->AddSpawnArea(worldCoords, Vec2(50.0f, 50.0f), defaultConfig);
			//}
			//
			//if (m_CurrentDragArea == Simulation::SimAreaType::GOAL)
			//{
			//	Point worldCoords = m_App->GetECMRenderer()->ScreenToWorldCoordinates(screenPos.x, screenPos.y);
			//	sim->AddGoalArea(worldCoords, Vec2(50.0f, 50.0f));
			//}

			m_CurrentDragArea = Simulation::SimAreaType::NONE;
		}
	}
}