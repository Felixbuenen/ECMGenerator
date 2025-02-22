#include "EnvironmentEditor.h"

#include "Application.h"
#include "Command.h"

#include "SDL.h"

namespace ECM {

	namespace WindowApplication {

		void EnvironmentEditor::Initialize(Application* application)
		{
			m_App = application;
			m_CurrentDragArea = Simulation::SimAreaType::NONE;
			m_SelectedArea = nullptr;
			m_ActiveGizmo = nullptr;
			m_TransformMode = TRANSLATE;

			m_TranslateGizmo.Initialize(m_App->GetUndoRedoManager(), application->GetECMRenderer());
			m_ScaleGizmo.Initialize(m_App->GetUndoRedoManager(), application->GetECMRenderer());
		}

		void EnvironmentEditor::HandleInput(SDL_Event& e)
		{
			if (m_ActiveGizmo)
			{
				if (m_ActiveGizmo->HandleInput(e, m_App->GetECMRenderer()))
				{
					return;
				}
			}

			if (m_ActiveGizmo && e.type == SDL_KEYDOWN)
			{
				if (e.key.keysym.sym == SDLK_w)
				{
					m_TransformMode = TRANSLATE;
					m_TranslateGizmo.SetActiveArea(m_SelectedArea);
					m_ActiveGizmo = &m_TranslateGizmo;
					m_App->GetECMRenderer()->RenderGizmo(m_ActiveGizmo);
				}
				if (e.key.keysym.sym == SDLK_r)
				{
					m_TransformMode = SCALE;
					m_ScaleGizmo.SetActiveArea(m_SelectedArea);
					m_ActiveGizmo = &m_ScaleGizmo;
					m_App->GetECMRenderer()->RenderGizmo(m_ActiveGizmo);
				}
			}

			// EVENT: place simulation area
			if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT && m_CurrentDragArea != Simulation::SimAreaType::NONE)
			{
				HandleDropArea(Point(e.button.x, e.button.y));
				return;
			}

			// TODO: click drag gizmos handling...

			if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT && m_CurrentDragArea == Simulation::SimAreaType::NONE)
			{
				HandleLeftClickGeneral(Point(e.button.x, e.button.y));
				return;
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

		void EnvironmentEditor::SelectArea(Simulation::Area* area)
		{
			if (!area)
			{
				if (m_SelectedArea)
				{
					m_ActiveGizmo->SetActiveArea(nullptr);
				}

				m_ActiveGizmo = nullptr;
				m_SelectedArea = nullptr;

				m_App->GetECMRenderer()->RenderGizmo(nullptr);

				return;
			}

			std::cout << "Selected area " << area->ID << std::endl;

			if (m_TransformMode == TRANSLATE) m_ActiveGizmo = &m_TranslateGizmo;
			else if (m_TransformMode == SCALE) m_ActiveGizmo = &m_ScaleGizmo;
			else return;

			m_SelectedArea = area;
			m_ActiveGizmo->SetActiveArea(area);
			m_App->GetECMRenderer()->RenderGizmo(m_ActiveGizmo);
		}

		void EnvironmentEditor::HandleLeftClickGeneral(const Point& screenPos)
		{
			// todo: undo redo select area

			// check if user clicked spawn- or goal area
			auto& spawnAreas = m_App->GetSimulator()->GetSpawnAreas();
			auto& goalAreas = m_App->GetSimulator()->GetGoalAreas();

			Point worldPos = m_App->GetECMRenderer()->ScreenToWorldCoordinates(screenPos.x, screenPos.y);

			bool foundIntersection = false;
			for (auto& a : spawnAreas)
			{
				// check if area intersects
				bool intersects = worldPos.x <= (a.Position.x + a.HalfWidth);
				intersects &= worldPos.x >= (a.Position.x - a.HalfWidth);
				intersects &= worldPos.y <= (a.Position.y + a.HalfHeight);
				intersects &= worldPos.y >= (a.Position.y - a.HalfHeight);

				if (intersects)
				{
					if (m_SelectedArea == &a) return;
					
					foundIntersection = true;
					m_App->GetUndoRedoManager()->Invoke(new CMD_SelectArea(this, m_SelectedArea, &a));

					//SelectArea(&a);
				}
			}

			// TODO: refactor to avoid code duplication

			if (foundIntersection) return;
			for (auto& a : goalAreas)
			{
				// check if area intersects
				bool intersects = worldPos.x <= (a.Position.x + a.HalfWidth);
				intersects &= worldPos.x >= (a.Position.x - a.HalfWidth);
				intersects &= worldPos.y <= (a.Position.y + a.HalfHeight);
				intersects &= worldPos.y >= (a.Position.y - a.HalfHeight);

				if (intersects)
				{
					if (m_SelectedArea == &a) return;

					foundIntersection = true;
					m_App->GetUndoRedoManager()->Invoke(new CMD_SelectArea(this, m_SelectedArea, &a));

					//SelectArea(&a);
				}
			}

			if (!foundIntersection && m_SelectedArea)
			{
				m_App->GetUndoRedoManager()->Invoke(new CMD_SelectArea(this, m_SelectedArea, nullptr));
				//SelectArea(nullptr);
			}
		}


		void EnvironmentEditor::HandleDropArea(const Point& screenPos)
		{
			m_App->GetUndoRedoManager()->Invoke(new CMD_AddSimulationArea(m_CurrentDragArea, screenPos, Vec2(50.0f, 50.0f), m_App));

			m_CurrentDragArea = Simulation::SimAreaType::NONE;
		}
	}
}