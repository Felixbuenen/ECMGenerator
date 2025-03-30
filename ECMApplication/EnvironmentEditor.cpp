#include "EnvironmentEditor.h"

#include "Application.h"
#include "Command.h"
#include "Simulator.h"
#include "Area.h"
#include "UtilityFunctions.h"
#include "ECMRenderer.h"

#include "SDL.h"

namespace ECM {

	namespace WindowApplication {

		EnvironmentEditor::~EnvironmentEditor()
		{
			delete m_SelectedAreaConnection;
		}

		void EnvironmentEditor::Initialize(Application* application)
		{
			m_App = application;
			m_CurrentDragArea = Simulation::SimAreaType::NONE;
			m_SelectedArea = nullptr;
			m_SelectedAreaConnection = new Simulation::AreaConnection();
			m_ActiveGizmo = nullptr;
			m_TransformMode = TRANSLATE;

			m_TranslateGizmo.Initialize(m_App->GetUndoRedoManager(), application->GetECMRenderer());
			m_ScaleGizmo.Initialize(m_App->GetUndoRedoManager(), application->GetECMRenderer());
			m_App->GetApplicationState()->selectedAreaConnection = m_SelectedAreaConnection;
		}

		void EnvironmentEditor::HandleInput(SDL_Event& e)
		{
			if (m_ActiveGizmo)
			{
				if (m_ActiveGizmo->HandleInput(e, m_App->GetECMRenderer())) return;
			}

			// EVENT: switch area editing gizmo
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

			// EVENT: quit connection mode
			if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE && m_App->GetApplicationState()->mode == ApplicationMode::CONNECT)
			{
				m_IsDraggingAreaConnection = false;
				m_App->GetApplicationState()->mode = ApplicationMode::BUILD;
				SwitchEditorState();
			}

			// EVENT: place simulation area
			if (e.type == SDL_MOUSEBUTTONDOWN 
				&& e.button.button == SDL_BUTTON_LEFT 
				&& m_CurrentDragArea != Simulation::SimAreaType::NONE 
				&& m_App->GetApplicationState()->mode == ApplicationMode::BUILD)
			{
				HandleDropArea(Point(e.button.x, e.button.y));
				return;
			}

			// EVENT: handle dragging connection between simulation areas
			if (m_App->GetApplicationState()->leftMouseDragging
				&& m_App->GetApplicationState()->mode == ApplicationMode::CONNECT)
			{
				SetDragAreaConnection();
				return;
			}

			// EVENT: left mouse button down
			if (e.type == SDL_MOUSEBUTTONDOWN 
				&& e.button.button == SDL_BUTTON_LEFT 
				&& m_CurrentDragArea == Simulation::SimAreaType::NONE)
			{
				HandleLeftButtonDownGeneral(Point(e.button.x, e.button.y));
				return;
			}

			// EVENT: left mouse button up
			if (e.type == SDL_MOUSEBUTTONUP
				&& e.button.button == SDL_BUTTON_LEFT)
			{
				HandleLeftButtonUpGeneral(Point(e.button.x, e.button.y));
				return;
			}
		}

		void EnvironmentEditor::Update()
		{

		}

		void EnvironmentEditor::Render()
		{
			RenderDragArea();
			RenderDragAreaConnection();
		}

		void EnvironmentEditor::SwitchEditorState()
		{
			if (m_App->GetApplicationState()->mode == ApplicationMode::CONNECT)
			{
				m_SelectedArea = nullptr;
				m_CurrentDragArea = Simulation::SimAreaType::NONE;
				m_ActiveGizmo = nullptr;
				m_App->GetECMRenderer()->RenderGizmo(nullptr);
			}
			
			if (m_App->GetApplicationState()->mode == ApplicationMode::BUILD)
			{
				m_ConnectionDragHoverArea = nullptr;
			}
			
		}

		void EnvironmentEditor::StartDragArea(Simulation::SimAreaType areaType)
		{
			m_CurrentDragArea = areaType;
		}

		void EnvironmentEditor::StopDragArea()
		{
			m_CurrentDragArea = Simulation::SimAreaType::NONE;
		}

		void EnvironmentEditor::RenderDragArea()
		{
			if (m_CurrentDragArea == Simulation::SimAreaType::NONE) return;

			// TODO: place in variables
			float halfWidth = 50;
			float halfHeight = 50;

			m_App->GetECMRenderer()->RenderDragArea(m_CurrentDragArea, m_App->GetApplicationState()->mousePosition, Vec2(halfWidth, halfHeight));
		}

		void EnvironmentEditor::RenderDragAreaConnection()
		{
			if (!m_IsDraggingAreaConnection) return;

			m_App->GetECMRenderer()->RenderDragConnection(m_SelectedArea->Position, m_ConnectionDragHoverArea);
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

			if (m_TransformMode == TRANSLATE) m_ActiveGizmo = &m_TranslateGizmo;
			else if (m_TransformMode == SCALE) m_ActiveGizmo = &m_ScaleGizmo;
			else return;

			m_SelectedArea = area;
			m_ActiveGizmo->SetActiveArea(area);
			m_App->GetECMRenderer()->RenderGizmo(m_ActiveGizmo);
		}

		void EnvironmentEditor::HandleLeftButtonDownGeneral(const Point& screenPos)
		{
			auto& spawnAreas = m_App->GetSimulator()->GetSpawnAreas();
			auto& goalAreas = m_App->GetSimulator()->GetGoalAreas();

			// if connection mode: only check if user clicked connections
			if (m_App->GetApplicationState()->mode == CONNECT)
			{
				const ECMRenderer* ecmRenderer = m_App->GetECMRenderer();
				const int MIN_SCREEN_DIST = 5;
				const int MIN_SCREEN_DIST_SQ = MIN_SCREEN_DIST*MIN_SCREEN_DIST;

				for (const auto& sa : spawnAreas)
				{
					auto& goalAreasConnected = sa.connectedGoalAreas;
					for (const auto& gaID : goalAreasConnected)
					{
						Point p1Screen = ecmRenderer->WorldToScreenCoordinates(sa.Position.x, sa.Position.y);
						Point p2Screen = ecmRenderer->WorldToScreenCoordinates(goalAreas[gaID].Position.x, goalAreas[gaID].Position.y);

						Point pointOnConnection = Utility::MathUtility::GetClosestPointOnSegment(screenPos, p1Screen, p2Screen);
						if (Utility::MathUtility::SquareDistance(screenPos, pointOnConnection) < MIN_SCREEN_DIST_SQ)
						{
							m_App->GetUndoRedoManager()->Invoke(new CMD_SelectAreaConnection(this, 
								Simulation::AreaConnection(m_SelectedAreaConnection->spawnID, m_SelectedAreaConnection->goalID), 
								Simulation::AreaConnection(sa.ID, gaID)));
							return;
						}
					}
				}

				// if nothing selected (and previously something was selected), deselect current selection
				if (m_SelectedAreaConnection->spawnID != -1)
				{
					m_App->GetUndoRedoManager()->Invoke(new CMD_SelectAreaConnection(this, 
						Simulation::AreaConnection(m_SelectedAreaConnection->spawnID, m_SelectedAreaConnection->goalID), 
						Simulation::AreaConnection()));
				}

				return;
			}

			SelectAreaAtPosition(screenPos);
		}

		void EnvironmentEditor::HandleLeftButtonUpGeneral(const Point& screenPos)
		{
			if (m_IsDraggingAreaConnection)
			{
				// check if mouse currently hovering over sim area. if so, establish connection
				if (m_ConnectionDragHoverArea)
				{
					if (m_SelectedArea->Type == Simulation::SimAreaType::SPAWN)
					{
						m_App->GetSimulator()->ConnectSpawnGoalAreas(m_SelectedArea->ID, m_ConnectionDragHoverArea->ID, 0.5f);
					}
					else
					{
						m_App->GetSimulator()->ConnectSpawnGoalAreas(m_ConnectionDragHoverArea->ID, m_SelectedArea->ID, 0.5f);
					}
				}

				m_SelectedArea = nullptr;
				SetDragAreaConnection(false);
			}
		}


		void EnvironmentEditor::HandleDropArea(const Point& screenPos)
		{
			m_App->GetUndoRedoManager()->Invoke(new CMD_AddSimulationArea(m_CurrentDragArea, screenPos, Vec2(50.0f, 50.0f), m_App));

			m_CurrentDragArea = Simulation::SimAreaType::NONE;
		}

		// check if user clicked spawn- or goal area
		void EnvironmentEditor::SelectAreaAtPosition(const Point& screenPos)
		{
			auto& spawnAreas = m_App->GetSimulator()->GetSpawnAreas();
			auto& goalAreas = m_App->GetSimulator()->GetGoalAreas();


			Point worldPos = m_App->GetECMRenderer()->ScreenToWorldCoordinates(screenPos.x, screenPos.y);

			bool foundIntersection = false;
			for (auto& a : spawnAreas)
			{
				// check if area intersects
				if (a.Intersects(worldPos))
				{
					if (m_SelectedArea == &a) return;

					foundIntersection = true;
					m_App->GetUndoRedoManager()->Invoke(new CMD_SelectArea(this, m_SelectedArea, &a));
				}
			}

			// TODO: refactor to avoid code duplication

			if (foundIntersection) return;
			for (auto& a : goalAreas)
			{
				// check if area intersects
				if (a.Intersects(worldPos))
				{
					if (m_SelectedArea == &a) return;

					foundIntersection = true;
					m_App->GetUndoRedoManager()->Invoke(new CMD_SelectArea(this, m_SelectedArea, &a));
				}
			}

			if (!foundIntersection && m_SelectedArea)
			{
				m_App->GetUndoRedoManager()->Invoke(new CMD_SelectArea(this, m_SelectedArea, nullptr));
			}
		}

		void EnvironmentEditor::SelectAreaConnection(Simulation::AreaConnection areaConnection)
		{
			m_SelectedAreaConnection->goalID = areaConnection.goalID;
			m_SelectedAreaConnection->spawnID = areaConnection.spawnID;
		}

		// handles the event when the user click-drags a connection between two simulation areas
		void EnvironmentEditor::SetDragAreaConnection(bool drag)
		{
			m_IsDraggingAreaConnection = drag;

			if (!drag) return;

			Point mouseScreenPos = m_App->GetApplicationState()->mousePosition;

			// first check if we are currently selecting an area, aka the source area for the connection
			if (!m_SelectedArea)
			{
				SelectAreaAtPosition(mouseScreenPos);

				// if user didn't select an area, there is no connection to be made
				if (!m_SelectedArea)
				{
					m_IsDraggingAreaConnection = false;
					return;
				}
			}

			// user is dragging a connection from the selected area source: check if user hovers over spawn- or goal area (depending on source area)

			Point mouseWorldPos = m_App->GetECMRenderer()->ScreenToWorldCoordinates(mouseScreenPos.x, mouseScreenPos.y);
			if (m_SelectedArea->Type == Simulation::SPAWN)
			{
				auto& goalAreas = m_App->GetSimulator()->GetGoalAreas();

				for (auto& ga : goalAreas)
				{
					if (ga.Intersects(mouseWorldPos))
					{
						m_ConnectionDragHoverArea = &ga;
						return;
					}
				}
			}
			else
			{
				auto& spawnAreas = m_App->GetSimulator()->GetSpawnAreas();

				for (auto& sa : spawnAreas)
				{
					if (sa.Intersects(mouseWorldPos))
					{
						m_ConnectionDragHoverArea = &sa;
						return;
					}
				}
			}

			// didn't find an intersection
			m_ConnectionDragHoverArea = nullptr;
		}

	}
}