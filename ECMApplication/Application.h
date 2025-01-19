#pragma once

#include "Environment.h" 
#include "ECMRenderer.h"
#include "ECMPathPlanner.h"
#include "Simulator.h"
#include "AStar.h"
#include "SimAreaPanel.h"
#include "UndoRedoManager.h"
#include "EnvironmentEditor.h"

class SDL_Window;
typedef union SDL_Event;

namespace ECM
{
	class ECMGenerator;
	class ECM;
	class ECMRendererColorSettings;
	class ECMCell;

	struct Segment;

	namespace PathPlanning
	{
		typedef std::vector<Point> Path;
	}

	namespace WindowApplication
	{
		class EnvironmentEditor;

		struct ApplicationState
		{
			// simulation
			bool simulationPlaying = false;
			bool simulationPaused = true;
			
			// window state
			float camZoomFactor;
			float camOffsetX;
			float camOffsetY;
			
			// ECM state
			Environment* environment;
			Simulation::Simulator* simulator;
			std::shared_ptr<ECM> ecm;

			// misc
			const ECMCell* cellToDraw;
			SimAreaDrag dragAreaType;
			Point mousePosition;

			// path
			Point pathStartPoint;
			Point pathGoalPoint;
			bool startPointSelected;
			PathPlanning::Path pathToDraw;
			PathPlanning::Corridor corridorToDraw;
			std::vector<Segment> portalsToDraw;
		};

		class Application
		{
		public:
			Application(PathPlanning::ECMPathPlanner* planner, Environment* environment, Simulation::Simulator* simulator, int undoRedoStack=10) 
				: m_Planner(planner), m_UndoRedoManager(undoRedoStack), m_EnvEditor(this)
			{
				m_ApplicationState.environment = environment;
				m_ApplicationState.simulator = simulator;
			}

			bool InitializeApplication(const char* title, bool fullScreen, int screenWidth=0, int screenHeight=0);
			void Run();
			void Clear();

			ApplicationState* GetApplicationState() { return &m_ApplicationState; }
			SDL_Renderer* GetApplicationRenderer() { return m_Renderer; }
			ECMRenderer* GetECMRenderer() { return &m_ECMRenderer; }
			EnvironmentEditor* GetEnvironmentEditor() { return &m_EnvEditor; }
			Simulation::Simulator* GetSimulator() { return m_ApplicationState.simulator; }
			UndoRedoManager* GetUndoRedoManager() { return &m_UndoRedoManager; }
			SDL_Window* GetWindow() { return m_Window; }


		private:
			bool InitializeWindow(const char* title, bool fullScreen, int screenWidth, int screenHeight);
			bool InitializeRenderer();
			void HandleMouseEvent(SDL_Event& event);
			void HandleKeyEvent(SDL_Event& event);

			void HandlePlaceSimArea();
			
			bool HandleInput(SDL_Event& event);
			void Update(SDL_Event& event);
			void Render();

			UI m_UI;

			SDL_Renderer* m_Renderer;
			SDL_Window* m_Window;
			ECMRenderer m_ECMRenderer;
			UndoRedoManager m_UndoRedoManager;
			EnvironmentEditor m_EnvEditor;
			
			ApplicationState m_ApplicationState;
			PathPlanning::ECMPathPlanner* m_Planner;
			bool m_Quit;

			
			float m_DeltaTime;
		};
	}
}