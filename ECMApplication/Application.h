#pragma once

// note: forward declare of Environment::TestEnvironment (enum) and ECMRenderer::ECMRendererColorSettings (struct) not possible
#include "Environment.h" 
#include "ECMRenderer.h"
#include "ECMPathPlanner.h"
#include "Simulator.h"
#include "AStar.h"
#include "SimAreaPanel.h"

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
		//enum class SimAreaDrag;

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

			Point dragAreaPosition;
			SimAreaDrag dragAreaType;

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
			Application(PathPlanning::ECMPathPlanner* planner, Environment* environment, Simulation::Simulator* simulator) : m_Planner(planner)
			{
				m_ApplicationState.environment = environment;
				m_ApplicationState.simulator = simulator;
			}

			bool InitializeApplication(const char* title, int screenWidth, int screenHeight);
			void Run();
			void Clear();

			ApplicationState* GetApplicationState() { return &m_ApplicationState; }
			ECMRenderer* GetRenderer() { return &m_Renderer; }
			Simulation::Simulator* GetSimulator() { return m_ApplicationState.simulator; }
			SDL_Window* GetWindow() { return m_Window; }


		private:
			bool InitializeWindow(const char* title, int screenWidth, int screenHeight);
			bool InitializeRenderer();
			void HandleMouseEvent(SDL_Event& event);
			void HandleKeyEvent(SDL_Event& event);

			void CreateUI(SDL_Event& event);

			ApplicationState m_ApplicationState;
			ECMRenderer m_Renderer;
			SDL_Window* m_Window;
			PathPlanning::ECMPathPlanner* m_Planner;
			bool m_Quit;

			// TODO: encapsulate UI panels in a single ApplicationUI class
			SimAreaPanel m_AreaPanel;

			// UI state variables
			// TODO: create separate class for panels (e.g. SimAreaPanel, PlaybackPanel, PropertiesPanel..)
			bool connectorMode = false;
			bool buildMode = true;
		};
	}
}