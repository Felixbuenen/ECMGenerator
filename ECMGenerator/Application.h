#pragma once

// note: forward declare of Environment::TestEnvironment (enum) and ECMRenderer::ECMRendererColorSettings (struct) not possible
#include "Environment.h" 
#include "ECMRenderer.h"
#include "ECMPathPlanner.h"

class SDL_Window;

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
		struct ApplicationState
		{
			// window state
			float camZoomFactor;
			float camOffsetX;
			float camOffsetY;
			
			// ECM state
			Environment environment;
			std::shared_ptr<ECM> ecm;

			// misc
			const ECMCell* cellToDraw;

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
			bool InitializeApplication(const char* title, Environment& environment, int screenWidth, int screenHeight);
			void Run();
			void Clear();

			ApplicationState* GetApplicationState() { return &m_ApplicationState; }
			SDL_Window* GetWindow() { return m_Window; }


		private:
			bool InitializeEnvironment(const char* file);
			bool InitializeEnvironment(Environment::TestEnvironment environment);
			bool InitializeWindow(const char* title, int screenWidth, int screenHeight);
			bool InitializeRenderer();

			ApplicationState m_ApplicationState;
			ECMRenderer m_Renderer;
			SDL_Window* m_Window;
			PathPlanning::ECMPathPlanner m_Planner;
		};
	}
}