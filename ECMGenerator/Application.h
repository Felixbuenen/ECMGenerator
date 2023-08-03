#pragma once

// note: forward declare of Environment::TestEnvironment (enum) and ECMRenderer::ECMRendererColorSettings (struct) not possible
#include "Environment.h" 
#include "ECMRenderer.h"

class SDL_Window;

namespace ECM
{
	class ECMGenerator;
	class ECM;
	class ECMRendererColorSettings;
	class ECMCell;

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
		};

		class Application
		{		
		public:
			Application() { }

			bool InitializeApplication(const char* title, const char* environment_path, int screenWidth, int screenHeight, float zoomFactor = 0.65f);
			bool InitializeApplication(const char* title, Environment::TestEnvironment environment, int screenWidth, int screenHeight, float zoomFactor = 0.65f);
			void Run();
			void Clear();

			ApplicationState* GetApplicationState() { return &m_ApplicationState; }
			SDL_Window* GetWindow() { return m_Window; }


		private:
			bool InitializeEnvironment(const char* file);
			bool InitializeEnvironment(Environment::TestEnvironment environment);
			bool InitializeWindow(const char* title, int screenWidth, int screenHeight, float zoomFactor);
			bool InitializeRenderer();

			ApplicationState m_ApplicationState;
			ECMRenderer m_Renderer;
			SDL_Window* m_Window;
		};
	}
}