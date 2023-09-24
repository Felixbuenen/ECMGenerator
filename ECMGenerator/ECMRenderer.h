#pragma once

#include <memory>
#include <vector>
#include <stdint.h>

/*
* TODO:
* > Make an ECM window (or "ECM::Application") class and ECM window state class
* > In this, we handle all the input logic
* > The ECM window class will use the ECMRenderer to render graphics to the window.
* > The window state class will maintain a cached state of the application, e.g. the selected ECM cell, 
*    a start and goal position for the path planner.
*/

class SDL_Window;
class SDL_Renderer;
class SDL_Surface;

namespace ECM {

	class ECM;
	class Environment;
	struct Segment;
	struct ECMCell;

	namespace WindowApplication {

		class Application;
		struct ApplicationState;

		class ECMRenderer
		{
		public:
			struct ECMRendererColorSettings
			{
				int background_R = 0x22, background_G = 0x22, background_B = 0x22;
				int walkableArea_R = 0xff, walkableArea_G = 0xff, walkableArea_B = 0xff;
				int obstacle_R = 0xaa, obstacle_G = 0x11, obstacle_B = 0x11;
				// etc....
			};

		public:
			void Initialize(Application* app);
			void Render();
			void Clear();

		private:
			void InitializeRenderContext(int width, int height, const char* title);

			void UpdateRenderState();

			// different draw calls
			void SetupSDLWindow();
			void DrawBackground();
			void DrawWalkableArea();
			void DrawObstacles();
			void DrawMedialAxis();
			void DrawECMVertices();
			void HighlightECMVertex(int idx);
			void DrawPath(); // bij deze even kijken hoe ik dat ga doen, want het pad is in principe geen onderdeel van ECM, wellicht met ref van pad
			void DrawRandomTestPath();
			void DrawInsideVerts();
			void DrawClosestObstaclePoints();
			void DrawCorridor();
			void DrawPortals();
			void DrawHalfEdge(int idx);

			void DebugSetDrawECMCell(float screenX, float screenY);
			void DebugDrawECMCell();
			void DebugDrawSecondaryLines();
			void DebugDrawCellValues();

		private:
			std::shared_ptr<ECM> m_Ecm;
			Environment* m_Env;
			ECMRendererColorSettings m_ColorSettings;

			float m_CamZoomFactor;
			float m_YRotation;
			int m_CamOffsetX, m_CamOffsetY;

			ApplicationState* m_AppState;

			// SDL member variables
		private:
			SDL_Renderer* m_Renderer;
			SDL_Surface* m_ScreenSurface;
		};


	}
}